#include "sensors.h"

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"          // <-- NEW: beat-detection

#define I2C_SDA 6
#define I2C_SCL 7

#define MPU6500_ADDR    0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_WHO_AM_I    0x75
#define MPU_ACCEL_XOUT_H 0x3B

#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f

#define BUFFER_LEN      100
#define REFRESH_SAMPLES  50

MAX30105 particleSensor;

uint32_t irBuffer[BUFFER_LEN];
uint32_t redBuffer[BUFFER_LEN];

int32_t heartRate  = 0;
int32_t spo2       = 0;
int8_t  hrValid    = 0;
int8_t  spo2Valid  = 0;

// =====================================================================
//  Beat-detection state  (replaces Maxim HR; lives here, not exposed)
// =====================================================================
static const byte HR_RATE_SIZE  = 8;
static byte       hrRates[HR_RATE_SIZE];
static byte       hrRateSpot    = 0;
static byte       hrRatesFilled = 0;
static uint32_t   hrLastBeat    = 0;   // set to millis() after init
static int        hrBeatAvg     = 0;

// =====================================================================
//  SpO2 rolling-buffer write cursor
//  After fillSpo2Baseline() fills indices 0..99 and runs the algo once,
//  tickSensors() continues writing from index 50, overwriting the older
//  half.  When the cursor reaches 100 the algo re-runs and the window
//  slides back to 50.
// =====================================================================
static int spoWriteIdx = BUFFER_LEN - REFRESH_SAMPLES;  // = 50

// =====================================================================

struct MotionReading {
  float ax, ay, az;
  float gx, gy, gz;
};

static void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static bool initMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("[ERROR] MAX30102 not found.");
    return false;
  }
  particleSensor.setup();
  byte ledCurrent = 0x3A;
  particleSensor.setPulseAmplitudeRed(ledCurrent);
  particleSensor.setPulseAmplitudeIR(ledCurrent);
  particleSensor.enableDIETEMPRDY();
  Serial.println("[OK] MAX30102 initialized");
  return true;
}

static bool initMPU6500() {
  mpuWriteReg(MPU_PWR_MGMT_1, 0x00);
  delay(100);

  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(MPU_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6500_ADDR, (uint8_t)1);

  if (Wire.available() < 1) {
    Serial.println("[ERROR] MPU WHO_AM_I failed.");
    return false;
  }
  uint8_t whoAmI = Wire.read();
  if (whoAmI != 0x70 && whoAmI != 0x68) {
    Serial.printf("[ERROR] MPU WHO_AM_I = 0x%02X\n", whoAmI);
    return false;
  }
  Serial.println("[OK] MPU initialized");
  return true;
}

// Blocking ONLY at startup — fills the SpO2 buffer once so the algo has
// a full window from the very first readSensors() call.
static void fillSpo2Baseline() {
  Serial.println("[MAX30102] Collecting baseline samples...");
  for (int i = 0; i < BUFFER_LEN; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
  maxim_heart_rate_and_oxygen_saturation(
      irBuffer, BUFFER_LEN, redBuffer,
      &spo2, &spo2Valid, &heartRate, &hrValid);
  // spoWriteIdx stays at 50: tickSensors() writes new data from there.
}

static bool readMPU(MotionReading &m) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom((uint8_t)MPU6500_ADDR, (uint8_t)14);
  if (Wire.available() < 14) return false;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();   // temp bytes — discard
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  m.ax = rawAx / ACCEL_SCALE;  m.ay = rawAy / ACCEL_SCALE;  m.az = rawAz / ACCEL_SCALE;
  m.gx = rawGx / GYRO_SCALE;   m.gy = rawGy / GYRO_SCALE;   m.gz = rawGz / GYRO_SCALE;
  return true;
}

// =====================================================================
//  initSensors  — unchanged except we anchor hrLastBeat after init
// =====================================================================
bool initSensors() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!initMAX30102()) return false;
  if (!initMPU6500())  return false;

  fillSpo2Baseline();

  hrLastBeat = millis();   // anchor so the very first inter-beat delta
                           // doesn't start from 0 (epoch)
  return true;
}

// =====================================================================
//  tickSensors  — call every loop() iteration (replaces updateVitals)
//
//  Drains however many samples the MAX30102 FIFO has ready.
//  For each sample:
//    • Runs checkForBeat() for HR  (40–120 BPM, 500 ms refractory)
//    • Pushes the sample into the SpO2 rolling buffer; re-runs the
//      Maxim SpO2 algorithm and slides the window every 50 new samples.
// =====================================================================
void tickSensors() {
  particleSensor.check();

  while (particleSensor.available()) {
    uint32_t irValue  = particleSensor.getIR();
    uint32_t redValue = particleSensor.getRed();
    particleSensor.nextSample();

    // ------------------------------------------------------------------
    //  HR  — beat-by-beat detection
    // ------------------------------------------------------------------
    if (irValue > 50000UL && checkForBeat((long)irValue)) {
      uint32_t now  = millis();
      long     delta = (long)(now - hrLastBeat);

      if (delta >= 500) {                           // refractory gate
        hrLastBeat = now;
        float bpm  = 60000.0f / (float)delta;

        if (bpm >= 40.0f && bpm <= 120.0f) {
          hrRates[hrRateSpot++] = (byte)bpm;
          hrRateSpot %= HR_RATE_SIZE;
          if (hrRatesFilled < HR_RATE_SIZE) hrRatesFilled++;

          hrBeatAvg = 0;
          for (byte i = 0; i < hrRatesFilled; i++) hrBeatAvg += hrRates[i];
          hrBeatAvg /= hrRatesFilled;

          Serial.printf("[HR] beat %.1f bpm  avg %d\n", bpm, hrBeatAvg);
        }
      }
    }

    // ------------------------------------------------------------------
    //  SpO2  — rolling buffer; re-run Maxim algo every REFRESH_SAMPLES
    // ------------------------------------------------------------------
    if (spoWriteIdx < BUFFER_LEN) {
      irBuffer[spoWriteIdx]  = irValue;
      redBuffer[spoWriteIdx] = redValue;
      spoWriteIdx++;
    }

    if (spoWriteIdx >= BUFFER_LEN) {
      maxim_heart_rate_and_oxygen_saturation(
          irBuffer, BUFFER_LEN, redBuffer,
          &spo2, &spo2Valid, &heartRate, &hrValid);

      // Slide window: keep the newer half
      for (int i = REFRESH_SAMPLES; i < BUFFER_LEN; i++) {
        irBuffer[i - REFRESH_SAMPLES]  = irBuffer[i];
        redBuffer[i - REFRESH_SAMPLES] = redBuffer[i];
      }
      spoWriteIdx = BUFFER_LEN - REFRESH_SAMPLES;   // = 50
    }
  }
}

// =====================================================================
//  readSensors  — now NON-BLOCKING
//  Just snapshots the values that tickSensors() keeps up-to-date.
// =====================================================================
bool readSensors(SensorPacket &data) {
  data.heartRate   = (hrRatesFilled > 0) ? hrBeatAvg : -1;
  data.spo2        = spo2Valid ? (int)spo2 : -1;
  data.temperature = particleSensor.readTemperature();

  MotionReading motion;
  if (!readMPU(motion)) {
    Serial.println("[WARN] MPU read failed.");
    motion.ax = motion.ay = motion.az = 0;
    motion.gx = motion.gy = motion.gz = 0;
  }

  data.ax = motion.ax;  data.ay = motion.ay;  data.az = motion.az;
  data.gx = motion.gx;  data.gy = motion.gy;  data.gz = motion.gz;

  return true;
}
