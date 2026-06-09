/**
 * @file sensors.cpp
 * @brief Sensor acquisition and processing for the VitalsPatch wearable system.
 *
 * Handles:
 * - MAX30102 heart rate, SpO2, and temperature sensing
 * - MPU6500 motion sensing
 * - Fall detection
 * - Non-blocking sensor updates and data caching
 *
 * @author Eeshani Shilamkar, Maya Desai, Arnav Mohan
 * @author VitalsPatch Team
 */

#include "sensors.h"
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

#define I2C_SDA  6
#define I2C_SCL  7

// MPU6500 I2C address and register map
#define MPU6500_ADDR      0x68
#define MPU_PWR_MGMT_1    0x6B
#define MPU_WHO_AM_I      0x75
#define MPU_ACCEL_XOUT_H  0x3B

// Sensitivity divisors from MPU6500 datasheet (±2g, ±250°/s defaults)
#define ACCEL_SCALE       16384.0f
#define GYRO_SCALE        131.0f

struct MotionReading { float ax, ay, az, gx, gy, gz; };

static MAX30105 particleSensor;

// SpO2 uses a 100-sample rolling buffer; the Maxim algorithm re-runs every 50 new samples
#define BUFFER_LEN       100
#define REFRESH_SAMPLES   50
static uint32_t irBuffer[BUFFER_LEN];
static uint32_t redBuffer[BUFFER_LEN];
static int32_t  spo2      = -1;
static int8_t   spo2Valid = 0;
static int      spoWriteIdx = BUFFER_LEN - REFRESH_SAMPLES;  // start halfway so the first run is quick

// 8-sample rolling average for instantaneous BPM, plus a 20-second filtered window
#define RATE_SIZE     8
#define BPM_WIN_SIZE 60
static byte     rates[RATE_SIZE];
static byte     rateSpot      = 0;
static long     lastBeat      = 0;
static byte     ratesFilled   = 0;
static int      beatAvg       = 0;
static float    bpmWindow[BPM_WIN_SIZE];
static byte     bpmWindowCount = 0;
static uint32_t bpmWindowStart = 0;
#define BPM_WINDOW_MS  20000UL   // print filtered BPM every 20 s

// Temperature is cached to avoid the ~29 ms blocking read every loop iteration
static float    cachedTemp   = 0.0f;
static uint32_t lastTempRead = 0;
#define TEMP_CACHE_MS  2000UL    // refresh every 2 s

// MPU is polled at 50 Hz — enough for fall detection without flooding I2C
static MotionReading latestMotion = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
static uint32_t lastMpuPoll  = 0;
#define MPU_POLL_MS   20UL

// Fall detection: look for a free-fall phase followed by a sharp impact within 500 ms
static bool     fallFlag      = false;
static bool     inFreeFall    = false;
static uint32_t freeFallStart = 0;
#define FREE_FALL_G    0.5f    // accel magnitude below this triggers free-fall phase
#define IMPACT_G       2.5f   // accel magnitude above this confirms impact
#define FALL_WINDOW_MS 500UL  // impact must follow free-fall within this window

// ─── Internal helpers ────────────────────────────────────────────────────────

static void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static bool initMPU6500() {
  mpuWriteReg(MPU_PWR_MGMT_1, 0x00);  // wake the MPU out of sleep mode
  delay(100);

  // Verify chip identity before trusting any readings
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(MPU_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6500_ADDR, (uint8_t)1);
  if (Wire.available() < 1) { Serial.println("[ERROR] MPU WHO_AM_I failed."); return false; }
  uint8_t id = Wire.read();
  if (id != 0x70 && id != 0x68) { Serial.printf("[ERROR] MPU WHO_AM_I = 0x%02X\n", id); return false; }
  Serial.println("[OK] MPU initialized");
  return true;
}

// Read all 6 axes (accel + gyro) in one 14-byte burst for efficiency
static bool readMPU(MotionReading &m) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((uint8_t)MPU6500_ADDR, (uint8_t)14);
  if (Wire.available() < 14) return false;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();   // die-temperature bytes — not used here
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  // Convert raw counts to g and °/s
  m.ax = rawAx / ACCEL_SCALE;  m.ay = rawAy / ACCEL_SCALE;  m.az = rawAz / ACCEL_SCALE;
  m.gx = rawGx / GYRO_SCALE;   m.gy = rawGy / GYRO_SCALE;   m.gz = rawGz / GYRO_SCALE;
  return true;
}

static bool initMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("[ERROR] MAX30102 not found."); return false;
  }
  // ledBrightness=60, sampleAvg=1, ledMode=2(R+IR), sampleRate=100, pulseWidth=411, adcRange=4096
  particleSensor.setup(60, 1, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x3A);
  particleSensor.setPulseAmplitudeIR(0x3A);
  particleSensor.setPulseAmplitudeGreen(0);    // green LED off — not needed
  particleSensor.enableDIETEMPRDY();
  Serial.println("[OK] MAX30102 initialized");
  return true;
}

/**
 * @brief Fills the SpO2 buffer with 100 real samples at startup.
 *
 * This is the only blocking call in the sensor module. Running the Maxim
 * algorithm requires a full 100-sample buffer, so we collect them once here
 * rather than waiting for tickSensors() to accumulate them gradually.
 */
static void fillSpo2Baseline() {
  Serial.println("[MAX30102] Collecting baseline...");
  for (int i = 0; i < BUFFER_LEN; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
  int32_t dHR; int8_t dHRv;
  maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_LEN, redBuffer,
                                          &spo2, &spo2Valid, &dHR, &dHRv);
  // spoWriteIdx stays at 50 so tickSensors() fills the second half first
}

/**
 * @brief Returns a mean BPM from the 20-second window, rejecting outliers.
 *
 * Computes mean and std dev, then averages only samples within 2 std devs.
 * Falls back to the plain mean if all samples are outliers.
 */
static float filteredBpmMean() {
  if (bpmWindowCount == 0) return -1.0f;
  float sum = 0;
  for (byte i = 0; i < bpmWindowCount; i++) sum += bpmWindow[i];
  float mean = sum / bpmWindowCount;
  float var  = 0;
  for (byte i = 0; i < bpmWindowCount; i++) var += (bpmWindow[i]-mean)*(bpmWindow[i]-mean);
  float stddev = sqrtf(var / bpmWindowCount);
  float fSum = 0; byte fCount = 0;
  for (byte i = 0; i < bpmWindowCount; i++)
    if (fabsf(bpmWindow[i]-mean) <= 2.0f*stddev) { fSum += bpmWindow[i]; fCount++; }
  return (fCount > 0) ? fSum / fCount : mean;
}

// ─── Public API ──────────────────────────────────────────────────────────────

/**
 * @brief Initializes I2C, MAX30102, and MPU6500, then fills the SpO2 baseline.
 * @return true if all hardware initialized successfully.
 */
bool initSensors() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // 400 kHz fast mode
  if (!initMAX30102()) return false;
  if (!initMPU6500())  return false;
  fillSpo2Baseline();
  lastBeat       = millis();
  bpmWindowStart = millis();
  return true;
}

/**
 * @brief Non-blocking sensor tick — call every loop() iteration.
 *
 * Drains the MAX30102 FIFO, updates beat detection and the SpO2 rolling buffer,
 * refreshes the temperature cache, polls the MPU at 50 Hz, and runs the
 * two-phase fall detection state machine.
 */
void tickSensors() {

  // ── MAX30102: drain FIFO ─────────────────────────────────────────────────
  particleSensor.check();
  while (particleSensor.available()) {
    uint32_t ir  = particleSensor.getIR();
    uint32_t red = particleSensor.getRed();
    particleSensor.nextSample();

    // Write to rolling buffer; when it fills, run the Maxim SpO2 algorithm
    // then shift the buffer back by REFRESH_SAMPLES to reuse the old data
    irBuffer[spoWriteIdx]  = ir;
    redBuffer[spoWriteIdx] = red;
    if (++spoWriteIdx >= BUFFER_LEN) {
      int32_t dHR; int8_t dHRv;
      maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_LEN, redBuffer,
                                              &spo2, &spo2Valid, &dHR, &dHRv);
      for (int i = REFRESH_SAMPLES; i < BUFFER_LEN; i++) {
        irBuffer[i - REFRESH_SAMPLES]  = irBuffer[i];
        redBuffer[i - REFRESH_SAMPLES] = redBuffer[i];
      }
      spoWriteIdx = BUFFER_LEN - REFRESH_SAMPLES;
    }

    // Beat detection — ignore noise below 50k IR counts
    if (ir > 50000UL && checkForBeat((long)ir)) {
      long delta = millis() - lastBeat;
      if (delta >= 500) {  // debounce: ignore beats faster than 120 BPM
        lastBeat = millis();
        float bpm = 60000.0f / (float)delta;
        if (bpm >= 40.0f && bpm <= 120.0f) {
          rates[rateSpot++] = (byte)bpm;
          rateSpot %= RATE_SIZE;
          if (ratesFilled < RATE_SIZE) ratesFilled++;
          int s = 0;
          for (byte x = 0; x < ratesFilled; x++) s += rates[x];
          beatAvg = s / ratesFilled;
          if (bpmWindowCount < BPM_WIN_SIZE) bpmWindow[bpmWindowCount++] = bpm;
        }
      }
    }
  }

  // ── Temperature: refresh cache every 2 s ─────────────────────────────────
  if (millis() - lastTempRead >= TEMP_CACHE_MS) {
    float t = particleSensor.readTemperature();
    if (t > -40.0f && t < 85.0f) cachedTemp = t;  // sanity check before caching
    lastTempRead = millis();
  }

  // ── MPU6500 + fall detection at 50 Hz ────────────────────────────────────
  if (millis() - lastMpuPoll >= MPU_POLL_MS) {
    lastMpuPoll = millis();
    MotionReading m;
    if (readMPU(m)) {
      latestMotion = m;
      float mag = sqrtf(m.ax*m.ax + m.ay*m.ay + m.az*m.az);

      // Phase 1: detect free-fall (near-zero g)
      if (!inFreeFall && mag < FREE_FALL_G) {
        inFreeFall    = true;
        freeFallStart = millis();
      } else if (inFreeFall) {
        // Phase 2: look for impact within FALL_WINDOW_MS
        if (mag > IMPACT_G && millis() - freeFallStart <= FALL_WINDOW_MS) {
          fallFlag   = true;
          inFreeFall = false;
          Serial.println("[FALL] Detected!");
        } else if (millis() - freeFallStart > FALL_WINDOW_MS) {
          inFreeFall = false;   // window expired — not a fall
        }
      }
    }
  }

  // ── 20-second BPM window diagnostic ─────────────────────────────────────
  if (millis() - bpmWindowStart >= BPM_WINDOW_MS) {
    float fMean = filteredBpmMean();
    if (fMean > 0.0f)
      Serial.printf("[BPM] 20s filtered=%.1f  rolling=%d\n", fMean, beatAvg);
    else
      Serial.println("[BPM] 20s window: no valid beats");
    bpmWindowCount = 0;
    bpmWindowStart = millis();
  }
}

/**
 * @brief Reads the latest sensor values into a SensorPacket.
 *
 * Returns -1 for HR/SpO2 if no valid readings are available yet.
 * fallDetected is a one-shot flag — cleared after this call.
 */
bool readSensors(SensorPacket &data) {
  data.heartRate   = (ratesFilled > 0) ? beatAvg : -1;
  data.spo2        = spo2Valid ? (int32_t)spo2 : -1;
  data.temperature = cachedTemp;
  data.ax = latestMotion.ax;  data.ay = latestMotion.ay;  data.az = latestMotion.az;
  data.gx = latestMotion.gx;  data.gy = latestMotion.gy;  data.gz = latestMotion.gz;
  data.fallDetected = fallFlag;
  fallFlag = false;   // consumed — caller is responsible for acting on it
  return true;
}