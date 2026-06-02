#include "sensors.h"

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#define I2C_SDA 6
#define I2C_SCL 7

#define MPU6500_ADDR 0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_WHO_AM_I 0x75
#define MPU_ACCEL_XOUT_H 0x3B

#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE 131.0f

#define BUFFER_LEN 100
#define REFRESH_SAMPLES 50

MAX30105 particleSensor;

uint32_t irBuffer[BUFFER_LEN];
uint32_t redBuffer[BUFFER_LEN];

int32_t heartRate = 0;
int32_t spo2 = 0;

int8_t hrValid = 0;
int8_t spo2Valid = 0;

struct MotionReading {

  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;
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

static void fillSpo2Baseline() {

  Serial.println("[MAX30102] Collecting baseline samples...");

  for (int i = 0; i < BUFFER_LEN; i++) {

    while (!particleSensor.available()) {
      particleSensor.check();
    }

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();

    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,
    BUFFER_LEN,
    redBuffer,
    &spo2,
    &spo2Valid,
    &heartRate,
    &hrValid
  );
}

static void updateVitals() {

  for (int i = REFRESH_SAMPLES; i < BUFFER_LEN; i++) {

    redBuffer[i - REFRESH_SAMPLES] = redBuffer[i];
    irBuffer[i - REFRESH_SAMPLES] = irBuffer[i];
  }

  for (int i = BUFFER_LEN - REFRESH_SAMPLES; i < BUFFER_LEN; i++) {

    while (!particleSensor.available()) {
      particleSensor.check();
    }

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();

    // Serial.printf(
    //   "i=%d IR=%lu RED=%lu\n",
    //   i,
    //   irBuffer[i],
    //   redBuffer[i]
    // );
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,
    BUFFER_LEN,
    redBuffer,
    &spo2,
    &spo2Valid,
    &heartRate,
    &hrValid
  );

  Serial.printf(
    "HR=%ld valid=%d  SPO2=%ld valid=%d\n",
    heartRate,
    hrValid,
    spo2,
    spo2Valid
  );
}

static bool readMPU(MotionReading &m) {

  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom((uint8_t)MPU6500_ADDR, (uint8_t)14);

  if (Wire.available() < 14) {
    return false;
  }

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  m.ax = rawAx / ACCEL_SCALE;
  m.ay = rawAy / ACCEL_SCALE;
  m.az = rawAz / ACCEL_SCALE;

  m.gx = rawGx / GYRO_SCALE;
  m.gy = rawGy / GYRO_SCALE;
  m.gz = rawGz / GYRO_SCALE;

  return true;
}

bool initSensors() {

  Wire.begin(I2C_SDA, I2C_SCL);

  Wire.setClock(400000);

  if (!initMAX30102()) {
    return false;
  }

  if (!initMPU6500()) {
    return false;
  }

  fillSpo2Baseline();

  return true;
}

bool readSensors(SensorPacket &data) {

  updateVitals();

  MotionReading motion;

  if (!readMPU(motion)) {

    Serial.println("[WARN] MPU read failed.");

    motion.ax = 0;
    motion.ay = 0;
    motion.az = 0;

    motion.gx = 0;
    motion.gy = 0;
    motion.gz = 0;
  }

  data.heartRate = hrValid ? heartRate : -1;
  data.spo2 = spo2Valid ? spo2 : -1;

  data.temperature = particleSensor.readTemperature();

  data.ax = motion.ax;
  data.ay = motion.ay;
  data.az = motion.az;

  data.gx = motion.gx;
  data.gy = motion.gy;
  data.gz = motion.gz;

  return true;
}