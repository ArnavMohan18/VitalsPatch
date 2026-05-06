/*
 * VitalsPatch - Phase 1 Sensor/Data Acquisition Module
 *
 * Reads MAX30102 and MPU-6500 values, computes basic HR/SpO2 values with the
 * SparkFun algorithm, then sends newline-terminated packets to the STM32
 * through the HM-19 BLE UART service.
 *
 * XIAO ESP32-C3 wiring:
 *   SDA -> GPIO 6
 *   SCL -> GPIO 7
 *   HM-19 BLE peripheral is connected to the STM32 UART side.
 *
 * Libraries:
 *   - NimBLE-Arduino
 *   - SparkFun MAX3010x Pulse and Proximity Sensor Library
 */

#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <NimBLEDevice.h>
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
#define REFRESH_SAMPLES 25
#define SEND_INTERVAL_MS 1000

static const char *HM19_ADDRESS = "80:6f:b0:74:44:70";
static const char *HM19_SERVICE_UUID = "FFE0";
static const char *HM19_CHAR_UUID = "FFE1";

MAX30105 particleSensor;

uint32_t irBuffer[BUFFER_LEN];
uint32_t redBuffer[BUFFER_LEN];
int32_t heartRate = 0;
int32_t spo2 = 0;
int8_t hrValid = 0;
int8_t spo2Valid = 0;

static NimBLEAdvertisedDevice *foundDevice = nullptr;
static NimBLEClient *bleClient = nullptr;
static NimBLERemoteCharacteristic *bleChar = nullptr;
static bool shouldConnect = false;
static bool connected = false;

static uint32_t lastSendMs = 0;

struct MotionReading {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *advertisedDevice) {
    if (advertisedDevice->getAddress().toString() == HM19_ADDRESS) {
      Serial.println("[BLE] HM-19 found. Stopping scan.");
      foundDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
      shouldConnect = true;
      NimBLEDevice::getScan()->stop();
    }
  }
};

void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuRead(MotionReading &m) {
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

bool initMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("[ERROR] MAX30102 not found. Check wiring.");
    return false;
  }

  particleSensor.setup(60, 4, 2, 50, 411, 4096);
  particleSensor.enableDIETEMPRDY();
  Serial.println("[OK] MAX30102 found");
  return true;
}

bool initMPU6500() {
  mpuWriteReg(MPU_PWR_MGMT_1, 0x00);
  delay(100);

  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(MPU_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6500_ADDR, (uint8_t)1);

  if (Wire.available() < 1) {
    Serial.println("[ERROR] MPU WHO_AM_I read failed.");
    return false;
  }

  uint8_t whoAmI = Wire.read();
  if (whoAmI != 0x70 && whoAmI != 0x68) {
    Serial.printf("[ERROR] MPU WHO_AM_I = 0x%02X. Check wiring/AD0.\n", whoAmI);
    return false;
  }

  Serial.println("[OK] MPU-6500/MPU-6050 compatible IMU found");
  return true;
}

void fillSpo2Baseline() {
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
    irBuffer, BUFFER_LEN, redBuffer, &spo2, &spo2Valid, &heartRate, &hrValid
  );
}

void updateVitals() {
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
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_LEN, redBuffer, &spo2, &spo2Valid, &heartRate, &hrValid
  );
}

void startBleScan() {
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new ScanCallbacks(), false);
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(50);
  Serial.println("[BLE] Scanning for HM-19...");
  scan->start(0, false);
}

void connectToHM19() {
  shouldConnect = false;
  NimBLEDevice::getScan()->clearResults();

  bleClient = NimBLEDevice::createClient();
  Serial.println("[BLE] Connecting to HM-19...");

  if (!bleClient->connect(foundDevice)) {
    Serial.println("[BLE] Connect failed.");
    NimBLEDevice::deleteClient(bleClient);
    bleClient = nullptr;
    connected = false;
    startBleScan();
    return;
  }

  NimBLERemoteService *service = bleClient->getService(HM19_SERVICE_UUID);
  if (!service) {
    Serial.println("[BLE] FFE0 service not found.");
    bleClient->disconnect();
    connected = false;
    startBleScan();
    return;
  }

  bleChar = service->getCharacteristic(HM19_CHAR_UUID);
  if (!bleChar) {
    Serial.println("[BLE] FFE1 characteristic not found.");
    bleClient->disconnect();
    connected = false;
    startBleScan();
    return;
  }

  connected = true;
  Serial.println("[BLE] Connected. Ready to send sensor packets.");
}

uint8_t checksumBeforeLastComma(const char *packet) {
  uint8_t checksum = 0;
  const char *lastComma = strrchr(packet, ',');
  if (lastComma == nullptr) {
    return 0;
  }

  for (const char *p = packet; p < lastComma; p++) {
    checksum ^= (uint8_t)(*p);
  }
  return checksum;
}

bool sendPacket(const MotionReading &m, float tempC) {
  int32_t hrOut = hrValid ? heartRate : -1;
  int32_t spo2Out = spo2Valid ? spo2 : -1;

  char packet[180];
  snprintf(packet, sizeof(packet),
           "VP1,%lu,%ld,%ld,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,00",
           (unsigned long)millis(),
           (long)hrOut,
           (long)spo2Out,
           tempC,
           m.ax, m.ay, m.az,
           m.gx, m.gy, m.gz);

  uint8_t checksum = checksumBeforeLastComma(packet);
  char checksumText[5];
  snprintf(checksumText, sizeof(checksumText), "%02X\n", checksum);

  char *lastComma = strrchr(packet, ',');
  if (lastComma == nullptr) {
    return false;
  }
  *(lastComma + 1) = '\0';
  strncat(packet, checksumText, sizeof(packet) - strlen(packet) - 1);

  Serial.print("[TX] ");
  Serial.print(packet);

  if (!connected || bleClient == nullptr || !bleClient->isConnected() || bleChar == nullptr) {
    Serial.println("[BLE] Not connected; packet printed only.");
    connected = false;
    return false;
  }

  size_t packetLen = strlen(packet);
  for (size_t offset = 0; offset < packetLen; offset += 18) {
    size_t chunkLen = packetLen - offset;
    if (chunkLen > 18) {
      chunkLen = 18;
    }
    if (!bleChar->writeValue((uint8_t *)(packet + offset), chunkLen, false)) {
      return false;
    }
    delay(10);
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== VitalsPatch Phase 1 Sensor Module ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!initMAX30102() || !initMPU6500()) {
    while (1) {
      delay(1000);
    }
  }

  fillSpo2Baseline();

  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  startBleScan();
}

void loop() {
  if (shouldConnect && !connected) {
    delay(500);
    connectToHM19();
  }

  if (bleClient != nullptr && connected && !bleClient->isConnected()) {
    Serial.println("[BLE] Disconnected. Restarting scan.");
    connected = false;
    bleChar = nullptr;
    startBleScan();
  }

  uint32_t now = millis();
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    updateVitals();

    MotionReading motion = {0};
    bool imuOk = mpuRead(motion);
    float tempC = particleSensor.readTemperature();

    if (!imuOk) {
      Serial.println("[WARN] IMU read failed; sending zeros for motion.");
    }

    sendPacket(motion, tempC);
  }

  delay(25);
}
