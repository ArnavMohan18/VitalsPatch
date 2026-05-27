#include "sensors.h"
#include <NimBLEDevice.h>

/* ===================== BLE GLOBALS ===================== */

static NimBLEAdvertisedDevice     *foundDevice = nullptr;
static NimBLEClient               *client      = nullptr;
static NimBLERemoteCharacteristic *chr         = nullptr;

static bool shouldConnect = false;
static bool connected     = false;

/* ===================== SAMPLE BUFFER ===================== */

#define PATIENT_ID   1
#define SAMPLE_COUNT 3

int   hrBuf[SAMPLE_COUNT];
int   spo2Buf[SAMPLE_COUNT];
float tempBuf[SAMPLE_COUNT];

int      sampleIndex = 0;
uint32_t lastSample  = 0;

/* ===================== SCAN CALLBACK ===================== */

class ScanCallbacks : public NimBLEScanCallbacks {

  void onResult(const NimBLEAdvertisedDevice *dev) override {

    Serial.print("[SCAN] Device: ");
    Serial.println(dev->toString().c_str());

    if (dev->isAdvertisingService(NimBLEUUID("FFE0")) &&
        dev->getAddress().toString() == "80:6f:b0:74:44:70") {

      Serial.println("[BLE] TARGET HM-19 FOUND");

      delete foundDevice;
      foundDevice   = new NimBLEAdvertisedDevice(*dev);
      shouldConnect = true;
      NimBLEDevice::getScan()->stop();
    }
  }
};

/* ===================== CONNECT ===================== */

void connectToHM19() {

  shouldConnect = false;

  if (client != nullptr) {
    NimBLEDevice::deleteClient(client);
    client = nullptr;
  }

  if (foundDevice == nullptr) {
    Serial.println("[BLE] foundDevice is null — restarting scan");
    NimBLEDevice::getScan()->start(0, false);
    return;
  }

  Serial.println("[BLE] Attempting connection...");

  client = NimBLEDevice::createClient();

  if (!client->connect(foundDevice)) {
    Serial.println("[BLE] CONNECT FAILED");
    NimBLEDevice::deleteClient(client);
    client    = nullptr;
    connected = false;
    delete foundDevice;
    foundDevice = nullptr;
    NimBLEDevice::getScan()->start(0, false);
    return;
  }

  Serial.println("[BLE] CONNECTED");

  NimBLERemoteService *svc = client->getService("FFE0");
  if (!svc) {
    Serial.println("[BLE] Service FFE0 not found");
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    client = nullptr;
    delete foundDevice;
    foundDevice = nullptr;
    NimBLEDevice::getScan()->start(0, false);
    return;
  }

  chr = svc->getCharacteristic("FFE1");
  if (!chr) {
    Serial.println("[BLE] Characteristic FFE1 not found");
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    client = nullptr;
    delete foundDevice;
    foundDevice = nullptr;
    NimBLEDevice::getScan()->start(0, false);
    return;
  }

  delete foundDevice;
  foundDevice = nullptr;

  connected = true;
  Serial.println("[BLE] READY TO SEND DATA");
}

/* ===================== BUILD PACKET ===================== */

void buildPacket(char *out, size_t len) {

  char hrStr[40];
  snprintf(hrStr, sizeof(hrStr), "[%d,%d,%d]",
           hrBuf[0], hrBuf[1], hrBuf[2]);

  char spo2Str[40];
  snprintf(spo2Str, sizeof(spo2Str), "[%d,%d,%d]",
           spo2Buf[0], spo2Buf[1], spo2Buf[2]);

  // convert C -> F: F = C * 9/5 + 32
  char tempStr[60];
  snprintf(tempStr, sizeof(tempStr), "[%.1f, %.1f, %.1f]",
           tempBuf[0] * 9.0f / 5.0f + 32.0f,
           tempBuf[1] * 9.0f / 5.0f + 32.0f,
           tempBuf[2] * 9.0f / 5.0f + 32.0f);

  const char *errStr = "[0,0,0,0,0,0]";

  snprintf(out, len,
           "Patient:%d;HR:%s;SPO2:%s;TEMP:%s;ERR:%s\n",
           PATIENT_ID, hrStr, spo2Str, tempStr, errStr);
}

/* ===================== SEND ===================== */

void sendPacket() {

  if (!connected || !client || !client->isConnected() || !chr) return;

  char packet[200];
  memset(packet, 0, sizeof(packet));
  buildPacket(packet, sizeof(packet));

  Serial.print("[TX] ");
  Serial.print(packet);

  bool ok = chr->writeValue(
      (uint8_t *)packet,
      strlen(packet),
      false);

  if (!ok) Serial.println("[BLE] WRITE FAILED");
}

/* ===================== SETUP ===================== */

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=================================");
  Serial.println("ESP32-C3 HM-19 SENSOR SIMULATOR");
  Serial.println("=================================");

  if (!initSensors()) {
    Serial.println("[ERROR] Sensor init failed — halting");
    while (true) delay(1000);
  }

  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(false, false, false);

  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new ScanCallbacks(), false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(80);

  Serial.println("[BLE] Starting scan...");
  scan->start(0, false);
}

/* ===================== LOOP ===================== */

void loop() {

  if (shouldConnect && !connected) {
    delay(300);
    connectToHM19();
  }

  if (connected && client && !client->isConnected()) {
    Serial.println("[BLE] DISCONNECTED");
    connected = false;
    chr       = nullptr;
    NimBLEDevice::deleteClient(client);
    client    = nullptr;
    NimBLEDevice::getScan()->start(0, false);
  }

  // collect one real sensor reading per second
  if (connected && millis() - lastSample >= 1000) {
    lastSample = millis();

    SensorPacket data;
    if (readSensors(data)) {

      hrBuf[sampleIndex]   = data.heartRate;
      spo2Buf[sampleIndex] = data.spo2;
      tempBuf[sampleIndex] = data.temperature;

      sampleIndex++;

      // buffer full — send and reset
      if (sampleIndex >= SAMPLE_COUNT) {
        sendPacket();
        sampleIndex = 0;
      }
    }
  }

  delay(20);
}