#include "sensors.h"
#include <NimBLEDevice.h>

unsigned long lastSend = 0;

/* ===================== BLE GLOBALS ===================== */

static NimBLEAdvertisedDevice     *foundDevice = nullptr;
static NimBLEClient               *client      = nullptr;
static NimBLERemoteCharacteristic *chr         = nullptr;

static bool shouldConnect = false;
static bool connected     = false;

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

void buildPacket(const SensorPacket &data, char *out, size_t len) {
  snprintf(
      out,
      len,
      "<VP1,%lu,%d,%d,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,00>\n",
      (unsigned long)millis(),
      data.heartRate,
      data.spo2,
      data.temperature,
      data.ax,
      data.ay,
      data.az,
      data.gx,
      data.gy,
      data.gz);
}

/* ===================== SEND ===================== */

void sendPacket(const SensorPacket &data) {

  if (!connected || !client || !client->isConnected() || !chr) return;

  char packet[200];
  memset(packet, 0, sizeof(packet));
  buildPacket(data, packet, sizeof(packet));

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

  initSensors();  // initialize MAX30102 + MPU6500

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

  if (connected && millis() - lastSend >= 1000) {
    lastSend = millis();
    SensorPacket data;
    if (readSensors(data)) {
      sendPacket(data);
    }
  }

  delay(20);
}