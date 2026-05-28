#include "sensors.h"
#include <NimBLEDevice.h>

/* ==========================================================================
 *  PER-BOARD CONFIG  ->  the only two things that differ between the boards
 *  --------------------------------------------------------------------------
 *  BOARD 1 (Patient 1):  PATIENT_ID = 1   TARGET_HM19_MAC = "80:6f:b0:74:44:70"
 *  BOARD 2 (Patient 2):  PATIENT_ID = 2   TARGET_HM19_MAC = "80:6f:b0:6f:a8:8a"
 *
 *  Flash this same sketch to each ESP32-C3. Change ONLY the two #defines
 *  below before flashing each board. Everything else stays the same.
 * ==========================================================================*/

#define PATIENT_ID        1
#define TARGET_HM19_MAC   "80:6f:b0:74:44:70"   // this board talks to THIS HM-19 only

/* ===================== BLE GLOBALS ===================== */

static NimBLEAdvertisedDevice     *foundDevice = nullptr;
static NimBLEClient               *client      = nullptr;
static NimBLERemoteCharacteristic *chr         = nullptr;

static bool shouldConnect = false;
static bool connected     = false;


// ===================== ALERT BITMASK =====================

#define ALERT_HR_LOW    (1 << 0)
#define ALERT_HR_HIGH   (1 << 1)
#define ALERT_TEMP_HIGH (1 << 2)
#define ALERT_TEMP_LOW  (1 << 3)
#define ALERT_SPO2_LOW  (1 << 4)
#define ALERT_FALL      (1 << 5)   // not used (no motion sensor)

/* ===================== SAMPLE BUFFER ===================== */

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

    // match the HM-19 service AND the specific MAC for THIS board
    if (dev->isAdvertisingService(NimBLEUUID("FFE0")) &&
        dev->getAddress().toString() == TARGET_HM19_MAC) {

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
uint8_t analyze_hr(int bpm) {
  uint8_t f = 0;
  if (bpm < 60)  f |= ALERT_HR_LOW;
  if (bpm > 100) f |= ALERT_HR_HIGH;
  return f;
}

uint8_t analyze_temp(float tempC) {
  uint8_t f = 0;
  if (tempC > 38.0f) f |= ALERT_TEMP_HIGH;
  if (tempC < 35.0f) f |= ALERT_TEMP_LOW;
  return f;
}

uint8_t analyze_spo2(int spo2) {
  return (spo2 < 90) ? ALERT_SPO2_LOW : 0;
}


void buildPacket(char *out, size_t len) {

  char hrStr[40];
  snprintf(hrStr, sizeof(hrStr), "[%d,%d,%d]",
           hrBuf[0], hrBuf[1], hrBuf[2]);

  char spo2Str[40];
  snprintf(spo2Str, sizeof(spo2Str), "[%d,%d,%d]",
           spo2Buf[0], spo2Buf[1], spo2Buf[2]);

  // convert C -> F
  char tempStr[60];
  snprintf(tempStr, sizeof(tempStr), "[%.1f,%.1f,%.1f]",
           tempBuf[0] * 9.0f / 5.0f + 32.0f,
           tempBuf[1] * 9.0f / 5.0f + 32.0f,
           tempBuf[2] * 9.0f / 5.0f + 32.0f);

  // ===================== FLAG LOGIC =====================
  uint8_t flags = 0;

  // use latest sample in buffer (last written index - 1)
  int last = (sampleIndex == 0) ? SAMPLE_COUNT - 1 : sampleIndex - 1;

  flags |= analyze_hr(hrBuf[last]);
  flags |= analyze_spo2(spo2Buf[last]);

  // tempBuf is stored in C (before conversion in packet logic)
  flags |= analyze_temp(tempBuf[last]);

  // no motion sensor here → fall = 0
  uint8_t fall_flag = 0;

  char errStr[64];
  snprintf(errStr, sizeof(errStr),
           "[%d,%d,%d,%d,%d,%d]",
           (flags & ALERT_HR_LOW)    ? 1 : 0,
           (flags & ALERT_HR_HIGH)   ? 1 : 0,
           (flags & ALERT_TEMP_HIGH) ? 1 : 0,
           (flags & ALERT_TEMP_LOW)  ? 1 : 0,
           (flags & ALERT_SPO2_LOW)  ? 1 : 0,
           fall_flag);

  snprintf(out, len,
           "Patient:%d;HR:%s;SPO2:%s;TEMP:%s;ERR:%s\n",
           PATIENT_ID, hrStr, spo2Str, tempStr, errStr);
}

// void buildPacket(char *out, size_t len) {

//   char hrStr[40];
//   snprintf(hrStr, sizeof(hrStr), "[%d,%d,%d]",
//            hrBuf[0], hrBuf[1], hrBuf[2]);

//   char spo2Str[40];
//   snprintf(spo2Str, sizeof(spo2Str), "[%d,%d,%d]",
//            spo2Buf[0], spo2Buf[1], spo2Buf[2]);

//   // convert C -> F: F = C * 9/5 + 32
//   char tempStr[60];
//   snprintf(tempStr, sizeof(tempStr), "[%.1f, %.1f, %.1f]",
//            tempBuf[0] * 9.0f / 5.0f + 32.0f,
//            tempBuf[1] * 9.0f / 5.0f + 32.0f,
//            tempBuf[2] * 9.0f / 5.0f + 32.0f);

//   const char *errStr = "[0,0,0,0,0,0]";

//   snprintf(out, len,
//            "Patient:%d;HR:%s;SPO2:%s;TEMP:%s;ERR:%s\n",
//            PATIENT_ID, hrStr, spo2Str, tempStr, errStr);
// }

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
  Serial.printf("PATIENT_ID = %d  ->  HM-19 %s\n", PATIENT_ID, TARGET_HM19_MAC);
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
