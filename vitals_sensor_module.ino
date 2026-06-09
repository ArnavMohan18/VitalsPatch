#include "sensors.h"
#include <NimBLEDevice.h>

//maya patient 2
#define PATIENT_ID       2
#define TARGET_HM19_MAC  "80:6f:b0:74:44:70"

/* ===================== BLE GLOBALS ===================== */
static NimBLEAdvertisedDevice     *foundDevice = nullptr;
static NimBLEClient               *client      = nullptr;
static NimBLERemoteCharacteristic *chr         = nullptr;
static bool shouldConnect = false;
static bool connected     = false;

/* ===================== ALERT BITMASK ===================== */
#define ALERT_HR_HIGH   (1 << 0)
#define ALERT_HR_LOW    (1 << 1)
#define ALERT_TEMP_HIGH (1 << 2)
#define ALERT_TEMP_LOW  (1 << 3)
#define ALERT_SPO2_LOW  (1 << 4)
#define ALERT_FALL      (1 << 5)

/* ===================== TIMING ===================== */
#define VITAL_WINDOW_MS  20000UL          // one vital packet per 20 s
#define VITAL_SLOTS      3                // samples spread across the window
#define VITAL_SAMPLE_MS  (VITAL_WINDOW_MS / VITAL_SLOTS)   // ≈ 6667 ms
#define FALL_CHECK_MS    200UL            // fall-flag poll rate

#define SAMPLE_COUNT     VITAL_SLOTS      // keeps buildPacket signature stable

int     hrBuf[SAMPLE_COUNT];
int     spo2Buf[SAMPLE_COUNT];
float   tempBuf[SAMPLE_COUNT];           // stored in °C; converted in buildPacket

uint32_t lastVitalSample = 0;
uint32_t lastVitalSend   = 0;
uint32_t lastFallCheck   = 0;
int      vitalSlot       = 0;

/* ===================== SCAN CALLBACK ===================== */
class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *dev) override {
    Serial.print("[SCAN] Device: ");
    Serial.println(dev->toString().c_str());
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
  if (client != nullptr) { NimBLEDevice::deleteClient(client); client = nullptr; }
  if (foundDevice == nullptr) {
    Serial.println("[BLE] foundDevice is null — restarting scan");
    NimBLEDevice::getScan()->start(0, false); return;
  }
  Serial.println("[BLE] Attempting connection...");
  client = NimBLEDevice::createClient();
  if (!client->connect(foundDevice)) {
    Serial.println("[BLE] CONNECT FAILED");
    NimBLEDevice::deleteClient(client); client = nullptr; connected = false;
    delete foundDevice; foundDevice = nullptr;
    NimBLEDevice::getScan()->start(0, false); return;
  }
  Serial.println("[BLE] CONNECTED");
  NimBLERemoteService *svc = client->getService("FFE0");
  if (!svc) {
    Serial.println("[BLE] Service FFE0 not found");
    client->disconnect(); NimBLEDevice::deleteClient(client); client = nullptr;
    delete foundDevice; foundDevice = nullptr;
    NimBLEDevice::getScan()->start(0, false); return;
  }
  chr = svc->getCharacteristic("FFE1");
  if (!chr) {
    Serial.println("[BLE] Characteristic FFE1 not found");
    client->disconnect(); NimBLEDevice::deleteClient(client); client = nullptr;
    delete foundDevice; foundDevice = nullptr;
    NimBLEDevice::getScan()->start(0, false); return;
  }
  delete foundDevice; foundDevice = nullptr;
  connected = true;
  Serial.println("[BLE] READY TO SEND DATA");
}

/* ===================== BUILD / SEND PACKET ===================== */
uint8_t analyze_hr(int bpm) {
  uint8_t f = 0;
  if (bpm > 0 && bpm < 60)  f |= ALERT_HR_LOW;
  if (bpm > 120)             f |= ALERT_HR_HIGH;
  return f;
}

uint8_t analyze_temp(float tempF) {
  uint8_t f = 0;
  if (tempF > 100.4f) f |= ALERT_TEMP_HIGH;
  if (tempF < 75.0f)  f |= ALERT_TEMP_LOW; //based on finger placement
  return f;
}

uint8_t analyze_spo2(int s) {
  return (s >= 0 && s < 90) ? ALERT_SPO2_LOW : 0;
}

// fall_flag: 0 = vital packet, 1 = fall alert
void buildPacket(char *out, size_t len, uint8_t fall_flag) {
  char hrStr[40], spo2Str[40], tempStr[60];
  snprintf(hrStr,   sizeof(hrStr),   "[%d,%d,%d]",   hrBuf[0], hrBuf[1], hrBuf[2]);
  snprintf(spo2Str, sizeof(spo2Str), "[%d,%d,%d]",   spo2Buf[0], spo2Buf[1], spo2Buf[2]);
  snprintf(tempStr, sizeof(tempStr), "[%.1f,%.1f,%.1f]",
           tempBuf[0]*9.0f/5.0f + 32.0f + 5.0f,
           tempBuf[1]*9.0f/5.0f + 32.0f + 5.0f,
           tempBuf[2]*9.0f/5.0f + 32.0f + 5.0f);

  uint8_t flags = 0;

  // HR — average of valid (>0) samples
  float hrSum = 0; int hrCount = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++)
    if (hrBuf[i] > 0) { hrSum += hrBuf[i]; hrCount++; }
  if (hrCount > 0) flags |= analyze_hr((int)(hrSum / hrCount));

  // SpO2 — average of valid (>=0) samples only
  float spo2Sum = 0; int spo2Count = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++)
    if (spo2Buf[i] >= 0) { spo2Sum += spo2Buf[i]; spo2Count++; }
  if (spo2Count > 0) flags |= analyze_spo2((int)(spo2Sum / spo2Count));

  // Temperature — convert averaged °C to °F (same offset as display string)
  float avgTempC = (tempBuf[0] + tempBuf[1] + tempBuf[2]) / 3.0f;
  float avgTempF = avgTempC * 9.0f / 5.0f + 32.0f + 5.0f;
  // avgTempF =  avgTempF + 16.0f;
  flags |= analyze_temp(avgTempF);

  char errStr[64];
  snprintf(errStr, sizeof(errStr), "[%d,%d,%d,%d,%d,%d]",
           (flags & ALERT_HR_HIGH)   ? 1 : 0,
           (flags & ALERT_HR_LOW)    ? 1 : 0,
           (flags & ALERT_TEMP_HIGH) ? 1 : 0,
           (flags & ALERT_TEMP_LOW)  ? 1 : 0,
           (flags & ALERT_SPO2_LOW)  ? 1 : 0,
           fall_flag);

  snprintf(out, len, "Patient:%d;HR:%s;SPO2:%s;TEMP:%s;ERR:%s\n",
           PATIENT_ID, hrStr, spo2Str, tempStr, errStr);
}

void sendPacket(uint8_t fall_flag = 0) {
  if (!connected || !client || !client->isConnected() || !chr) return;
  char packet[200];
  memset(packet, 0, sizeof(packet));
  buildPacket(packet, sizeof(packet), fall_flag);
  Serial.print("[TX] ");
  Serial.print(packet);
  if (!chr->writeValue((uint8_t*)packet, strlen(packet), false))
    Serial.println("[BLE] WRITE FAILED");
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=================================");
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
  tickSensors();   // always: drains FIFO, polls MPU at 50 Hz, detects falls

  if (shouldConnect && !connected) { delay(300); connectToHM19(); }

  if (connected && client && !client->isConnected()) {
    Serial.println("[BLE] DISCONNECTED");
    connected = false; chr = nullptr;
    NimBLEDevice::deleteClient(client); client = nullptr;
    NimBLEDevice::getScan()->start(0, false);
  }

  if (!connected) { delay(20); return; }

  uint32_t now = millis();

  // ── Fall alert — 200 ms poll, fires immediately on detection ─────
  if (now - lastFallCheck >= FALL_CHECK_MS) {
    lastFallCheck = now;
    SensorPacket snap;
    readSensors(snap);
    if (snap.fallDetected) {
      // Save vital accumulation buffers, build fall packet, then restore
      int   savedHr  [SAMPLE_COUNT];
      int   savedSpo2[SAMPLE_COUNT];
      float savedTemp[SAMPLE_COUNT];
      memcpy(savedHr,   hrBuf,   sizeof(hrBuf));
      memcpy(savedSpo2, spo2Buf, sizeof(spo2Buf));
      memcpy(savedTemp, tempBuf, sizeof(tempBuf));

      for (int i = 0; i < SAMPLE_COUNT; i++) {
        hrBuf[i]   = snap.heartRate;
        spo2Buf[i] = snap.spo2;
        tempBuf[i] = snap.temperature;
      }
      sendPacket(1);   // fall_flag = 1

      memcpy(hrBuf,   savedHr,   sizeof(hrBuf));
      memcpy(spo2Buf, savedSpo2, sizeof(spo2Buf));
      memcpy(tempBuf, savedTemp, sizeof(tempBuf));
    }
  }

  // ── Vital sample — 3 readings spread across the 20 s window ─────
  if (now - lastVitalSample >= VITAL_SAMPLE_MS && vitalSlot < VITAL_SLOTS) {
    lastVitalSample = now;
    SensorPacket snap;
    readSensors(snap);
    hrBuf  [vitalSlot] = snap.heartRate;
    spo2Buf[vitalSlot] = snap.spo2;
    tempBuf[vitalSlot] = snap.temperature;
    vitalSlot++;
  }

  // ── Vital send — one packet at the end of each 20 s window ───────
  if (now - lastVitalSend >= VITAL_WINDOW_MS) {
    // Edge case: if a slot wasn't filled, copy the last valid one
    for (int i = vitalSlot; i < VITAL_SLOTS; i++) {
      int src = (vitalSlot > 0) ? vitalSlot - 1 : 0;
      hrBuf[i] = hrBuf[src]; spo2Buf[i] = spo2Buf[src]; tempBuf[i] = tempBuf[src];
    }
    lastVitalSend   = now;
    lastVitalSample = now;   // re-anchor sample timer to window boundary
    vitalSlot       = 0;
    sendPacket(0);           // fall_flag = 0; falls sent separately above
  }

  delay(20);
}