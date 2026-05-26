// ============================================================================
//  VitalsPatch -- Receiver ESP32  (one per patient)
// ----------------------------------------------------------------------------
//  Connects to ONE patient's HM-19 over BLE, subscribes to FFE1 notifications,
//  reassembles the vitals line (BLE splits it into <=20-byte chunks), tags it
//  with the patient letter, and prints it to USB serial for the dashboard.
//
//  >>> ONLY TWO LINES CHANGE between the two boards. <<<
//      Patient A:  PATIENT_TAG "A"   HM19_ADDRESS "80:6f:b0:74:44:70"
//      Patient B:  PATIENT_TAG "B"   HM19_ADDRESS "80:6f:b0:6f:a8:8a"
//
//  Output to the PC looks like:   A,5234,73,98.2,36.6,1.01,0
//  (tag, then the STM32's  t,hr,spo2,temp,acc,flags  record)
//
//  Library: NimBLE-Arduino v2.x  (Tools > Manage Libraries > "NimBLE-Arduino")
// ============================================================================

#include <NimBLEDevice.h>

// ----- CHANGE THESE TWO LINES PER BOARD -----
#define PATIENT_TAG   "A"
#define HM19_ADDRESS  "80:6f:b0:74:44:70"
// --------------------------------------------

static NimBLEAdvertisedDevice* foundDevice = nullptr;
static NimBLEClient*           pClient     = nullptr;
static bool shouldConnect = false;
static bool connected     = false;

// Reassembly buffer: BLE notifications arrive in fragments, so we collect
// bytes until we hit '\n', then emit one complete, tagged line.
static String lineBuf = "";

// ---- notifications from the HM-19 (FFE1) ----
static void notifyCallback(NimBLERemoteCharacteristic* /*pChr*/,
                           uint8_t* pData, size_t length, bool /*isNotify*/) {
  for (size_t i = 0; i < length; i++) {
    char c = (char)pData[i];
    if (c == '\n') {
      lineBuf.trim();                 // drop stray '\r' / spaces
      if (lineBuf.length() > 0) {
        Serial.print(PATIENT_TAG);    // -> "A," or "B,"
        Serial.print(',');
        Serial.println(lineBuf);      // -> the STM32 record
      }
      lineBuf = "";
    } else if (c != '\r') {
      lineBuf += c;
      if (lineBuf.length() > 80) lineBuf = "";  // safety against runaway data
    }
  }
}

// ---- mark link as down; loop() handles the actual re-scan ----
class ClientCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient* /*c*/, int /*reason*/) override {
    Serial.println("# disconnected, will rescan...");
    connected = false;
    lineBuf   = "";
  }
};
static ClientCallbacks clientCB;

// ---- find our specific HM-19 by address ----
class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    if (advertisedDevice->getAddress().toString() == HM19_ADDRESS) {
      Serial.println("# HM-19 found, stopping scan");
      if (foundDevice) { delete foundDevice; foundDevice = nullptr; }  // no leak on reconnect
      foundDevice   = new NimBLEAdvertisedDevice(*advertisedDevice);
      shouldConnect = true;
      NimBLEDevice::getScan()->stop();
    }
  }
};

void connectToHM19() {
  shouldConnect = false;
  NimBLEDevice::getScan()->clearResults();

  if (pClient == nullptr) {
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(&clientCB, false);
  }

  Serial.println("# connecting...");
  if (!pClient->connect(foundDevice)) {
    Serial.println("# connect failed");
    return;                       // loop() restarts the scan
  }

  NimBLERemoteService* pService = pClient->getService("FFE0");
  if (!pService) { Serial.println("# FFE0 missing"); pClient->disconnect(); return; }

  NimBLERemoteCharacteristic* pChar = pService->getCharacteristic("FFE1");
  if (!pChar)    { Serial.println("# FFE1 missing"); pClient->disconnect(); return; }

  if (pChar->canNotify()) {
    pChar->subscribe(true, notifyCallback);
    Serial.print("# subscribed, streaming patient ");
    Serial.println(PATIENT_TAG);
  }
  connected = true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.print("# VitalsPatch receiver -- patient ");
  Serial.println(PATIENT_TAG);

  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new ScanCallbacks(), false);
  pScan->setActiveScan(false);
  pScan->setInterval(100);
  pScan->setWindow(50);
  pScan->start(0, false);
}

void loop() {
  if (shouldConnect && !connected) {
    delay(500);
    connectToHM19();
  } else if (!connected && !shouldConnect &&
             !NimBLEDevice::getScan()->isScanning()) {
    NimBLEDevice::getScan()->start(0, false);   // (re)start scanning
  }
  delay(100);
}
