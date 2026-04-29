#include <NimBLEDevice.h>

static NimBLEAdvertisedDevice* foundDevice = nullptr;
static bool shouldConnect = false;
static bool connected = false;

void notifyCallback(NimBLERemoteCharacteristic* pChr,
                    uint8_t* pData,
                    size_t length,
                    bool isNotify) {
  Serial.print("STM32 says: ");
  Serial.write(pData, length);
  Serial.println();
}

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) {
    if (advertisedDevice->getAddress().toString() == "80:6f:b0:74:44:70") {
      Serial.println("HM-19 found. Stopping scan.");

      foundDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
      shouldConnect = true;

      NimBLEDevice::getScan()->stop();
    }
  }
};

void connectToHM19() {
  shouldConnect = false;

  NimBLEDevice::getScan()->clearResults();

  NimBLEClient* pClient = NimBLEDevice::createClient();

  Serial.println("Connecting to HM-19...");
  if (!pClient->connect(foundDevice)) {
    Serial.println("Connect failed.");
    NimBLEDevice::deleteClient(pClient);
    connected = false;
    return;
  }

  Serial.println("Connected.");

  NimBLERemoteService* pService = pClient->getService("FFE0");
  if (!pService) {
    Serial.println("FFE0 not found.");
    pClient->disconnect();
    return;
  }

  Serial.println("FFE0 found.");

  NimBLERemoteCharacteristic* pChar = pService->getCharacteristic("FFE1");
  if (!pChar) {
    Serial.println("FFE1 not found.");
    pClient->disconnect();
    return;
  }

  Serial.println("FFE1 found.");

  if (pChar->canNotify()) {
    pChar->subscribe(true, notifyCallback);
    Serial.println("Subscribed.");
  }

  connected = true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("NimBLE HM-19 delayed-connect test");

  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new ScanCallbacks(), false);
  pScan->setActiveScan(false);
  pScan->setInterval(100);
  pScan->setWindow(50);

  Serial.println("Scanning...");
  pScan->start(0, false);
}

void loop() {
  if (shouldConnect && !connected) {
    delay(500);
    connectToHM19();
  }

  delay(100);
}