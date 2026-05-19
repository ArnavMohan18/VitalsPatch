#include <NimBLEDevice.h>

static NimBLEAdvertisedDevice* foundDevice = nullptr;
static NimBLEClient* pClient = nullptr;
static NimBLERemoteCharacteristic* pChar = nullptr;

static bool shouldConnect = false;
static bool connected = false;
static bool ledOn = false;

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) {
    if (advertisedDevice->getAddress().toString() == "80:6f:b0:6f:a8:8a") {
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

  pClient = NimBLEDevice::createClient();

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

  pChar = pService->getCharacteristic("FFE1");
  if (!pChar) {
    Serial.println("FFE1 not found.");
    pClient->disconnect();
    return;
  }

  Serial.println("FFE1 found.");
  Serial.println("Ready to write to STM32.");

  connected = true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("ESP32-C3 HM-19 Writer");

  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new ScanCallbacks(), false);
  pScan->setActiveScan(false);
  pScan->setInterval(100);
  pScan->setWindow(50);

  Serial.println("Scanning for HM-19...");
  pScan->start(0, false);
}

void loop() {
  if (shouldConnect && !connected) {
    delay(500);
    connectToHM19();
  }

  if (connected && pClient != nullptr && pClient->isConnected() && pChar != nullptr) {
    if (ledOn) {
      pChar->writeValue("0", 1, false);
      Serial.println("B: Sent 0 to STM32: LED OFF");
    } else {
      pChar->writeValue("1", 1, false);
      Serial.println("B: Sent 1 to STM32: LED ON");
    }

    ledOn = !ledOn;
    delay(3000);
  }

  delay(100);
}