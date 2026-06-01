#include "sensors.h"
#include <math.h>
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

#define PATIENT_ID        2
#define TARGET_HM19_MAC   "80:6f:b0:74:44:70"   // this board talks to THIS HM-19 only

/* ===================== BLE GLOBALS ===================== */

static NimBLEAdvertisedDevice     *foundDevice = nullptr;
static NimBLEClient               *client      = nullptr;
static NimBLERemoteCharacteristic *chr         = nullptr;

static bool shouldConnect = false;
static bool connected     = false;


// ===================== ALERT BITMASK =====================

#define ALERT_HR_HIGH   (1 << 0)
#define ALERT_HR_LOW    (1 << 1)
#define ALERT_TEMP_HIGH (1 << 2)
#define ALERT_TEMP_LOW  (1 << 3)
#define ALERT_SPO2_LOW  (1 << 4)
#define ALERT_FALL      (1 << 5)   // not used (no motion sensor)

#define ALERT_FALL      (1 << 5)

typedef enum {
    NORMAL,
    IMPACT,
    POST_FALL
} FallState;

FallState fallState = NORMAL;

uint32_t fallTimer = 0;
bool fallDetected = false;

float impactPitch = 0.0f;

/* ===================== SAMPLE BUFFER ===================== */

#define SAMPLE_COUNT 3
#define RAW_SAMPLE_COUNT 9

int hrRaw[RAW_SAMPLE_COUNT];
int spo2Raw[RAW_SAMPLE_COUNT];
float tempRaw[RAW_SAMPLE_COUNT];

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
  if (bpm > 120) f |= ALERT_HR_HIGH;
  return f;
}

float accel_mag(float ax, float ay, float az)
{
    return sqrtf(ax * ax +
                 ay * ay +
                 az * az);
}

float gyro_mag(float gx, float gy, float gz)
{
    return fabsf(gx) +
           fabsf(gy) +
           fabsf(gz);
}

float pitch_angle(float ax,
                  float ay,
                  float az)
{
    return atan2f(
               ax,
               sqrtf(ay * ay + az * az))
           * 180.0f / PI;
}

uint8_t analyze_temp(float tempF) {
  uint8_t f = 0;
  if ((tempF * 9.0f / 5.0f + 32.0f + 5.0f)> 100.4f) f |= ALERT_TEMP_HIGH;
  if ((tempF * 9.0f / 5.0f + 32.0f + 5.0f)< 95.0f) f |= ALERT_TEMP_LOW;
  return f;
}

uint8_t analyze_spo2(int spo2) {
  return (spo2 < 90) ? ALERT_SPO2_LOW : 0;
}
void analyze_motion(uint32_t t,
                    float ax,
                    float ay,
                    float az,
                    float gx,
                    float gy,
                    float gz)
{
    float acc = accel_mag(ax, ay, az);

    float gyro = gyro_mag(gx, gy, gz);

    float pitch = pitch_angle(ax, ay, az);

    switch (fallState)
    {
        case NORMAL:

            if (acc > 1.0f)
            {
                impactPitch = pitch;

                fallState = IMPACT;

                fallTimer = t;

                //Serial.println("[FALL] IMPACT");
            }

            break;

        case IMPACT:

            if (t - fallTimer > 1000)
            {
                float angleChange =
                    fabsf(pitch - impactPitch);

                if (angleChange > 20.0f)
                {
                    fallState = POST_FALL;

                    fallTimer = t;

                   // Serial.println("[FALL] ORIENTATION CHANGE");
                }
                else
                {
                    fallState = NORMAL;
                }
            }

            break;

        case POST_FALL:

            if (gyro < 50.0f)
            {
                if (t - fallTimer > 3000)
                {
                    fallDetected = true;

                   // Serial.println("[FALL] DETECTED");

                    fallState = NORMAL;
                }
            }
            else
            {
                fallState = NORMAL;
            }

            break;
    }
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
           tempBuf[0] * 9.0f / 5.0f + 32.0f + 5.0f,
           tempBuf[1] * 9.0f / 5.0f + 32.0f + 5.0f,
           tempBuf[2] * 9.0f / 5.0f + 32.0f + 5.0f);


  // ===================== FLAG LOGIC =====================
  uint8_t flags = 0;
  float avgHR =
    (hrBuf[0] + hrBuf[1] + hrBuf[2]) / 3.0f;

  // float avgSpO2 =
  //   (spo2Buf[0] + spo2Buf[1] + spo2Buf[2]) / 3.0f;
  float avgSpO2 = 0;
int validSpO2Count = 0;

for (int i = 0; i < SAMPLE_COUNT; i++) {
    if (spo2Buf[i] >= 0) {   // ignore negative readings
        avgSpO2 += spo2Buf[i];
        validSpO2Count++;
    }
}

if (validSpO2Count > 0) {
    avgSpO2 /= validSpO2Count;
    flags |= analyze_spo2((int)avgSpO2);
}
  float avgTempC =
    (tempBuf[0] + tempBuf[1] + tempBuf[2]) / 3.0f;
  // use latest sample in buffer (last written index - 1)
  int last = (sampleIndex == 0) ? SAMPLE_COUNT - 1 : sampleIndex - 1;
  flags |= analyze_hr((int)avgHR);
  //flags |= analyze_spo2((int)avgSpO2);
  flags |= analyze_temp(avgTempC);   // if using Fahrenheit thresholds
  
 uint8_t fall_flag = fallDetected ? 1 : 0;

if (fallDetected)
{
    flags |= ALERT_FALL;
    fallDetected = false;
}

  char errStr[64];
  snprintf(errStr, sizeof(errStr),
           "[%d,%d,%d,%d,%d,%d]",
           (flags & ALERT_HR_HIGH)    ? 1 : 0,
           (flags & ALERT_HR_LOW)   ? 1 : 0,
           (flags & ALERT_TEMP_HIGH) ? 1 : 0,
           (flags & ALERT_TEMP_LOW)  ? 1 : 0,
           (flags & ALERT_SPO2_LOW)  ? 1 : 0,
           fall_flag);

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

uint32_t lastMotion = 0;

  // collect one real sensor reading per second
  if (connected && millis() - lastSample >= 1000) {
    lastSample = millis();


    SensorPacket data;
    if (readSensors(data)) {

    analyze_motion(
    millis(),
    data.ax,
    data.ay,
    data.az,
    data.gx,
    data.gy,
    data.gz
);
    hrRaw[sampleIndex]   = data.heartRate;
    spo2Buf[sampleIndex / 3] = data.spo2;
    tempBuf[sampleIndex / 3] = data.temperature;


    sampleIndex++;


    // raw HR buffer full — average into hrBuf, then send
    if (sampleIndex >= RAW_SAMPLE_COUNT) {
        for (int i = 0; i < SAMPLE_COUNT; i++) {
          int start = i * 3;


            int sum = 0;
            int validCount = 0;


            for (int j = 0; j < 3; j++) {
              int value = hrRaw[start + j];


              // ignore invalid HR readings
              if (value >= 0) {
                sum += value;
                validCount++;
            }
          }


        if (validCount > 0) {
          hrBuf[i] = sum / validCount;
        } else {
          hrBuf[i] = -1;
        }
      }


      sendPacket();
      sampleIndex = 0;
    }


    }
    
  }


  delay(20);
}
