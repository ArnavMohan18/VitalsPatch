// ==========================
// VitalsPatch ESP32
// This program starts the ESP32, initializes the two sensors,
// continuously reads data from them, and sends that data
// over UART to the STM32 for Maya's side to process.
// ==========================


// -------- Libraries --------

// Wire library: lets the ESP32 talk over I2C,
// which is how it communicates with both sensors.
#include <Wire.h>

// MAX30105 library: controls the MAX30102 sensor
// (used for pulse and blood-oxygen-related readings).
#include "MAX30105.h"

// FastIMU library: controls the MPU-6500 sensor
// (measures motion — acceleration and rotation).
#include <FastIMU.h>


// -------- Objects --------

// Create a sensor object for the MAX30102.
// We'll use "particleSensor" whenever we talk to it.
MAX30105 particleSensor;

// Create a sensor object for the MPU-6500.
// "IMU" = Inertial Measurement Unit (motion sensor).
MPU6500 IMU;

// Calibration data holder for the MPU.
// Starts at zero — we're not calibrating manually here.
calData calib = { 0 };

// Temporary holders for the newest MPU readings.
// The library fills these when we ask for data.
AccelData accelData;   // holds accel X/Y/Z
GyroData  gyroData;    // holds gyro  X/Y/Z


// -------- Sensor Data Packet --------

// One neat bundle that holds ALL sensor readings.
// This is what gets sent to the STM32 as a single packet.
//
// Using int32_t (instead of "long") guarantees 4 bytes
// on both the ESP32 and the STM32, so the byte layout
// matches exactly on both sides. This is important.
struct SensorData {
  int32_t ir;     // IR reading from MAX30102 (4 bytes)
  int32_t red;    // Red reading from MAX30102 (4 bytes)

  float ax;       // accel X (4 bytes)
  float ay;       // accel Y (4 bytes)
  float az;       // accel Z (4 bytes)

  float gx;       // gyro  X (4 bytes)
  float gy;       // gyro  Y (4 bytes)
  float gz;       // gyro  Z (4 bytes)
};
// Total struct size: 32 bytes

// A single global "data" variable that every function
// reads from or writes to. This is the shared bundle.
SensorData data;


// -------- Setup --------
// setup() runs ONCE when the ESP32 powers on or resets.
// This is where we turn things on and get everything ready.
void setup() {

  // Start USB serial (to the computer / Serial Monitor).
  // Used only for debug prints so we can see what's happening.
  Serial.begin(115200);

  // Start Serial2 (UART2) — this is the line going to the STM32.
  //   115200     = speed in bits per second
  //   SERIAL_8N1 = 8 data bits, No parity, 1 stop bit (standard)
  //   16         = RX pin (ESP32 receives on GPIO 16)
  //   17         = TX pin (ESP32 transmits on GPIO 17)
  //
  // WIRING REMINDER:
  //   ESP32 TX (GPIO 17) --> STM32 RX
  //   ESP32 RX (GPIO 16) <-- STM32 TX
  //   ESP32 GND <---------> STM32 GND  (must be shared!)
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  // Short pause so the Serial Monitor has time to connect.
  delay(1000);

  // Print startup messages so we know things are alive.
  Serial.println("VitalsPatch ESP32 starting...");
  Serial.println("Serial2 initialized for STM32 communication");

  // Turn on I2C so we can talk to the two sensors.
  Wire.begin();
  Serial.println("I2C initialized");

  // Get each sensor ready to use.
  initMAX30102();
  initMPU6500();
}


// -------- Loop --------
// loop() runs over and over, FOREVER, after setup() finishes.
// Each pass: read sensors, fill the struct, send it to the STM32.
void loop() {

  // Read the MAX30102 and store values into the shared struct.
  readMAX30102();

  // Read the MPU-6500 and store values into the shared struct.
  readMPU6500();

  // Send the freshly-updated struct to the STM32 over UART2.
  sendToSTM32();

  // Separator line in the Serial Monitor so output is readable.
  Serial.println("-----------------------------------");

  // Wait half a second before doing it all again.
  // (Adjust this if you need faster or slower updates.)
  delay(500);
}


// -------- Functions --------


// --------------------
// Initialize MAX30102
// --------------------
// Turns on the MAX30102 and applies its settings.
void initMAX30102() {

  Serial.println("Initializing MAX30102...");

  // Try to start the sensor over I2C at standard speed.
  // If it's not found, begin() returns false.
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 was not found. Check wiring and power.");
    // Freeze the program here so we notice the problem.
    while (1);
  }

  // Sensor configuration values (safe defaults for PPG signals):
  byte ledBrightness = 60;    // LED power (higher = brighter)
  byte sampleAverage = 4;     // averages samples to reduce noise
  byte ledMode       = 2;     // 2 = Red + IR LEDs both on
  int  sampleRate    = 100;   // readings per second
  int  pulseWidth    = 411;   // LED pulse length in microseconds
  int  adcRange      = 4096;  // ADC measurement range

  // Apply the settings to the sensor.
  particleSensor.setup(ledBrightness, sampleAverage, ledMode,
                       sampleRate, pulseWidth, adcRange);

  Serial.println("MAX30102 initialized successfully.");
}


// --------------------
// Read MAX30102 values
// --------------------
// Grabs the newest IR and Red values and stores them
// in the shared 'data' struct so sendToSTM32() can ship them.
void readMAX30102() {

  // Ask the sensor for the latest IR and Red readings.
  long irValue  = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // Save the values into the shared struct.
  // Casting to int32_t ensures the struct field size is exact.
  data.ir  = (int32_t)irValue;
  data.red = (int32_t)redValue;

  // Debug prints so we can see the values in the Serial Monitor.
  Serial.print("MAX30102 -> IR: ");
  Serial.print(irValue);
  Serial.print("  Red: ");
  Serial.println(redValue);
}


// --------------------
// Initialize MPU-6500
// --------------------
// Turns on the MPU-6500 motion sensor.
void initMPU6500() {

  Serial.println("Initializing MPU-6500...");

  // Start the sensor. 0x68 is its default I2C address.
  // init() returns 0 on success, nonzero on failure.
  int err = IMU.init(calib, 0x68);

  if (err != 0) {
    Serial.print("MPU-6500 initialization failed. Error: ");
    Serial.println(err);
    // Freeze the program so the issue is obvious.
    while (1);
  }

  Serial.println("MPU-6500 initialized successfully.");
}


// --------------------
// Read MPU-6500 values
// --------------------
// Pulls the newest accel + gyro readings, stores them in
// the shared 'data' struct, and also prints them for debug.
void readMPU6500() {

  // Step 1: Tell the library to grab fresh data from the sensor.
  IMU.update();

  // Step 2: Copy that data into the temporary holders.
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);

  // Step 3: Copy from the temporary holders into the shared struct.
  // This is the struct that sendToSTM32() will actually transmit.
  data.ax = accelData.accelX;
  data.ay = accelData.accelY;
  data.az = accelData.accelZ;

  data.gx = gyroData.gyroX;
  data.gy = gyroData.gyroY;
  data.gz = gyroData.gyroZ;

  // Debug prints — accelerometer values.
  Serial.print("MPU6500 -> Accel X: ");
  Serial.print(accelData.accelX);
  Serial.print("  Y: ");
  Serial.print(accelData.accelY);
  Serial.print("  Z: ");
  Serial.println(accelData.accelZ);

  // Debug prints — gyroscope values.
  Serial.print("MPU6500 -> Gyro  X: ");
  Serial.print(gyroData.gyroX);
  Serial.print("  Y: ");
  Serial.print(gyroData.gyroY);
  Serial.print("  Z: ");
  Serial.println(gyroData.gyroZ);
}


// --------------------
// Send data to STM32
// --------------------
// Sends the shared 'data' struct to the STM32 as a single
// framed binary packet.
//
// Packet format (34 bytes total):
//   [0xAA] [0x55] [32 bytes of SensorData] [1 byte checksum]
//
// - 0xAA 0x55 = "sync header". Tells the STM32: "a new packet starts here."
// - Payload   = the raw bytes of the SensorData struct.
// - Checksum  = XOR of all payload bytes. Lets the STM32 check
//               that the data didn't get corrupted on the way over.
void sendToSTM32() {

  // The two header bytes that mark the start of a packet.
  uint8_t header[2] = { 0xAA, 0x55 };

  // Treat the struct as a raw stream of bytes.
  // (uint8_t*) = "pretend this is a pointer to bytes"
  // &data      = "the memory address of our struct"
  uint8_t* payload     = (uint8_t*)&data;
  size_t   payloadSize = sizeof(SensorData);   // should be 32

  // Compute a simple XOR checksum across the payload.
  // XOR-ing every byte into 'checksum' gives a single byte
  // that the STM32 can recompute and compare.
  uint8_t checksum = 0;
  for (size_t i = 0; i < payloadSize; i++) {
    checksum ^= payload[i];
  }

  // Send the three pieces, in order, over UART2:
  //   1) the 2-byte header
  //   2) the 32-byte payload
  //   3) the 1-byte checksum
  Serial2.write(header, 2);
  Serial2.write(payload, payloadSize);
  Serial2.write(checksum);
}
