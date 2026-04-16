// ==========================
// VitalsPatch ESP32
// This program starts the ESP32, initializes the two sensors,
// and continuously reads data from them.
// ==========================


// -------- Libraries --------

// Include the Wire library.
// This is used for I2C communication, which is the protocol
// the ESP32 uses to talk to both the MAX30102 and MPU-6500.
#include <Wire.h>

// Include the SparkFun MAX3010x library.
// This library lets us control and read data from the MAX30102 sensor.
#include "MAX30105.h"

// Include the FastIMU library.
// This library lets us control and read data from the MPU-6500 sensor.
#include <FastIMU.h>


// -------- Objects --------

// Create an object named particleSensor for the MAX30102.
// "particleSensor" is just the variable name we will use in code
// whenever we want to configure or read from the MAX30102.
MAX30105 particleSensor;

// Create an object named IMU for the MPU-6500.
// "IMU" stands for Inertial Measurement Unit, which is a sensor
// that measures acceleration and rotation.
MPU6500 IMU;

// Create a calibration data variable named calib.
// This stores calibration settings for the MPU sensor.
// { 0 } means all values start at zero for now.
calData calib = { 0 };

// Create a variable named accelData to store accelerometer readings.
// This will hold acceleration values in the x, y, and z directions.
AccelData accelData;

// Create a variable named gyroData to store gyroscope readings.
// This will hold angular velocity values in the x, y, and z directions.
GyroData gyroData;

// -------- Sensor Data Packet --------

// This struct groups all sensor readings into one place
struct SensorData {
  long ir;     // infrared value from MAX30102
  long red;    // red light value from MAX30102

  float ax;    // acceleration in X direction
  float ay;    // acceleration in Y direction
  float az;    // acceleration in Z direction

  float gx;    // gyro rotation in X
  float gy;    // gyro rotation in Y
  float gz;    // gyro rotation in Z
};

// Create one global variable that will store all sensor data
SensorData data;


// -------- Setup --------

// setup() runs only one time when the ESP32 turns on or resets.
// We use setup() to initialize things before the main program starts.
void setup() {

  // Start serial communication between the ESP32 and the computer.
  // 115200 is the communication speed in bits per second.
  // This lets us print messages to the Serial Monitor for debugging.
  Serial.begin(115200);

  // Wait 1 second so the Serial Monitor has time to connect.
  delay(1000);

  // Print a startup message so we know the program began running.
  Serial.println("VitalsPatch ESP32 starting...");

  // Start I2C communication.
  // This must happen before the ESP32 can talk to the sensors.
  Wire.begin();

  // Print a message confirming I2C has started.
  Serial.println("I2C initialized");

  // Call the function that initializes the MAX30102 sensor.
  initMAX30102();

  // Call the function that initializes the MPU-6500 sensor.
  initMPU6500();
}


// -------- Loop --------

// loop() runs forever after setup() finishes.
// This is where the ESP32 keeps reading sensor data again and again.
void loop() {

  // Call the function that reads and prints MAX30102 data.
  readMAX30102();

  // Call the function that reads and prints MPU-6500 data.
  readMPU6500();

  // Print a separator line so the output is easier to read
  // in the Serial Monitor between one cycle and the next.
  Serial.println("-----------------------------------");

  // Wait 500 milliseconds before reading the sensors again.
  delay(500);
}


// -------- Functions --------


// --------------------
// Initialize MAX30102
// --------------------

// This function sets up the MAX30102 so it is ready to use.
void initMAX30102() {

  // Print message to show MAX30102 initialization is starting.
  Serial.println("Initializing MAX30102...");

  // Try to start communication with the MAX30102 using I2C.
  // particleSensor.begin(...) returns false if the sensor is not found.
  //
  // particleSensor = the MAX30102 sensor object
  // Wire = tells the library to use I2C
  // I2C_SPEED_STANDARD = standard I2C speed (safe default)
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {

    // Print an error message if the sensor was not found.
    Serial.println("MAX30102 was not found. Check wiring and power.");

    // Stop the program completely if the sensor cannot be initialized.
    // while(1) creates an infinite loop, so the code stays here forever.
    while (1);
  }

  // These variables store the startup settings for the MAX30102.

  // ledBrightness controls how bright the sensor LEDs are.
  // Higher value = brighter LED.
  byte ledBrightness = 60;

  // sampleAverage controls how many samples are averaged together.
  // Averaging helps reduce noise in readings.
  byte sampleAverage = 4;

  // ledMode controls which LEDs are active.
  // 2 means Red LED + IR LED are both used.
  byte ledMode = 2;

  // sampleRate controls how many readings per second the sensor takes.
  int sampleRate = 100;

  // pulseWidth controls how long each LED pulse lasts.
  // Larger pulse width can improve sensitivity but may slow things down.
  int pulseWidth = 411;

  // adcRange controls the measurement range of the sensor ADC.
  // ADC = analog-to-digital converter.
  int adcRange = 4096;

  // Apply all of the settings above to the MAX30102 sensor.
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  // Print a success message if setup is complete.
  Serial.println("MAX30102 initialized successfully.");
}


// --------------------
// Read MAX30102 values
// --------------------

// This function reads the raw Red and IR values from the MAX30102.
void readMAX30102() {

  // Read the raw infrared value from the sensor and store it in irValue.
  // "long" is used because sensor values can get large.
  long irValue = particleSensor.getIR();

  // Read the raw red-light value from the sensor and store it in redValue.
  long redValue = particleSensor.getRed();

  // Print a label so we know this data came from the MAX30102.
  Serial.print("MAX30102 -> IR: ");

  // Print the raw IR value.
  Serial.print(irValue);

  // Print a label before the red value.
  Serial.print("  Red: ");

  // Print the raw red value and move to the next line.
  Serial.println(redValue);
}


// --------------------
// Initialize MPU-6500
// --------------------

// This function sets up the MPU-6500 so it is ready to use.
void initMPU6500() {

  // Print message to show MPU-6500 initialization is starting.
  Serial.println("Initializing MPU-6500...");

  // Try to start communication with the MPU-6500.
  //
  // IMU.init(...) returns an error code:
  // 0 means success
  // nonzero means something went wrong
  //
  // calib = calibration data variable
  // 0x68 = the usual I2C address of the MPU-6500
  int err = IMU.init(calib, 0x68);

  // Check whether initialization failed.
  if (err != 0) {

    // Print an error message and the error code.
    Serial.print("MPU-6500 initialization failed. Error: ");
    Serial.println(err);

    // Stop the program completely if the MPU-6500 cannot be initialized.
    while (1);
  }

  // Print a success message if setup is complete.
  Serial.println("MPU-6500 initialized successfully.");
}


// --------------------
// Read MPU-6500 values
// --------------------

// This function updates and prints the accelerometer and gyroscope data.
void readMPU6500() {

  // Update the sensor so the library gets the newest readings.
  IMU.update();

  // Copy the newest accelerometer values into accelData.
  // The "&" means we are giving the function the address of accelData
  // so it can fill that variable with data.
  IMU.getAccel(&accelData);

  // Copy the newest gyroscope values into gyroData.
  IMU.getGyro(&gyroData);

  // Print a label to show these are accelerometer readings.
  Serial.print("MPU6500 -> Accel X: ");

  // Print acceleration in the X direction.
  Serial.print(accelData.accelX);

  // Print label and acceleration in the Y direction.
  Serial.print("  Y: ");
  Serial.print(accelData.accelY);

  // Print label and acceleration in the Z direction.
  Serial.print("  Z: ");
  Serial.println(accelData.accelZ);

  // Print a label to show these are gyroscope readings.
  Serial.print("MPU6500 -> Gyro  X: ");

  // Print angular velocity in the X direction.
  Serial.print(gyroData.gyroX);

  // Print label and angular velocity in the Y direction.
  Serial.print("  Y: ");
  Serial.print(gyroData.gyroY);

  // Print label and angular velocity in the Z direction.
  Serial.print("  Z: ");
  Serial.println(gyroData.gyroZ);
}