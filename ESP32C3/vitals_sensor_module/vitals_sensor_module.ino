#include "sensors.h"

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== VitalsPatch Sensor Test ===");

  if (!initSensors()) {
    Serial.println("[FATAL] Sensor initialization failed.");

    while (1) {
      delay(1000);
    }
  }
}

void loop() {

  if (millis() - lastPrint >= 1000) {

    lastPrint = millis();

    SensorPacket data;

    if (readSensors(data)) {

      Serial.println("========== SENSOR DATA ==========");

      Serial.print("Heart Rate: ");
      Serial.println(data.heartRate);

      Serial.print("SpO2: ");
      Serial.println(data.spo2);

      Serial.print("Temperature: ");
      Serial.println(data.temperature);

      Serial.println("--- Accelerometer (g) ---");

      Serial.print("AX: ");
      Serial.println(data.ax, 3);

      Serial.print("AY: ");
      Serial.println(data.ay, 3);

      Serial.print("AZ: ");
      Serial.println(data.az, 3);

      Serial.println("--- Gyroscope (deg/s) ---");

      Serial.print("GX: ");
      Serial.println(data.gx, 2);

      Serial.print("GY: ");
      Serial.println(data.gy, 2);

      Serial.print("GZ: ");
      Serial.println(data.gz, 2);

      Serial.println();
    }
    else {

      Serial.println("[ERROR] Failed to read sensors.");
    }
  }
}