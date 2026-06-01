#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute;
int beatAvg;

// SpO2 averaging
const byte SPO2_SIZE = 4;
byte spo2Readings[SPO2_SIZE];
byte spo2Spot = 0;
int spo2Avg;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
}

void loop() {
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  if (checkForBeat(irValue) == true) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;

      // Compute SpO2 from red/IR ratio at each beat
      if (irValue > 0) {
        float ratio = (float)redValue / (float)irValue;
        // Empirical formula for MAX30102
        int spo2 = 110 - (25 * ratio);
        spo2 = constrain(spo2, 0, 100);

        spo2Readings[spo2Spot++] = spo2;
        spo2Spot %= SPO2_SIZE;

        spo2Avg = 0;
        for (byte x = 0; x < SPO2_SIZE; x++)
          spo2Avg += spo2Readings[x];
        spo2Avg /= SPO2_SIZE;
      }
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  Serial.print(", SpO2=");
  Serial.print(spo2Avg);
  Serial.print("%");

  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();
}