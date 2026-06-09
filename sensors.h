#ifndef SENSORS_H
#define SENSORS_H
#include <Arduino.h>

struct SensorPacket {
  int32_t heartRate;
  int32_t spo2;
  float   temperature;
  float   ax, ay, az;
  float   gx, gy, gz;
  bool    fallDetected;   // set by tickSensors(), consumed by readSensors()
};

bool initSensors();
bool readSensors(SensorPacket &data);
void tickSensors();
#endif