#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

struct SensorPacket {

  int32_t heartRate;
  int32_t spo2;

  float temperature;

  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;
};

bool initSensors();

bool readSensors(SensorPacket &data);

void tickSensors();

#endif