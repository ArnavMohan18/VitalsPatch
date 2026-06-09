/**
 * @file sensors.h
 * @brief Sensor interface for the VitalsPatch wearable system.
 *
 * @author Arnav Mohan, Maya Desai, Eeshani Shilamkar
 * @author VitalsPatch Team
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

struct SensorPacket {
    int32_t heartRate;
    int32_t spo2;
    float   temperature;

    float   ax, ay, az;
    float   gx, gy, gz;

    bool    fallDetected;
};

bool initSensors();
bool readSensors(SensorPacket &data);
void tickSensors();

#endif