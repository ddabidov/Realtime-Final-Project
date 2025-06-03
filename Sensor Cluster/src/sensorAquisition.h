#ifndef SENSOR_AQUISITION_H
#define SENSOR_AQUISITION_H

#include <Arduino.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <TinyGPSPlus.h>   
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "FreeRTOS.h"

// Enum for sensor data type
typedef enum {
    SENSOR_BARO,
    SENSOR_ACCEL,
    SENSOR_GPS
} SensorType_t;

// Data structures for sensor data
typedef struct {
    float pressure;
    float temperature;
} BaroData_t;

typedef struct {
    float x, y, z;
} AccelData_t;

typedef struct {
    double latitude;
    double longitude;
    double altitude;
} GPSData_t;

// Unified display data struct
typedef struct {
    SensorType_t type;
    union {
        BaroData_t baro;
        AccelData_t accel;
        GPSData_t gps;
    } data;
} DisplayData_t;

void taskBarometer(void *pvParameters);
void taskAccelerometer(void *pvParameters);
void taskGPS(void *pvParameters);

#endif // SENSOR_AQUISITION_H