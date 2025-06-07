#ifndef SENSOR_AQUISITION_H
#define SENSOR_AQUISITION_H

#include <Arduino.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include <TinyGPSPlus.h>   
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "FreeRTOS.h"

#define GPS_RX_PIN 13
#define GPS_TX_PIN 12

#define gpsSerial Serial1  // Use built-in Serial1 for Pi Pico


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
    float altitude; // Altitude can be calculated from pressure
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