#ifndef SENSOR_AQUISITION_H
#define SENSOR_AQUISITION_H

#include <Arduino.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include <TinyGPSPlus.h>   
#include <Adafruit_Sensor.h>
#include "I2Cdev.h"
#include "ADXL345.h"
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
    uint8_t month; // Month of the date
    uint8_t day;   // Day of the date
    uint16_t year; // Year of the date
    uint8_t hour;  // Hour of the time  
    uint8_t minute; // Minute of the time
    uint8_t second; // Second of the time
    bool locationValid; // Flag to indicate if location is valid // Flag to indicate if altitude is valid
    bool dateValid; // Flag to indicate if date is valid
    bool timeValid; // Flag to indicate if time is valid
} GPSData_t;

// Unified display data struct
typedef struct {
    SensorType_t type;
    union {
        BaroData_t baro;
        AccelData_t accel;
        GPSData_t gps;
    } data;
    unsigned long msSinceStart; // Timestamp in ms since start
} DisplayData_t;

void taskBarometer(void *pvParameters);
void taskAccelerometer(void *pvParameters);
void taskGPS(void *pvParameters);
void i2cScan(void); // <-- Add this line

#endif // SENSOR_AQUISITION_H