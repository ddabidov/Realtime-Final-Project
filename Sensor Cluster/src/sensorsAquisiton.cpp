#include "sensorAquisition.h"
#include <Arduino.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <TinyGPSPlus.h>   
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "FreeRTOS.h"



void taskBarometer(void *pvParameters) {
    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    // Initialize the barometer sensor
    if (!barometer.begin()) {
        Serial.println("Failed to initialize barometer!");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        // Read the barometer data
        float pressure = barometer.readPressure();
        float temperature = barometer.readTemperature();

        DisplayData_t msg;
        msg.type = SENSOR_BARO;
        msg.data.baro.pressure = pressure;
        msg.data.baro.temperature = temperature;
        xQueueSend(displayQueue, &msg, 0);

        // Delay for a while before the next reading
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskGPS(void *pvParameters) {
    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    // Initialize the GPS sensor
    if (!gps.begin()) {
        Serial.println("Failed to initialize GPS!");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        // Read the GPS data
        if (gps.available()) {
            // Replace with actual GPS data extraction
            GPSData_t gpsData;
            gpsData.latitude = gps.location.lat();
            gpsData.longitude = gps.location.lng();
            gpsData.altitude = gps.altitude.meters();

            DisplayData_t msg;
            msg.type = SENSOR_GPS;
            msg.data.gps = gpsData;
            xQueueSend(displayQueue, &msg, 0);
        }
        // Delay for a while before the next reading
        vTaskDelay(pdMS_TO_TICKS(10000)); // Example: poll every 10s
    }
}

void taskAccelerometer(void *pvParameters) {
    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    // Initialize the ADXL sensor
    if (!adxl.begin()) {
        Serial.println("Failed to initialize ADXL!");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        // Read the ADXL data
        float x, y, z;
        adxl.readAcceleration(x, y, z);

        DisplayData_t msg;
        msg.type = SENSOR_ACCEL;
        msg.data.accel.x = x;
        msg.data.accel.y = y;
        msg.data.accel.z = z;
        xQueueSend(displayQueue, &msg, 0);

        // Delay for a while before the next reading
        vTaskDelay(pdMS_TO_TICKS(20)); // Example: poll every 20ms
    }
}