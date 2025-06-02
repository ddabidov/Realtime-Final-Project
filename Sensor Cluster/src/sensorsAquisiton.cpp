#include "sensorsAquisiton.h"
#include <Arduino.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <TinyGPSPlus.h>   
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "FreeRTOS.h"



void taskBarometer(void *pvParameters) {
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

        // Print the data to the serial monitor
        Serial.print("Pressure: ");
        Serial.print(pressure);
        Serial.print(" hPa, Temperature: ");
        Serial.print(temperature);
        Serial.println(" °C");

        // Delay for a while before the next reading
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskGPS(void *pvParameters) {
    // Initialize the GPS sensor
    if (!gps.begin()) {
        Serial.println("Failed to initialize GPS!");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        // Read the GPS data
        if (gps.available()) {
            String location = gps.readLocation();
            Serial.print("GPS Location: ");
            Serial.println(location);
        } else {
            Serial.println("No GPS data available.");
        }

        // Delay for a while before the next reading
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskAccelerometer(void *pvParameters) {
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
        Serial.print("ADXL Acceleration - X: ");
        Serial.print(x);
        Serial.print(", Y: ");
        Serial.print(y);
        Serial.print(", Z: ");
        Serial.println(z);

        // Delay for a while before the next reading
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}