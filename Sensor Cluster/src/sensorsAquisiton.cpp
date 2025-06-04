#include "sensorAquisition.h"
#include <Arduino.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <TinyGPSPlus.h>   
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "FreeRTOS.h"



void taskBarometer(void *pvParameters) {
    BaroData_t baroData;
    QueueHandle_t barometerQueue = (QueueHandle_t)pvParameters;
    Serial.begin(9600);
	  while (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        delay(500);
        if (bmp.begin()){
            break;
            }
        }
    }

    while (true) {
    Serial.print("Temperature = ");
    Serial.println(" *C");
    
    baroData.temperature = bmp.readTemperature();
    baroData.pressure = bmp.readPreasure();
    baroData.altitude = bmp.readAltitude();
    xQueueSend(barometerQueue, baroData, 0);

    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");
    
    Serial.println();
    delay(500);
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