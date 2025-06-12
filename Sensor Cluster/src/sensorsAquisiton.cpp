// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

//both BMP180 and ADXL345 use I2C so they can share the same SDA and SCL lines (gpio 4 and 5 on the pi pico)



#include "sensorAquisition.h"
#include <Arduino.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include <TinyGPSPlus.h>   
#include <Adafruit_Sensor.h>
#include "FreeRTOS.h"
#include <queue.h>
#include "semphr.h"
#include "I2Cdev.h"
#include "ADXL345.h"


extern SemaphoreHandle_t serialMutex;

void taskBarometer(void *pvParameters) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: taskBarometer started");
    xSemaphoreGive(serialMutex);

    BaroData_t baroData;
    Adafruit_BMP085 bmp;
    // QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;

    int baroInitAttempts = 0;
    while (!bmp.begin()) {
        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
            Serial.println("[ERROR] BMP085 not detected. Barometer task waiting...");
            xSemaphoreGive(serialMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println("[INFO] BMP085 detected. Barometer task running.");
        xSemaphoreGive(serialMutex);
    }
    for (;;) {
        // Heartbeat: print every 2 seconds to show barometer task is alive
        static unsigned long lastHeartbeat = 0;
        if (millis() - lastHeartbeat > 2000) {
            if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
                Serial.println("[HEARTBEAT] Barometer task alive");
                xSemaphoreGive(serialMutex);
            }
            lastHeartbeat = millis();
        }

        baroData.temperature = bmp.readTemperature();
        baroData.pressure = bmp.readPressure();
        // Altitude calculation is often less critical for raw data logging,
        // but can be included if needed by taskDisplay.
        // baroData.altitude = bmp.readAltitude(); // Uncomment if altitude is needed

        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print("[BARO] P: ");
            Serial.print(baroData.pressure);
            Serial.print(" T: ");
            Serial.println(baroData.temperature);
            xSemaphoreGive(serialMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Adjust delay as per desired sampling rate
    }
}

// Remove the displayInfo() function as its functionality is moved to taskDisplay
/*
void displayInfo()
{
    // ... old content removed ...
}
*/
    
void taskGPS(void *pvParameters) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: taskGPS started");
    xSemaphoreGive(serialMutex);

    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    TinyGPSPlus gps;
    GPSData_t gpsData;

    // GPS Serial port is Serial1, pins defined in sensorAquisition.h
    gpsSerial.setRX(GPS_RX_PIN);
    gpsSerial.setTX(GPS_TX_PIN);
    gpsSerial.begin(9600); // Common baud rate for GPS modules

    vTaskDelay(pdMS_TO_TICKS(100)); // Small delay for serial to settle

    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: GPS Serial configured for taskGPS");
    xSemaphoreGive(serialMutex);

    for (;;) {
        bool sentenceProcessedThisPass = false;
        int bytesProcessed = 0;
        while (gpsSerial.available() > 0 && bytesProcessed < 128) { // Limit bytes per loop
            if (gps.encode(gpsSerial.read())) {
                sentenceProcessedThisPass = true;
            }
            bytesProcessed++;
        }
        // Always yield, even if no data
        vTaskDelay(pdMS_TO_TICKS(20));

        if (sentenceProcessedThisPass) {
            if (gps.location.isUpdated() && gps.location.isValid()) {
                gpsData.latitude = gps.location.lat();
                gpsData.longitude = gps.location.lng();
                
                if (gps.altitude.isUpdated() && gps.altitude.isValid()) {
                    gpsData.altitude = gps.altitude.meters();
                } else {
                    gpsData.altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0.0; 
                }

                if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
                    Serial.print("[GPS] Lat: ");
                    Serial.print(gpsData.latitude, 6);
                    Serial.print(" Lon: ");
                    Serial.print(gpsData.longitude, 6);
                    Serial.print(" Alt: ");
                    Serial.println(gpsData.altitude, 2);
                    xSemaphoreGive(serialMutex);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Poll GPS serial buffer frequently
    }
}

void taskAccelerometer(void *pvParameters) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: taskAccelerometer started");
    xSemaphoreGive(serialMutex);

    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    ADXL345 accel;
    AccelData_t accelData;
    int16_t ax, ay, az; // Raw accelerometer values

    // Wire.begin(); // Ensure Wire is initialized for I2C communication <- REMOVED, moved to setup()
    accel.initialize(); // Initialize ADXL345

    int accelInitAttempts = 0;
    while (!accel.testConnection()) {
        if (accelInitAttempts % 10 == 0) { // Only print every 10 attempts
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.print("DEBUG: ADXL345 init attempt ");
            Serial.println(accelInitAttempts);
            Serial.println("ADXL345 connection failed!");
            xSemaphoreGive(serialMutex);
        }
        accelInitAttempts++;
        vTaskDelay(pdMS_TO_TICKS(500)); // Always yield!
    }

    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: ADXL345 sensor initialized");
    xSemaphoreGive(serialMutex);
    
    const float scaleFactor = 1.0f / 256.0f; 

    while (true) {
        accel.getAcceleration(&ax, &ay, &az); 

        accelData.x = ax * scaleFactor;
        accelData.y = ay * scaleFactor;
        accelData.z = az * scaleFactor;

        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print("[ACCEL] X: ");
            Serial.print(accelData.x);
            Serial.print(" Y: ");
            Serial.print(accelData.y);
            Serial.print(" Z: ");
            Serial.println(accelData.z);
            xSemaphoreGive(serialMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

void i2cScan() {
    // ...existing code...
}
