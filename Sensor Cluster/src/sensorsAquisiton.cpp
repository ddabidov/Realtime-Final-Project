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
    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;

    int baroInitAttempts = 0;
    while (!bmp.begin()) {
        xSemaphoreTake(serialMutex, portMAX_DELAY); // Added missing semaphore take
        Serial.print("DEBUG: BMP085 init attempt ");
        Serial.println(++baroInitAttempts);
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        xSemaphoreGive(serialMutex);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: BMP085 sensor initialized");
    xSemaphoreGive(serialMutex);

    while (true) {
        baroData.temperature = bmp.readTemperature();
        baroData.pressure = bmp.readPressure();
        // Altitude calculation is often less critical for raw data logging,
        // but can be included if needed by taskDisplay.
        // baroData.altitude = bmp.readAltitude(); // Uncomment if altitude is needed

        DisplayData_t msg;
        msg.type = SENSOR_BARO;
        msg.data.baro = baroData;

        if (xQueueSend(displayQueue, &msg, pdMS_TO_TICKS(100)) != pdPASS) {
            // Optional: Log queue send failure (e.g., increment a counter)
            // Avoid Serial.print here to prevent re-introducing contention
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

    while (true) {
        bool sentenceProcessedThisPass = false;
        while (gpsSerial.available() > 0) {
            if (gps.encode(gpsSerial.read())) { // True if a new NMEA sentence is complete
                sentenceProcessedThisPass = true;
            }
        }

        if (sentenceProcessedThisPass) {
            if (gps.location.isUpdated() && gps.location.isValid()) {
                gpsData.latitude = gps.location.lat();
                gpsData.longitude = gps.location.lng();
                
                if (gps.altitude.isUpdated() && gps.altitude.isValid()) {
                    gpsData.altitude = gps.altitude.meters();
                } else {
                    gpsData.altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0.0; 
                }

                DisplayData_t msg;
                msg.type = SENSOR_GPS;
                msg.data.gps = gpsData;

                if (xQueueSend(displayQueue, &msg, pdMS_TO_TICKS(100)) != pdPASS) {
                    // Optional: Log queue send failure
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
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.print("DEBUG: ADXL345 init attempt ");
        Serial.println(++accelInitAttempts);
        Serial.println("ADXL345 connection failed!");
        xSemaphoreGive(serialMutex);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // accel.initialize(); // Optionally re-attempt initialization
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

        DisplayData_t msg;
        msg.type = SENSOR_ACCEL;
        msg.data.accel = accelData;

        if (xQueueSend(displayQueue, &msg, pdMS_TO_TICKS(100)) != pdPASS) {
            // Optional: Log queue send failure
        }
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

void i2cScan() {
    // ...existing code...
}
