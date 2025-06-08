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
extern SemaphoreHandle_t i2cMutex;

void taskBarometer(void *pvParameters) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: taskBarometer started");
    xSemaphoreGive(serialMutex);

    BaroData_t baroData;
    Adafruit_BMP085 bmp;
    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;

    int baroInitAttempts = 0;
    bool baroInit = false;
    while (!baroInit) {
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        baroInit = bmp.begin();
        xSemaphoreGive(i2cMutex);
        if (!baroInit) {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.print("DEBUG: BMP085 init attempt ");
            Serial.println(++baroInitAttempts);
            Serial.println("Could not find a valid BMP085 sensor, check wiring!");
            xSemaphoreGive(serialMutex);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: BMP085 sensor initialized");
    xSemaphoreGive(serialMutex);

    while (true) {
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        baroData.temperature = bmp.readTemperature();
        baroData.pressure = bmp.readPressure();
        xSemaphoreGive(i2cMutex);
        // Altitude calculation is often less critical for raw data logging,
        // but can be included if needed by taskDisplay.
        // baroData.altitude = bmp.readAltitude(); // Uncomment if altitude is needed

        DisplayData_t msg;
        msg.type = SENSOR_BARO;
        msg.data.baro = baroData;

        if (xQueueSend(displayQueue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.println("Barometer: Sent to displayQueue");
            xSemaphoreGive(serialMutex);
        } else {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.println("Barometer: Failed to send to displayQueue");
            xSemaphoreGive(serialMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Add small delay to allow display task to run
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

                if (xQueueSend(displayQueue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {
                    xSemaphoreTake(serialMutex, portMAX_DELAY);
                    Serial.println("GPS: Sent to displayQueue");
                    xSemaphoreGive(serialMutex);
                } else {
                    xSemaphoreTake(serialMutex, portMAX_DELAY);
                    Serial.println("GPS: Failed to send to displayQueue");
                    xSemaphoreGive(serialMutex);
                }
                vTaskDelay(pdMS_TO_TICKS(20)); // Add small delay to allow display task to run
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

    int accelInitAttempts = 0;
    bool accelInit = false;
    while (!accelInit) {
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        accel.initialize();
        accelInit = accel.testConnection();
        xSemaphoreGive(i2cMutex);
        if (!accelInit) {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.print("DEBUG: ADXL345 init attempt ");
            Serial.println(++accelInitAttempts);
            Serial.println("ADXL345 connection failed!");
            xSemaphoreGive(serialMutex);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: ADXL345 sensor initialized");
    xSemaphoreGive(serialMutex);
    
    const float scaleFactor = 1.0f / 256.0f; 

    while (true) {
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        accel.getAcceleration(&ax, &ay, &az);
        xSemaphoreGive(i2cMutex);

        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.print("RAW ACCEL: x="); Serial.print(ax);
        Serial.print(" y="); Serial.print(ay);
        Serial.print(" z="); Serial.println(az);
        xSemaphoreGive(serialMutex);

        accelData.x = ax * scaleFactor;
        accelData.y = ay * scaleFactor;
        accelData.z = az * scaleFactor;

        DisplayData_t msg;
        msg.type = SENSOR_ACCEL;
        msg.data.accel = accelData;

        if (xQueueSend(displayQueue, &msg, pdMS_TO_TICKS(100)) != pdPASS) {
            // Optional: Log queue send failure
        }
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.println("Accelerometer: Sent to displayQueue");
        xSemaphoreGive(serialMutex);
        vTaskDelay(pdMS_TO_TICKS(20)); // Add small delay to allow display task to run
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

void i2cScan() {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("I2C scan start...");
    xSemaphoreGive(serialMutex);
    for (uint8_t addr = 1; addr < 127; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.print("Found I2C device at 0x");
            Serial.println(addr, HEX);
            xSemaphoreGive(serialMutex);
        }
    }
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("I2C scan complete.");
    xSemaphoreGive(serialMutex);
}
