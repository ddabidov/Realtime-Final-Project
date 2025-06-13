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
#include "I2Cdev.h"
#include "ADXL345.h"


void taskBarometer(void *pvParameters) {
    Serial.println("DEBUG: taskBarometer started");

    BaroData_t baroData;
    Adafruit_BMP085 bmp;
    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;

    // Retry initialization until successful
    int baroInitAttempts = 0;
    while (true) {
        if (bmp.begin()) {
            Serial.println("[INFO] BMP085 detected. Barometer task running.");
            break;
        } else {
            Serial.print("[ERROR] BMP085 not detected. Barometer task waiting... Attempt ");
            Serial.println(baroInitAttempts);
            baroInitAttempts++;
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    for (;;) {
        // Heartbeat: print every 2 seconds to show barometer task is alive
        static unsigned long lastHeartbeat = 0;
        if (millis() - lastHeartbeat > 2000) {
            Serial.println("[HEARTBEAT] Barometer task alive");
            lastHeartbeat = millis();
        }

        baroData.temperature = bmp.readTemperature();
        baroData.pressure = bmp.readPressure();
        baroData.altitude = bmp.readAltitude();

        DisplayData_t msg;
        msg.type = SENSOR_BARO;
        msg.data.baro = baroData;
        xQueueSend(displayQueue, &msg, pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
    
// void taskGPS(void *pvParameters) { ... } // Commented out GPS task implementation

void taskAccelerometer(void *pvParameters) {
    Serial.println("DEBUG: taskAccelerometer started");

    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    ADXL345 accel;
    AccelData_t accelData;
    int16_t ax, ay, az; // Raw accelerometer values

    // Retry initialization until successful
    int accelInitAttempts = 0;
    accel.initialize(); // Initialize ADXL345
    while (true) {
        if (accel.testConnection()) {
            Serial.println("DEBUG: ADXL345 sensor initialized");
            break;
        } else {
            if (accelInitAttempts % 10 == 0) {
                Serial.print("DEBUG: ADXL345 init attempt ");
                Serial.println(accelInitAttempts);
                Serial.println("ADXL345 connection failed!");
            }
            accelInitAttempts++;
            vTaskDelay(pdMS_TO_TICKS(500));
            accel.initialize(); // Retry initialization
        }
    }
    const float scaleFactor = 1.0f / 256.0f;
    while (true) {
        accel.getAcceleration(&ax, &ay, &az);
        accelData.x = ax * scaleFactor;
        accelData.y = ay * scaleFactor;
        accelData.z = az * scaleFactor;

        DisplayData_t msg;
        msg.type = SENSOR_ACCEL;
        msg.data.accel = accelData;
        xQueueSend(displayQueue, &msg, pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void i2cScan(void) {
    // Stub implementation for linker
}

    }
}


