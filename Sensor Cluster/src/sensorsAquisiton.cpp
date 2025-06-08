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
#include <Adafruit_ADXL345_U.h>
#include "FreeRTOS.h"
#include <queue.h>
#include "semphr.h"
#include "I2Cdev.h"


extern SemaphoreHandle_t serialMutex;

void taskBarometer(void *pvParameters) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: taskBarometer started");
    xSemaphoreGive(serialMutex);

    i2cScan(); // Add this line

    BaroData_t baroData;
    Adafruit_BMP085 bmp;
    QueueHandle_t barometerQueue = (QueueHandle_t)pvParameters;

    // Barometer initialization loop with debug prints
    int baroInitAttempts = 0;
    while (!bmp.begin()) {
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.print("DEBUG: BMP085 init attempt ");
        Serial.println(++baroInitAttempts);
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        if (bmp.begin()){
            break;
            }
        }
        xSemaphoreGive(serialMutex);
        vTaskDelay(500);
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.println("DEBUG: BMP085 sensor initialized");
        xSemaphoreGive(serialMutex);

    while (true) {
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.println("DEBUG: taskBarometer loop running");
        Serial.print("Temperature = ");
        Serial.println(bmp.readTemperature());
        baroData.temperature = bmp.readTemperature();
        baroData.pressure = bmp.readPressure();
        baroData.altitude = bmp.readAltitude();
        if (xQueueSend(barometerQueue, &baroData, 0) == pdPASS) {
            Serial.println("DEBUG: Barometer data sent to queue");
        } else {
            Serial.println("DEBUG: Barometer data NOT sent to queue");
        }
        Serial.print("Pressure = ");
        Serial.print(bmp.readPressure());
        Serial.println(" Pa");
        Serial.print("Altitude = ");
        Serial.print(bmp.readAltitude());
        Serial.println(" meters");
        Serial.print("Pressure at sealevel (calculated) = ");
        Serial.print(bmp.readSealevelPressure());
        Serial.println(" Pa");
        Serial.print("Real altitude = ");
        Serial.print(bmp.readAltitude(101500));
        Serial.println(" meters");
        Serial.println();
        xSemaphoreGive(serialMutex);

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
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void displayInfo()
{
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    TinyGPSPlus gps;
    Serial.print(F("Location: ")); 
    if (gps.location.isValid())
    {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.println();
    xSemaphoreGive(serialMutex);
}
    
void taskGPS(void *pvParameters) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: taskGPS started");
    xSemaphoreGive(serialMutex);

    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    TinyGPSPlus gps;

    // GPS initialization loop with debug prints (if needed)
    int gpsInitAttempts = 0;
    bool gpsSerialReady = false;
    while (!gpsSerialReady) {
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.print("DEBUG: GPS Serial init attempt ");
        Serial.println(++gpsInitAttempts);
        xSemaphoreGive(serialMutex);

        gpsSerial.setRX(GPS_RX_PIN);
        gpsSerial.setTX(GPS_TX_PIN);
        gpsSerial.begin(9600);
        vTaskDelay(500);
        // Optionally check if GPS is sending data here
        if (gpsSerial) gpsSerialReady = true;
    }
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: GPS Serial initialized");
    Serial.println(F("BasicExample.ino"));
    Serial.println(F("Basic demonstration of TinyGPSPlus with hardware serial"));
    Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
    Serial.println(F("by Mikal Hart"));
    Serial.println();
    xSemaphoreGive(serialMutex);

    while (true) {
        static unsigned long lastDataTime = 0;
        static unsigned long lastCheckTime = 0;
        static bool gpsEverReceived = false;

        while (gpsSerial.available() > 0) {
            char c = gpsSerial.read();
            lastDataTime = millis();
            gpsEverReceived = true;
            if (gps.encode(c)) {
                xSemaphoreTake(serialMutex, portMAX_DELAY);
                Serial.println("DEBUG: GPS data parsed");
                xSemaphoreGive(serialMutex);
                displayInfo();
            }
        }

        if (millis() - lastCheckTime > 1000) {
            lastCheckTime = millis();
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            if (!gpsEverReceived) {
                Serial.println(F("GPS not detected: No data received yet."));
            } else if (millis() - lastDataTime > 2000) {
                Serial.println(F("GPS disconnected or not sending data!"));
            } else {
                Serial.println(F("GPS connected: Data is being received."));
            }
            xSemaphoreGive(serialMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskAccelerometer(void *pvParameters) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: taskAccelerometer started");
    xSemaphoreGive(serialMutex);

    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    Adafruit_ADXL345_Unified adxl;
    sensors_event_t event;

    // Accelerometer initialization loop with debug prints
    int accelInitAttempts = 0;
    while (!adxl.begin()) {
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Wire.begin();
        accel.initialize();
        if(accel.testConnection()){
            break;
        } 
        Serial.print("DEBUG: ADXL345 init attempt ");
        Serial.println(++accelInitAttempts);
        Serial.println("Failed to initialize ADXL345! Check wiring.");
        xSemaphoreGive(serialMutex);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: ADXL345 sensor initialized");
    xSemaphoreGive(serialMutex);

    while (true) {
        int16_t ax, ay, az;
        accel.getAcceleration(&ax, &ay, &az);

        DisplayData_t msg;
        msg.type = SENSOR_ACCEL;
        msg.data.accel.x = ax;
        msg.data.accel.y = ay;
        msg.data.accel.z = az;

        xSemaphoreTake(serialMutex, portMAX_DELAY);
        if (xQueueSend(displayQueue, &msg, 0) == pdPASS) {
            Serial.println("DEBUG: Accelerometer data sent to queue");
        } else {
            Serial.println("DEBUG: Accelerometer data NOT sent to queue");
        }
        xSemaphoreGive(serialMutex);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void i2cScan() {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: Starting I2C scan...");
    xSemaphoreGive(serialMutex);
    Wire.begin();
    for (uint8_t addr = 1; addr < 127; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.print("I2C device found at address 0x");
            Serial.println(addr, HEX);
            xSemaphoreGive(serialMutex);
        }
    }
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("DEBUG: I2C scan complete.");
    xSemaphoreGive(serialMutex);
}