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
#include <Adafruit_SSD1306.h>

int oled_begin = 0;

float barop, barot, accelx, accely, accelz = 0; // Global variables for accelerometer data
GPSData_t gpsData;

Adafruit_SSD1306 display(128, 32, &Wire, -1); // Initialize display with I2C

#define GPS_RX_PIN 13 // Define RX pin for GPS
#define GPS_TX_PIN 12 // Define TX pin for GPS
#define GPS_Serial Serial1 // Assuming GPS is connected to Serial1

void taskBarometer(void *pvParameters) {
    Serial.println("DEBUG: taskBarometer started");

    BaroData_t baroData;
    Adafruit_BMP085 bmp;
    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;

    // Retry initialization until successful
    while (true) {
        if (bmp.begin()) {
            Serial.println("[INFO] BMP085 detected. Barometer task running.");
            break;
        } else {
            Serial.print("[ERROR] BMP085 not detected. ");
            vTaskDelay(pdMS_TO_TICKS(2000));
            break;
        }
    }
    for (;;) {
        Serial.println("DEBUG: Barometer loop start");
        // Heartbeat: print every 2 seconds to show barometer task is alive
        Serial.println("DEBUG: Barometer about to read temperature");
        baroData.temperature = bmp.readTemperature();
        Serial.println("DEBUG: Barometer finished reading temperature");
        Serial.println("DEBUG: Barometer about to read pressure");
        baroData.pressure = bmp.readPressure();
        Serial.println("DEBUG: Barometer finished reading pressure");
        Serial.println("DEBUG: Barometer about to read altitude");
        baroData.altitude = bmp.readAltitude();
        Serial.println("DEBUG: Barometer finished reading altitude");
        Serial.println("DEBUG: Barometer about to send to displayQueue");
        DisplayData_t msg;
        msg.type = SENSOR_BARO;
        msg.data.baro = baroData;
        msg.msSinceStart = millis();
        BaseType_t baroSendResult = xQueueSend(displayQueue, &msg, pdMS_TO_TICKS(100));
        if (baroSendResult != pdPASS) {
            Serial.println("ERROR: Barometer failed to send to displayQueue!");
        } else {
            Serial.println("DEBUG: Barometer sent data to displayQueue");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
    
void taskGPS(void *pvParameters) { 
    TinyGPSPlus gps;
    DisplayData_t msg;
    GPS_Serial.setRX(GPS_RX_PIN); // Set RX pin for GPS
    GPS_Serial.setTX(GPS_TX_PIN); // Set TX pin for GPS

    GPS_Serial.begin(9600); // Initialize GPS serial communication
    while (1)
    {
          
        while (GPS_Serial.available()>0) {
            char nmeaChar = GPS_Serial.read();
            if (gps.encode(nmeaChar)) { 
            
                if (gps.location.isValid()) {
                    msg.data.gps.latitude = gps.location.lat();
                    msg.data.gps.longitude = gps.location.lng();
                }
                if (gps.altitude.isValid()) {
                    msg.data.gps.altitude = gps.altitude.meters();
                } 
                if (gps.date.isValid()) {
                    msg.data.gps.year = gps.date.year();
                    msg.data.gps.month = gps.date.month();
                    msg.data.gps.day = gps.date.day();
                }
                if (gps.time.isValid()) {
                    msg.data.gps.hour = gps.time.hour();
                    msg.data.gps.minute = gps.time.minute();
                    msg.data.gps.second = gps.time.second();
                }
                msg.type = SENSOR_GPS;
                msg.msSinceStart = millis(); // Timestamp in ms since start
                
                QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
                BaseType_t gpsSendResult = xQueueSend(displayQueue, &msg, pdMS_TO_TICKS(100));
                if (gpsSendResult != pdPASS) {
                    Serial.println("ERROR: GPS failed to send to displayQueue!");
                } else {
                    Serial.println("DEBUG: GPS sent data to displayQueue");
                }
            } 
        }
 } // Commented out GPS task implementation
}

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
            vTaskDelay(pdMS_TO_TICKS(100));
            accel.initialize(); // Retry initialization
        }
    }
    const float scaleFactor = 1.0f / 256.0f;
    while (true) {
        Serial.println("DEBUG: Accel loop start");
        Serial.println("DEBUG: Accel about to read sensors");
        Serial.println("DEBUG: Accel about to read acceleration");
        accel.getAcceleration(&ax, &ay, &az);
        Serial.println("DEBUG: Accel finished reading acceleration");
        accelData.x = ax * (1.0f / 256.0f);
        accelData.y = ay * (1.0f / 256.0f);
        accelData.z = az * (1.0f / 256.0f);
        Serial.println("DEBUG: Accel about to send to displayQueue");
        DisplayData_t msg;
        msg.type = SENSOR_ACCEL;
        msg.data.accel = accelData;
        msg.msSinceStart = millis();
        BaseType_t accelSendResult = xQueueSend(displayQueue, &msg, pdMS_TO_TICKS(100));
        if (accelSendResult != pdPASS) {
            Serial.println("ERROR: Accel failed to send to displayQueue!");
        } else {
            Serial.println("DEBUG: Accel sent data to displayQueue");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void oled_display(const DisplayData_t& data) {
    
    if (oled_begin == 0) {
        if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
            Serial.println("SSD1306 allocation failed");
            return; // Exit if display initialization fails
        }
        oled_begin = 1; // Set flag to indicate display has been initialized
        display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Initialize display
        display.clearDisplay(); // Clear the display buffer
    }

        switch (data.type){
        case(SENSOR_ACCEL):
            accelx = data.data.accel.x; // Store accelerometer data in global variables
            accely = data.data.accel.y;
            accelz = data.data.accel.z;
            break;
        case(SENSOR_BARO):
            barop = data.data.baro.pressure; // Store barometer data in global variables
            barot = data.data.baro.temperature;
            break;
        case(SENSOR_GPS):
            gpsData = data.data.gps; // Store GPS data in global variables
            break;
        }
    display.clearDisplay(); // Clear the display buffer
    display.setTextSize(1);                     // Set text size to 1   
    display.setTextColor(SSD1306_WHITE);        // Set text color to white
    
    
    display.setCursor(0, 0);
    display.print(barop);
    display.print(" ");
    display.print(barot);

    display.setCursor(0, 10);
    display.print(accelx);
    display.print(" ");
    display.print(accely);
    display.print(" ");
    display.print(accelz); // Print accelerometer data

    display.setCursor(0, 20);
    display.print(gpsData.latitude, 6); // Print GPS latitude
    display.print(" ");
    display.print(gpsData.longitude, 6); // Print GPS longitude 
    display.print(" ");
    display.print(gpsData.altitude, 2); // Print GPS altitude
            
    display.display();

}
    

// Display task: handles any sensor data received
void taskDisplay(void *pvParameters) {
    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    DisplayData_t data;

    Serial.println("I made it to step 1");
    // Debug: at this point, the code stops running and nothing else is recieved from the serial monitor
    Serial.println("I made it to step 2");
    Serial.println("DEBUG: taskDisplay started and running.");
    ///Serial.println("TEST TEST TEST");

    for (;;) {
        Serial.println("DEBUG: taskDisplay waiting for queue data...");
        if (xQueueReceive(displayQueue, &data, portMAX_DELAY) == pdPASS) {
            //if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
            //    Serial.println("SSD1306 allocation failed");
            //}
            Serial.println("DEBUG: taskDisplay received data, processing...");
            
            switch (data.type) {
                case SENSOR_BARO:
                    Serial.print("[BARO] P: ");
                    Serial.print(data.data.baro.pressure);
                    Serial.print(" T: ");
                    Serial.println(data.data.baro.temperature);                          //Also i did not commit any of this stuff
                    break;
                case SENSOR_ACCEL:
                    Serial.print("[ACCEL] X: ");
                    Serial.print(data.data.accel.x);
                    Serial.print(" Y: ");
                    Serial.print(data.data.accel.y);
                    Serial.print(" Z: ");
                    Serial.println(data.data.accel.z);              
                    break;
                case SENSOR_GPS:
                    Serial.print("[GPS] Lat: ");
                    Serial.print(data.data.gps.latitude, 6);
                    Serial.print(" Lon: ");
                    Serial.print(data.data.gps.longitude, 6);
                    Serial.print(" Alt: ");
                    Serial.println(data.data.gps.altitude, 2);
                    
                    break;
                default:
                    Serial.println("[UNKNOWN SENSOR DATA]");
            }
        } else {
            Serial.println("DEBUG: taskDisplay timed out waiting for queue");
        }
        oled_display(data); // Call the OLED display function with the received data
        vTaskDelay(pdMS_TO_TICKS(70)); // Yield to other tasks
        display.clearDisplay(); // Clear the display buffer
    
    }
}
