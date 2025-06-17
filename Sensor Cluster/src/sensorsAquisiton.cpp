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
        static unsigned long lastHeartbeat = 0;
        if (millis() - lastHeartbeat > 2000) {
            Serial.println("[HEARTBEAT] Barometer task alive");
            lastHeartbeat = millis();
        }
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
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
    
void taskGPS(void *pvParameters) { Serial.println("GPS TASK"); } // Commented out GPS task implementation

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



// Display task: handles any sensor data received
void taskDisplay(void *pvParameters) {
    //Adafruit_SSD1306 display(128, 32, &Wire, -1); // Initialize display with I2C
    //if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    //    Serial.println("SSD1306 allocation failed");
    //    for(;;); // Halt if display fails
    //}
    //display.clearDisplay(); // Clear the display buffer

    
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
            Serial.println("DEBUG: taskDisplay received data, processing...");

            switch (data.type) {
                case SENSOR_BARO:
                    Serial.print("[BARO] P: ");
                    Serial.print(data.data.baro.pressure);
                    Serial.print(" T: ");
                    Serial.println(data.data.baro.temperature);
                    //display.setCursor(0, 0);                    //THIS SHIT PRINTS NOTHING AT ALL I AM GOING TO KILL MYSELF
                    //display.setTextSize(1);                     //AND OUR PARTNER IS USELESS LIKE WHAT THE FUCK WHAT
                    //display.setTextColor(SSD1306_WHITE);        //DO YOU MEAN THAT YOU FUCKING CHATGPT'ED THE WHOLE REPORT
                    //display.print(F("[BARO] P: "));             //JSUT FUCKING DO IT YOURSELF YOU TWAT
                    //display.display();                          //Also i did not commit any of this stuff
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
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield to other tasks
    }
}