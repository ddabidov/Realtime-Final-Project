#include <Arduino.h>
#include <FreeRTOS.h>  
#include <queue.h>
#include "sensorAquisition.h"
#include "dataOutput.h"
#include "semphr.h" // Added for serialMutex
#include <Adafruit_SSD1306.h>


Adafruit_SSD1306 display(128, 32, &Wire, -1); // Initialize display with I2C

// Display task: handles any sensor data received
void taskDisplay(void *pvParameters) {
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 allocation failed");
        for(;;); // Halt if display fails
    }
    display.clearDisplay(); // Clear the display buffer

    
    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    DisplayData_t data;

    // Debug: Confirm display task is running
    Serial.println("DEBUG: taskDisplay started and running.");

    for (;;) {
        // Heartbeat: print every 2 seconds to show display task is alive
        static unsigned long lastHeartbeat = 0;
        if (millis() - lastHeartbeat > 2000) {
            Serial.println("[HEARTBEAT] Display task alive");
            lastHeartbeat = millis();
        }

        Serial.println("DEBUG: taskDisplay waiting for queue data...");
        if (xQueueReceive(displayQueue, &data, portMAX_DELAY) == pdPASS) {
            Serial.println("DEBUG: taskDisplay received data, processing...");

            switch (data.type) {
                case SENSOR_BARO:
                    Serial.print("[BARO] P: ");
                    Serial.print(data.data.baro.pressure);
                    Serial.print(" T: ");
                    Serial.println(data.data.baro.temperature);
                    display.setCursor(0, 0);                    //THIS SHIT PRINTS NOTHING AT ALL I AM GOING TO KILL MYSELF
                    display.setTextSize(1);                     //AND OUR PARTNER IS USELESS LIKE WHAT THE FUCK WHAT
                    display.setTextColor(SSD1306_WHITE);        //DO YOU MEAN THAT YOU FUCKING CHATGPT'ED THE WHOLE REPORT
                    display.print(F("[BARO] P: "));             //JSUT FUCKING DO IT YOURSELF YOU TWAT
                    display.display();                          //Also i did not commit any of this stuff
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
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            Serial.println("DEBUG: taskDisplay timed out waiting for queue");
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Yield to other tasks
    }
}