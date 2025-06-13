#include <Arduino.h>
#include <FreeRTOS.h>  
#include <queue.h>
#include "sensorAquisition.h"
#include "dataOutput.h"
#include "semphr.h" // Added for serialMutex

// Declare serialMutex as an external variable
extern SemaphoreHandle_t serialMutex;

// Display task: handles any sensor data received
void taskDisplay(void *pvParameters) {
    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println("DEBUG: taskDisplay entered main function.");
        xSemaphoreGive(serialMutex);
    }
    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    DisplayData_t data;

    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println("DEBUG: taskDisplay started and running.");
        xSemaphoreGive(serialMutex);
    }

    for (;;) {
        // Heartbeat: print every 2 seconds to show display task is alive
        static unsigned long lastHeartbeat = 0;
        if (millis() - lastHeartbeat > 2000) {
            if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
                Serial.println("[HEARTBEAT] Display task alive");
                xSemaphoreGive(serialMutex);
            }
            lastHeartbeat = millis();
        }

        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
            Serial.println("DEBUG: taskDisplay waiting for queue data...");
            xSemaphoreGive(serialMutex);
        }
        
        if (xQueueReceive(displayQueue, &data, portMAX_DELAY) == pdPASS) {
            if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
                Serial.println("DEBUG: taskDisplay received data, processing...");
                switch (data.type) {
                    case SENSOR_BARO:
                        Serial.print("[BARO] t=");
                        Serial.print(data.msSinceStart);
                        Serial.print("ms P: ");
                        Serial.print(data.data.baro.pressure);
                        Serial.print(" T: ");
                        Serial.println(data.data.baro.temperature);
                        break;
                    case SENSOR_ACCEL:
                        Serial.print("[ACCEL] t=");
                        Serial.print(data.msSinceStart);
                        Serial.print("ms X: ");
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
                xSemaphoreGive(serialMutex);
            }
        } else {
            if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
                Serial.println("DEBUG: taskDisplay failed to receive data from queue."); // Should not happen with portMAX_DELAY unless queue is deleted
                xSemaphoreGive(serialMutex);
            }
        }
    }
}