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
    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    DisplayData_t data;

    // Debug: Confirm display task is running
    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println("DEBUG: taskDisplay running");
        xSemaphoreGive(serialMutex);
    }

    for (;;) {
        if (xQueueReceive(displayQueue, &data, pdMS_TO_TICKS(1000)) == pdPASS) {
            if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
                Serial.println("DEBUG: taskDisplay received data");
                switch (data.type) {
                    case SENSOR_BARO:
                        Serial.print("[BARO] P: ");
                        Serial.print(data.data.baro.pressure);
                        Serial.print(" T: ");
                        Serial.println(data.data.baro.temperature);
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
                xSemaphoreGive(serialMutex);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
                Serial.println("DEBUG: taskDisplay timed out waiting for queue");
                xSemaphoreGive(serialMutex);
            }
        }
    }
}