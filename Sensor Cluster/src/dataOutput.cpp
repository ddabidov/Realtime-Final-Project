#include <Arduino.h>
#include <FreeRTOS.h>  
#include <queue.h>
#include "sensorAquisition.h"
#include "dataOutput.h"

extern QueueHandle_t displayQueue;

// Display task: handles any sensor data received
void taskDisplay(void *pvParameters) {
    DisplayData_t data;
    for (;;) {
        if (xQueueReceive(displayQueue, &data, portMAX_DELAY) == pdPASS) {
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
        }
    }
}