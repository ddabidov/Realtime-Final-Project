#include <Arduino.h>
#include <FreeRTOS.h>
#include <queue.h>
#include "sensorAquisition.h"
#include "dataOutput.h"


QueueHandle_t displayQueue;
QueueHandle_t barometerQueue;
QueueHandle_t accelQueue;
QueueHandle_t gpsQueue;
// Sensor data structures


// Forward declarations
void taskBarometer(void *pvParameters);
void taskAccelerometer(void *pvParameters);
void taskGPS(void *pvParameters);
void taskDisplay(void *pvParameters);

void setup() {
    Serial.begin(115200);

    // Create display queue
    displayQueue = xQueueCreate(10, sizeof(DisplayData_t));
    barometerQueue = xQueueCreate(10, sizeof(BaroData_t));
    accelQueue = xQueueCreate(10, sizeof(AccelData_t));
    gpsQueue = xQueueCreate(10, sizeof(GPSData_t));

    // Create sensor tasks, pass displayQueue to each
    xTaskCreate(taskBarometer, "Baro", 1024, (void*)barometerQueue, 2, NULL);
    vTaskCoreAffinitySet((TaskHandle_t) taskBarometer,0x01); // Set barometer task to core 0
    xTaskCreate(taskAccelerometer, "Accel", 1024, (void*)accelQueue, 2, NULL);
    vTaskCoreAffinitySet((TaskHandle_t) taskAccelerometer,0x01); // Set Accelerometer task to core 0
    xTaskCreate(taskGPS, "GPS", 1024, (void*)gpsQueue, 2, NULL);
    vTaskCoreAffinitySet((TaskHandle_t) taskGPS,0x01); // Set GPS task to core 0

    // Create display task (core 1)
    xTaskCreate(taskDisplay, "Display", 2048, displayQueue, 1, NULL);
    vTaskCoreAffinitySet((TaskHandle_t) taskDisplay,0x02); // Set barometer task to core 0
}

void loop() {
    // Not used with FreeRTOS
}


