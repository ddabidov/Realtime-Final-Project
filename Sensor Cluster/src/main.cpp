#include <Arduino.h>
#include <FreeRTOS.h>
#include <queue.h>
#include "sensorAquisition.h"
#include "dataOutput.h"
#include <Wire.h> // Added for Wire.begin()


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

TaskHandle_t baroTaskHandle = NULL;
TaskHandle_t accelTaskHandle = NULL;
TaskHandle_t gpsTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;

void setup() {
    Serial.begin(115200);
    Serial.println("DEBUG: Serial.begin called");
    while (!Serial) { delay(10); }
    Serial.println("DEBUG: Serial connection established");
    delay(1000); // Give time for USB CDC to initialize
    Serial.println("DEBUG: Post USB CDC delay");

    Wire.begin(); // Initialize I2C bus
    Serial.println("DEBUG: Wire.begin() called in setup");

    // Remove mutex creation
    // Run I2C scan before creating tasks
    i2cScan();

    displayQueue = xQueueCreate(20, sizeof(DisplayData_t));
    Serial.println("DEBUG: displayQueue created");
    barometerQueue = xQueueCreate(10, sizeof(BaroData_t));
    Serial.println("DEBUG: barometerQueue created");
    accelQueue = xQueueCreate(10, sizeof(AccelData_t));
    Serial.println("DEBUG: accelQueue created");
    gpsQueue = xQueueCreate(10, sizeof(GPSData_t));
    Serial.println("DEBUG: gpsQueue created");

    Serial.println("DEBUG: Creating tasks...");
    xTaskCreate(taskBarometer, "Baro", 2048, (void*)displayQueue, 2, &baroTaskHandle);
    xTaskCreate(taskBarometer, "Baro", 2048, (void*)displayQueue, 2, &baroTaskHandle);
    Serial.println("DEBUG: taskBarometer created");
    xTaskCreate(taskAccelerometer, "Accel", 2048, (void*)displayQueue, 2, &accelTaskHandle);
    xTaskCreate(taskAccelerometer, "Accel", 2048, (void*)displayQueue, 2, &accelTaskHandle);
    Serial.println("DEBUG: taskAccelerometer created");
    xTaskCreate(taskGPS, "GPS", 2048, (void*)displayQueue, 2, &gpsTaskHandle);
    xTaskCreate(taskGPS, "GPS", 2048, (void*)displayQueue, 2, &gpsTaskHandle);
    Serial.println("DEBUG: taskGPS created");
    xTaskCreate(taskDisplay, "Display", 4096, (void*)displayQueue, 3, &displayTaskHandle); // Increased stack size
    Serial.println("DEBUG: taskDisplay created");

    Serial.println("DEBUG: setup() complete");

    vTaskStartScheduler();
    // If we ever get here, scheduler failed to start or returned
    Serial.println("[FATAL] vTaskStartScheduler() returned! System halted.");
    for (;;) {}
}

// FreeRTOS idle hook
extern "C" void vApplicationIdleHook(void) {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 2000) {
        Serial.println("DEBUG: Idle task running");
        lastPrint = millis();
    }
}

void loop() {
    // Not used with FreeRTOS
}

// Remove duplicate vApplicationStackOverflowHook if present


