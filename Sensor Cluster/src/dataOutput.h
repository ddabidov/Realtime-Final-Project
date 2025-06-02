#include <Arduino.h>
#include <FreeRTOS.h>
#include <sensorAquisition.h>

// Unified display data struct
typedef struct {
    SensorType_t type;
    union {
        BaroData_t baro;
        AccelData_t accel;
        GPSData_t gps;
    } data;
} DisplayData_t;

void taskDisplay(void *pvParameters);

