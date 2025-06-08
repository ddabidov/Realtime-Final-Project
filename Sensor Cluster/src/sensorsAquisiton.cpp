// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here


/*                                  |
//                                  |
//                             _____|_____
//                             \         /
//                              \       / 
//                               \     /
//                                \   / 
//                                 \ /
//                                  V
//****************************************************************************
//PLUG SCL INTO GP5 AND SDA INTO GP4 ON PI PICO!!!!!!!!!!!!!!
//****************************************************************************
//                                  ^
//                                 / \
//                                /   \
//                               /     \
//                              /       \
//                             /         \ 
//                             -----------
//                                  |
//                                  |
//                                  |
*/



#include "sensorAquisition.h"
#include <Arduino.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include <TinyGPSPlus.h>   
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "FreeRTOS.h"



void taskBarometer(void *pvParameters) {
    BaroData_t baroData;
    QueueHandle_t barometerQueue = (QueueHandle_t)pvParameters;
    Serial.begin(9600);
	  while (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        osdelay(500);
        if (bmp.begin()){
            break;
            }
        }
    }

    while (true) {
    Serial.print("Temperature = ");
    Serial.println(" *C");
    
    baroData.temperature = bmp.readTemperature();
    baroData.pressure = bmp.readPreasure();
    baroData.altitude = bmp.readAltitude();
    xQueueSend(barometerQueue, baroData, 0);

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
    }


void displayInfo()
{
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
}
    
void taskGPS(void *pvParameters) {
    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    // Initialize the GPS sensor
    TinyGPSPlus gps;

    gpsSerial.setRX(GPS_RX_PIN);
    gpsSerial.setTX(GPS_TX_PIN);
    gpsSerial.begin(9600); // Start GPS serial on Serial1 with specified pins

    Serial.println(F("BasicExample.ino"));
    Serial.println(F("Basic demonstration of TinyGPSPlus with hardware serial"));
    Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
    Serial.println(F("by Mikal Hart"));
    Serial.println();

    while (true) {
        static unsigned long lastDataTime = 0;
    static unsigned long lastCheckTime = 0;
    static bool gpsEverReceived = false;

    // Check for incoming data from GPS
    while (gpsSerial.available() > 0)
    {
      char c = gpsSerial.read();
      lastDataTime = millis(); // Update time of last received byte
      gpsEverReceived = true;
      if (gps.encode(c))
        displayInfo();
      }

    // Periodically check GPS connection status
    if (millis() - lastCheckTime > 1000) // Every 1 second
    {
        lastCheckTime = millis();
    if (!gpsEverReceived)
    {
      Serial.println(F("GPS not detected: No data received yet."));
    }
    else if (millis() - lastDataTime > 2000)
    {
      Serial.println(F("GPS disconnected or not sending data!"));
    }
    else
    {
      Serial.println(F("GPS connected: Data is being received."));
    }
  }
        // Delay for a while before the next reading
        vTaskDelay(pdMS_TO_TICKS(10000)); // Example: poll every 10s
    }
}

void taskAccelerometer(void *pvParameters) {
    QueueHandle_t displayQueue = (QueueHandle_t)pvParameters;
    // Initialize the ADXL sensor
    if (!adxl.begin()) {
        Serial.println("Failed to initialize ADXL!");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        // Read the ADXL data
        float x, y, z;
        adxl.readAcceleration(x, y, z);

        DisplayData_t msg;
        msg.type = SENSOR_ACCEL;
        msg.data.accel.x = x;
        msg.data.accel.y = y;
        msg.data.accel.z = z;
        xQueueSend(displayQueue, &msg, 0);

        // Delay for a while before the next reading
        vTaskDelay(pdMS_TO_TICKS(20)); // Example: poll every 20ms
    }
}