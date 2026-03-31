#include <TinyGPSPlus.h>
#include "gps.h"
#include "io_top.h"
#include "RoboCompute.h"
// #include "leds.h"

// #define GPSBAUD 9600
//   #define GPSBAUD 4800
#define GPSBAUD 115200

TinyGPSPlus gps;
RoboStruct gpsdata;
QueueHandle_t gpsQue;
static uint32_t fix_age = 10000;
static unsigned long gpsTimeOut = 0;
static bool newGpsData = false;

/**
 * @brief Calculates the distance and bearing between two GPS coordinates.
 * 
 * @param lat1 Latitude of the starting point.
 * @param lon1 Longitude of the starting point.
 * @param lat2 Latitude of the destination point.
 * @param lon2 Longitude of the destination point.
 * @param distance Output pointer for the calculated distance in meters.
 * @param direction Output pointer for the calculated bearing in degrees.
 */
void RouteToPoint(double lat1, double lon1, double lat2, double lon2, double *distance, double *direction)
{
    *distance = distanceBetween(lat1, lon1, lat2, lon2);
    *direction = calculateBearing(lat1, lon1, lat2, lon2);
}

/**
 * @brief Initializes the GPS queue and serial communication.
 * 
 * @return true if initialization was successful.
 */
bool initgpsqueue(void)
{
    gpsQue = xQueueCreate(1, sizeof(RoboStruct));
    Serial2.begin(GPSBAUD, SERIAL_8N1, GPSRX, GPSTX);
    return true;
}

/**
 * @brief FreeRTOS task responsible for reading and parsing GPS data.
 * 
 * It continuously polls the secondary serial port for NMEA sentences,
 * uses TinyGPSPlus to parse them, and updates the global gpsQue with
 * the latest location, speed, heading, and fix status.
 * 
 * @param arg Task arguments (unused).
 */
void GpsTask(void *arg)
{
    Serial.println("Gps task running!");
    while (1)
    {
        bool updated = false;
        while (Serial2.available() > 0)
        {
            if (gps.encode(Serial2.read()))
            {
                updated = true;
            }
        }

        if (updated)
        {
            gpsTimeOut = millis();
            if (gps.location.isValid())
            {
                gpsdata.lat = gps.location.lat();
                gpsdata.lng = gps.location.lng();
            }
            
            if (gps.speed.isValid())
            {
                gpsdata.speed = (int)gps.speed.kmph();
            }
            if (gps.course.isValid())
            {
                gpsdata.gpsDir = (int)gps.course.deg();
            }
            
            gpsdata.gpsSat = (int)gps.satellites.value();
            gpsdata.gpsFixAge = gps.location.age();
            
            // A fix is valid if age is fresh and TinyGPS says it's valid
            gpsdata.gpsFix = (gps.location.isValid() && gpsdata.gpsFixAge < 2000);
            
            xQueueOverwrite(gpsQue, (void *)&gpsdata);
        }
        else if (millis() - gpsTimeOut > 5000)
        {
            if (gpsdata.gpsFix)
            {
                gpsdata.gpsFix = false;
                xQueueOverwrite(gpsQue, (void *)&gpsdata);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}