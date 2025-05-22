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

void RouteToPoint(double lat1, double lon1, double lat2, double lon2, double *distance, double *direction)
{
    *distance = gps.distanceBetween(lat1, lon1, lat2, lon2);
    *direction = gps.courseTo(lat1, lon1, lat2, lon2);
}

bool initgpsqueue(void)
{
    gpsQue = xQueueCreate(1, sizeof(gpsdata));
    return true;
}

void GpsTask(void *arg)
{
    Serial2.begin(GPSBAUD, SERIAL_8N1, GPSRX, GPSTX);
    Serial.println("Gps task running!");
    while (1)
    {
        while (Serial2.available() > 0)
        {
            char c = Serial2.read();
            // Serial.print(c);
            if (gps.encode(c))
            {
                gpsTimeOut = millis();
                if (gps.location.isUpdated() && gps.location.isValid())
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
                if (gpsdata.gpsFixAge < 1000)
                {
                    newGpsData = true;
                    gpsdata.gpsFix = true;
                }
                else
                {
                    if (gpsdata.gpsFix == true)
                    {
                        newGpsData = true;
                        gpsdata.gpsFix = false;
                    }
                }
            }
        }
        if (gpsTimeOut + 5000 < millis())
        {
            gpsdata.gpsFix = false;
        }
        if (newGpsData == true) // only updat if position has been changed
        {
            xQueueSend(gpsQue, (void *)&gpsdata, 10);
            newGpsData = false;
        }
        delay(1);
    }
}