#include <TinyGPSPlus.h>
#include "gps.h"
#include "io_top.h"
#include "leds.h"

// #define GPSBAUD 9600
//   #define GPSBAUD 4800
#define GPSBAUD 115200

TinyGPSPlus gps;
GpsDataType gpsdata;
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
    Serial1.begin(GPSBAUD, SERIAL_8N1, GPSRX, GPSTX);
    Serial.println("Gps task running!");
    while (1)
    {
        while (Serial1.available() > 0)
        {
            //char c = Serial1.read();
            //Serial.print(c);
            //if (gps.encode(c))
            if (gps.encode(Serial1.read()))
            {
                gpsTimeOut = millis();
                if (gps.location.isUpdated() && gps.location.isValid())
                {
                    if (gpsdata.lat != gps.location.lat())
                    {
                        gpsdata.lat = gps.location.lat();
                        newGpsData = true;
                    }
                    if (gpsdata.lon != gps.location.lng())
                    {
                        gpsdata.lon = gps.location.lng();
                        newGpsData = true;
                    }
                }
                if (gps.speed.isValid())
                {
                    gpsdata.speed = gps.speed.kmph();
                }
                if (gps.course.isValid() && gpsdata.cource != gps.course.deg())
                {
                    gpsdata.cource = gps.course.deg();
                }
                if (gpsdata.nrsats != gps.satellites.value())
                {
                    gpsdata.nrsats = gps.satellites.value();
                }
                fix_age = gps.location.age();
                if (fix_age < 1000)
                {
                    if (gpsdata.fix == false)
                    {
                        newGpsData = true;
                        gpsdata.fix = true;
                    }
                }
                else
                {
                    if (gpsdata.fix == true)
                    {
                        newGpsData = true;
                        gpsdata.fix = false;
                    }
                }
            }
        }
        if (gpsTimeOut + 5000 < millis())
        {
            gpsdata.fix = false;
        }
        if (newGpsData == true) // only updat if position has been changed
        {
            newGpsData = false;
            xQueueSend(gpsQue, (void *)&gpsdata, 10);
        }
        delay(1);
    }
}