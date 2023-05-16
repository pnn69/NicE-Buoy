/*
    https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/DeviceExample/DeviceExample.ino
    https://github.com/plerup/espsoftwareserial/blob/main/examples/swsertest/swsertest.ino
*/
#include <TinyGPSPlus.h>
#include "io.h"

#define GPSBAUD 9600

double lat, lng, speed, dir;

TinyGPSPlus gps;
/*
    Display data trough the RS232 interface
*/
void displayGPSInfo(void)
{
    if (gps.location.isValid())
    {
        // DECODE
        lat = gps.location.lat();
        Serial.print(gps.location.lat(), 6);
        lng = gps.location.lng();
        Serial.print(gps.location.lng(), 6);
        speed = gps.speed.kmph();
        Serial.print(gps.speed.kmph(), 6);
        dir = gps.course.deg();
        Serial.print(gps.course.deg(), 3);
    }
    else
    {
        Serial.print(F("INVALID GPS DATA"));
    }
}
/*
    Collect GPS data if new data is gatherd
*/
int GetNewGpsData(double *gpslat, double *gpslng)
{
    char in;
    //while (gpsSerial.available() > 0)
    while (Serial1.available() > 0)
    {
        //in = Serial1.read();
        //Serial.print(in);
        //if (gps.encode(in))
        if (gps.encode(Serial1.read()))
            if (gps.location.isValid())
            {
                *gpslat = gps.location.lat();
                *gpslng = gps.location.lng();
                return 1;
            }
    }
    return 0;
}
/*
    Calulate heading and distance given two sets of coordinates.
    Retun pointers distance and direction.
    unsigend long heading in degrees 0-259
    unsigend long distance in meters
*/
void RouteToPoint(double lat1, double lon1, double lat2, double lon2, unsigned long *distance, unsigned long *direction)
{
    *distance = gps.distanceBetween(lat1, lon1, lat2, lon2);
    *direction = gps.courseTo(lat1, lon1, lat2, lon2);
}

int InitGps(void)
{
    Serial1.begin(GPSBAUD,SERIAL_8N1, GpsRX, GpsTX);
    Serial.println(PSTR("\nGPS port created"));
    return 0;
}

void GpsTask(void *arg)
{
    InitGps();
    double la, lo;
    while (1)
    {
        GetNewGpsData(&la, &lo);
        delay(1000);
    }
}