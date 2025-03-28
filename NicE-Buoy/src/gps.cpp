/*
    https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/DeviceExample/DeviceExample.ino
    https://github.com/plerup/espsoftwareserial/blob/main/examples/swsertest/swsertest.ino
*/
#include <TinyGPSPlus.h>
#include "gps.h"
#include "io.h"

//#define GPSBAUD 9600
#define GPSBAUD 115200

bool gpsvalid = false;
bool gpsactive = true;
unsigned long gpsTimeOut = 0;
TinyGPSPlus gps;
GpsDataType gpsdata;

void InitGps(void)
{
    Serial1.begin(GPSBAUD, SERIAL_8N1, GPSRX, GPSTX);
    Serial.println(PSTR("GPS port created"));
}

static void printFloat(float val, bool valid, int len, int prec)
{
    if (!valid)
    {
        while (len-- > 1)
            Serial.print('*');
        Serial.print(' ');
    }
    else
    {
        Serial.print(val, prec);
        int vi = abs((int)val);
        int flen = prec + (val < 0.0 ? 2 : 1); // . and -
        flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                             : vi >= 10    ? 2
                                           : 1;
        for (int i = flen; i < len; ++i)
            Serial.print(' ');
    }
}

static void printInt(unsigned long val, bool valid, int len)
{
    char sz[32] = "*****************";
    if (valid)
        sprintf(sz, "%ld", val);
    sz[len] = 0;
    for (int i = strlen(sz); i < len; ++i)
        sz[i] = ' ';
    if (len > 0)
        sz[len - 1] = ' ';
    Serial.print(sz);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
    if (!d.isValid())
    {
        Serial.print(F("********** "));
    }
    else
    {
        char sz[32];
        sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
        Serial.print(sz);
    }

    if (!t.isValid())
    {
        Serial.print(F("******** "));
    }
    else
    {
        char sz[32];
        sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
        Serial.print(sz);
    }

    printInt(d.age(), d.isValid(), 5);
}

static void printStr(const char *str, int len)
{
    int slen = strlen(str);
    for (int i = 0; i < len; ++i)
        Serial.print(i < slen ? str[i] : ' ');
}

/*
    Display data trough the RS232 interface
*/
void displayGPSInfo(void)
{
    if (gps.location.isValid())
    {
        // DECODE
        // lat = gps.location.lat();
        // Serial.print(gps.location.lat(), 6);
        // lng = gps.location.lng();
        // Serial.print(gps.location.lng(), 6);
        // speed = gps.speed.kmph();
        // Serial.print(gps.speed.kmph(), 6);
        // dir = gps.course.deg();
        // Serial.print(gps.course.deg(), 3);

        printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
        printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
        printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
        printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
        printInt(gps.location.age(), gps.location.isValid(), 5);
        printDateTime(gps.date, gps.time);
        printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
        printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
        printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
        printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);
    }
    else
    {
        Serial.print(F("INVALID GPS DATA"));
    }
}
/*
    Collect GPS data if new data is gatherd
*/
bool GetNewGpsData()
{
    if (gpsactive == false) // disabled for dummy data added external
    {
        // while (Serial1.available() > 0); //flush data
        return (false);
    }
    while (Serial1.available() > 0)
    {
        if (gps.encode(Serial1.read()))
        {
            if (gps.location.isUpdated())
                if (gps.location.isValid())
                {
                    {
                        gpsTimeOut = millis();
                        gpsdata.lat = gps.location.lat();
                        gpsdata.lon = gps.location.lng();
                        if (gps.speed.isValid())
                        {
                            gpsdata.speed = gps.speed.kmph();
                        }
                        if (gps.course.isValid())
                        {
                            gpsdata.cource = gps.course.deg();
                        }
                        gpsdata.fix = true;
                        gpsdata.nrsats = (unsigned int)gps.satellites.value();
                        gpsvalid = true;
                        return true;
                    }
                }
        }
    }
    if (gpsTimeOut + 5000 < millis())
    {
        gpsdata.fix = false;
        gpsvalid = false;
    }
    return false;
}
/*
    Calulate heading and distance given two sets of coordinates.
    Return pointers distance and direction.
    unsigend double heading in degrees 0-259
    unsigend double distance in meters
*/
void RouteToPoint(double lat1, double lon1, double lat2, double lon2, double *distance, double *direction)
{
    *distance = gps.distanceBetween(lat1, lon1, lat2, lon2);
    *direction = gps.courseTo(lat1, lon1, lat2, lon2);
}
