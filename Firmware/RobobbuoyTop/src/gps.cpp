#include <TinyGPSPlus.h>
#include "gps.h"
#include "io_top.h"
#include "leds.h"

#define GPSBAUD 9600
// #define GPSBAUD 4800
// #define GPSBAUD 115200

static bool gpsok;
TinyGPSPlus gps;
GpsDataType gpsdata;
static LedData collorGps;
static uint32_t fix_age;
static unsigned long gpsTimeOut = 0;
static unsigned long blinktimer = millis();
static bool blink;

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

bool initgps(void)
{
    Serial.println(TinyGPSPlus::libraryVersion());
    Serial1.begin(GPSBAUD, SERIAL_8N1, GPSRX, GPSTX);
    Serial.println(PSTR("GPS port created"));
    return true;
}

void GpsTask(void *arg)
{
    gpsok = initgps();
    if (gpsok)
    {
        Serial.println("Gps task running!");
    }
    while (1)
    {
        if (blinktimer + 1000 < millis())
        {
            blinktimer = millis();
            blink = !blink;
        }

        while (Serial1.available() > 0)
        {
            // int t=Serial1.read();
            // Serial.print((char)t);
            // if (gps.encode(t))
            if (gps.encode(Serial1.read()))
            {
                gpsTimeOut = millis();
                if (gps.location.isUpdated() || gps.location.isValid())
                {
                    gpsdata.lat = gps.location.lat();
                    gpsdata.lon = gps.location.lng();
                }
                if (gps.speed.isValid())
                {
                    gpsdata.speed = (float)gps.speed.kmph();
                }
                if (gps.course.isValid())
                {
                    gpsdata.cource = (float)gps.course.deg();
                }
                gpsdata.nrsats = gps.satellites.value();
                fix_age = gps.location.age();
                if (fix_age < 1000)
                {
                    gpsdata.fix = true;
                    collorGps.color = CRGB::Green;
                }
                else
                {
                    gpsdata.fix = false;
                    if (blink)
                    {
                        collorGps.color = CRGB::Green;
                    }
                    else
                    {
                        collorGps.color = CRGB::Black;
                    }
                }
                xQueueSend(ledGps, (void *)&collorGps, 10);
            }
        }
        if (gpsTimeOut + 5000 < millis())
        {
            gpsTimeOut = millis();
            gpsdata.fix = false;
            collorGps.color = CRGB::Red;
            xQueueSend(ledGps, (void *)&collorGps, 10);
        }
        delay(1);
    }
}