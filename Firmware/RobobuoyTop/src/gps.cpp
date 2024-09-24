#include <TinyGPSPlus.h>
#include "gps.h"
#include "io_top.h"
#include "leds.h"

// #define GPSBAUD 9600
//   #define GPSBAUD 4800
#define GPSBAUD 115200

static bool gpsok;
TinyGPSPlus gps;
GpsDataType gpsdata;
QueueHandle_t gpsQue;
static LedData collorGps;
static uint32_t fix_age = 10000;
static unsigned long gpsTimeOut = 0;
static bool newGpsData = false;

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
    Serial1.begin(GPSBAUD, SERIAL_8N1, GPSRX, GPSTX);
    Serial.println("GPS port created");
    gpsok = true;
    return true;
}
bool initgpsqueue(void)
{
    gpsQue = xQueueCreate(1, sizeof(gpsdata));
    return true;
}

void GpsTask(void *arg)
{

    gpsok = initgps();
    if (gpsok)
    {
        collorGps.color = CRGB::Red;
        collorGps.blink = BLINK_OFF;
        xQueueSend(ledGps, (void *)&collorGps, 10);
        gpsdata.fix = false;
        Serial.println("Gps task running!");
    }
    while (1)
    {
        while (Serial1.available() > 0)
        {
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
                    gpsdata.speed = (float)gps.speed.kmph();
                }
                if (gps.course.isValid() && gpsdata.cource != (float)gps.course.deg())
                {
                    gpsdata.cource = (float)gps.course.deg();
                    newGpsData = true;
                }
                if (gpsdata.nrsats != gps.satellites.value())
                {
                    gpsdata.nrsats = gps.satellites.value();
                    if (gpsdata.fix == false)
                    {
                        newGpsData = true;
                    }
                }
                fix_age = gps.location.age();
                if (fix_age < 1000)
                {
                    if (gpsdata.fix != true)
                    {
                        gpsdata.fix = true;
                        collorGps.color = CRGB::Green;
                        collorGps.blink = BLINK_OFF;
                        xQueueSend(ledGps, (void *)&collorGps, 10);
                    }
                }
                else
                {
                    if (gpsdata.fix != false || collorGps.color != CRGB::Green || collorGps.blink != BLINK_SLOW)
                    {
                        gpsdata.fix = false;
                        newGpsData = true;
                        collorGps.color = CRGB::Green;
                        collorGps.blink = BLINK_SLOW;
                        xQueueSend(ledGps, (void *)&collorGps, 10);
                    }
                }
            }
        }
        if (gpsTimeOut + 5000 < millis())
        {
            gpsTimeOut = millis();
            gpsdata.fix = false;
            newGpsData = true;
            collorGps.color = CRGB::Red;
            collorGps.blink = BLINK_OFF;
            xQueueSend(ledGps, (void *)&collorGps, 10);
        }
        if (newGpsData == true)
        {
            newGpsData = false;
            // collorGps.color = CRGB::Green;
            // xQueueSend(ledGps, (void *)&collorGps, 10);
            // xQueueSend(gpsQue, (void *)&gpsdata, 10); // update util led
            // collorGps.color = CRGB::Black;
            // xQueueSend(ledGps, (void *)&collorGps, 10);
        }
        delay(1);
    }
}