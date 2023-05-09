/*
    https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/DeviceExample/DeviceExample.ino
    https://github.com/plerup/espsoftwareserial/blob/main/examples/swsertest/swsertest.ino
*/
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define GpsRX 6
#define GpsTX 4

// A sample NMEA stream.
const char *gpsStream =
    "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
    "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
    "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
    "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
    "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
    "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

float lat, lng;

EspSoftwareSerial::UART usbSerial;

// The TinyGPSPlus object
TinyGPSPlus gps;

void displayInfo(void)
{
    if (gps.location.isValid())
    {
        // DECODE
        lat = gps.location.lat();
        Serial.print(gps.location.lat(), 6);
        lng = gps.location.lng();
        Serial.print(gps.location.lng(), 6);
    }
    else
    {
        Serial.print(F("INVALID GPS DATA"));
    }
}

int InitGps(void)
{

    return 0;
}

void GpsTask(void *arg)
{
    usbSerial.begin(4800, EspSoftwareSerial::SWSERIAL_8N1, GpsRX, GpsTX, false, 95);
    usbSerial.println(PSTR("\nSoftware serial test started"));
    while (1)
    {
        while (usbSerial.available() > 0)
        {
            if (gps.encode(usbSerial.read()))
                displayInfo();
        }
        delay(100);
    }
}