/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
http://www.lilygo.cn/prod_view.aspx?TypeId=50003&Id=1130&FId=t3:50003:3
https://github.com/Xinyuan-LilyGO/TTGO-LoRa-Series
https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/T3_V1.6.1.pdf
433MHz is SX1278
*/
#include <Arduino.h>
#include "freertos/task.h"
#include <math.h>
#include <Wire.h>
#include "io.h"
#include "oled_ssd1306.h"
#include "LiLlora.h"
#include "general.h"

unsigned long timestamp;
static float heading = 0;
static double gpslatitude = 52.34567, gpslongitude = 4.567;                     // home
static double tglatitude = 52.29326976307006, tglongitude = 4.9328016467347435; // grasveld wsvop
// static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
static unsigned long tgdir = 0, tgdistance = 0;
unsigned long previousTime = 0;
int speedbb = 0, speedsb = 0;
bool ledstatus = false;

buoyDataType buoy1;
buoyDataType buoy2;
buoyDataType buoy3;
// loraDataType loraIn;
// loraDataType loraOut;

double smallestAngle(double heading1, double heading2)
{
    double angle = fmod(heading2 - heading1 + 360, 360); // Calculate the difference and keep it within 360 degrees
    if (angle > 180)
    {
        angle = 360 - angle; // Take the smaller angle between the two
    }
    return angle;
}

int determineDirection(double heading1, double heading2)
{
    double angle = fmod(heading2 - heading1 + 360, 360); // Calculate the difference and keep it within 360 degrees
    if (angle > 180)
    {
        return 1; // Angle is greater than 180, SB (turn right)
    }
    else
    {
        return 0; // Angle is less than or equal to 180, BB (Turn left)
    }
}
/*
    adjust speed Cosinus does not work here. We have to do it manually
*/
double Angle2SpeedFactor(double angle)
{
    // cos(correctonAngle * M_PI / 180.0); not working for mall angles
    if (angle <= 90)
    {
        return (map(angle, 0, 90, 100, 0) / 100.0);
    }
    else
    {
        return (map(angle, 90, 180, 0, -100) / 100.0);
    }
}

/*
calculate the speed of both motors depeniding on the angle and distance to the target.
Send the result to the esc module
if heading < 180 BB motor 100% and SB motor less
if heading > 180 SB motor 100% and BB motor less
Speed is a function of distance start slowing donw if Buoy is less than 10 meter away for targed
The distance is in meters.
*/

void setup()
{
    Serial.begin(115200);
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, false);
    initSSD1306();
    Wire.begin();
    InitLora();
    Serial.println("Setup done.");
    delay(1000);
    timestamp = millis();
}

void loop()
{
    if (loraOK)
    {
        int msg = polLora();
        if (msg)
        {
            String decode = loraIn.message;
            char tmparr[100];
            Serial.printf("Messag recieved ");
            Serial.print("RSSI: " + String(loraIn.rssi) + "\r\n");
            switch (msg)
            {
            case (POSITION + 128):
                Serial.println("Position recieved on request!");
                decode.toCharArray(tmparr, decode.length() + 1);
                double lat1, lon1;
                sscanf(tmparr, "%lf,%lf", &lat1, &lon1);
                buoy1.gpslatitude = lat1;
                buoy1.gpslongitude = lon1;
                buoy1.rssi = loraIn.rssi;
                break;
            case (GET_DIR_DISTANSE_ANCHER_POSITION + 128):
                Serial.println("direction and distance target recieved on request!");
                decode.toCharArray(tmparr, decode.length() + 1);
                unsigned long dist, dir;
                sscanf(tmparr, "%d,%d", &dir, &dist);
                buoy1.tgdir = dir;
                buoy1.tgdistance = dist;
                buoy1.rssi = loraIn.rssi;
                break;
            default:
                Serial.println("unknown command");
                // just for git test

                break;
            }
            udateDisplay();
        }
    }
    if (millis() - previousTime > 3000)
    { // do stuff every second
        previousTime = millis();
        sendLora();
    }
    delay(1);
}