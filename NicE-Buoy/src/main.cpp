/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
http://www.lilygo.cn/prod_view.aspx?TypeId=50003&Id=1130&FId=t3:50003:3
https://github.com/Xinyuan-LilyGO/TTGO-LoRa-Series
https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/T3_V1.6.1.pdf
433MHz is SX1278
*/
#include <Arduino.h>
#include "freertos/task.h"
#include "compass.h"
#include "gps.h"
#include "esc.h"
#include "indicator.h"
#include <math.h>
#include <Wire.h>

unsigned long timestamp;
static float heading = 0;
static double gpslatitude = 52.32097917684152, gpslongitude = 4.965395389421265; // home
static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357;   // steiger wsvop
static unsigned long tgdir = 0, tgdistance = 0;

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
        return map(angle, 0, 90, 1, 0);
    }
    else
    {
        return map(angle, 90, 180, 0, -1);
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
void CalcEngingSpeed(float magheading, unsigned long tgheading, unsigned long tgdistance)
{
    Message snd_msg;
    int speed = 0.0;
    double correctonAngle = 0;
    if (tgdistance < 1)
    {
        speed = 0;
    }
    else
    {
        speed = map(tgdistance, 1, 10, 10, 100); // map speed 1-10 meter -> 10-100%
    }

    // Angle between calculated angel to steer and the current direction of the vessel
    correctonAngle = smallestAngle(magheading, tgheading);
    if (determineDirection(magheading, tgheading))
    {
        snd_msg.speedbb = speed;
        snd_msg.speedsb = int(speed * Angle2SpeedFactor(correctonAngle));
    }
    else
    {
        snd_msg.speedbb = int(speed * Angle2SpeedFactor(correctonAngle));
        snd_msg.speedsb = speed;
    }
    if (xQueueSend(escspeed, (void *)&snd_msg, 10) != pdTRUE)
    {
        Serial.println("Error sending speed to esc");
    }
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    if (InitCompass())
    {
        Serial.println("Compass found!!");
    }else{
        Serial.println("Compass ERROR!!");
    }
    InitGps();
    // xTaskCreate(EscTask, "EscTask", 100, NULL, 1, NULL);
    // xTaskCreate(IndicatorTask, "IndicatorTask", 100, NULL, 1, NULL);
    Serial.println("Setup done.");
    delay(1000);
    timestamp = millis();
}
void loop()
{
    if (millis() % 100)
    { // do stuff every 100msec
        heading = CompassAverage(GetHeading());
        //heading = GetHeading();
        RouteToPoint(gpslatitude, gpslongitude, tglatitude, tglongitude, &tgdistance, &tgdir); // calculate heading and distance
        if (tgdistance < 5000)                                                                // test if target is in range
        {
            //CalcEngingSpeed(heading, tgdir, tgdistance);
        }
        else
        {
            //CalcEngingSpeed(heading, heading, 0); //do notihing
        }
    }
    if (millis() % 1000)
    { // do stuff every second
        Serial.printf("Compass heading : %3.2f\n\r", heading);
        if (GetNewGpsData(&gpslatitude, &gpslongitude))
        {
            Serial.printf(" lat:%3.5f long:%3.5f\n\r", gpslatitude, gpslongitude);
            Serial.printf(" Distance:%d direction:%d\n\r", tgdistance, tgdir);
        }
        else
        {
            Serial.println("No new gps data!");
        }
    }
    delay(1000);
}