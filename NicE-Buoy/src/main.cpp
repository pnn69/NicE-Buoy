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
#include "io.h"
#include "oled_ssd1306.h"
#include "LiLlora.h"

unsigned long timestamp;
static float heading = 0;
static double gpslatitude = 52.34567, gpslongitude = 4.567;                     // home
static double tglatitude = 52.29326976307006, tglongitude = 4.9328016467347435; // grasveld wsvop
// static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
static unsigned long tgdir = 0, tgdistance = 0;
unsigned long previousTime = 0;
int speedbb = 0, speedsb = 0;
bool ledstatus = false;

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
void CalcEngingSpeed(float magheading, unsigned long tgheading, unsigned long tgdistance, int *bb, int *sb)
{
    Message snd_msg;
    int speed = 0.0;
    double correctonAngle = 0;
    if (tgdistance < 5)
    {
        speed = 0;
    }
    else
    {
        tgdistance = constrain(tgdistance, 0, 20);
        speed = map(tgdistance, 5, 20, 10, 100); // map speed 1-10 meter -> 10-100%
    }

    // Angle between calculated angel to steer and the current direction of the vessel
    correctonAngle = smallestAngle(magheading, tgheading);
    // Serial.printf("correctonAngle %.4f Angle2SpeedFactor %.4f \n",correctonAngle,Angle2SpeedFactor(correctonAngle));
    if (determineDirection(magheading, tgheading))
    {
        *bb = speed;
        *sb = int(speed * Angle2SpeedFactor(correctonAngle));
    }
    else
    {
        *bb = int(speed * Angle2SpeedFactor(correctonAngle));
        *sb = speed;
    }

    snd_msg.speedbb = *bb;
    snd_msg.speedsb = *sb;
    if (xQueueSend(escspeed, (void *)&snd_msg, 10) != pdTRUE)
    {
        Serial.println("Error sending speed to esc");
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, false);
    initSSD1306();
    Wire.begin();
    if (InitCompass())
    {
        Serial.println("Compass found!!");
    }
    else
    {
        Serial.println("Compass ERROR!!");
    }
    InitGps();
    InitLora();
    xTaskCreate(EscTask, "EscTask", 2400, NULL, 5, NULL);
    xTaskCreate(IndicatorTask, "IndicatorTask", 2400, NULL, 5, NULL);
    Serial.println("Setup done.");
    delay(1000);
    timestamp = millis();
}

void loop()
{
    if (loraOK)
    {
        polLora();
    }
    if (millis() % 100)
    { // do stuff every 100msec
        heading = CompassAverage(GetHeading());
        // heading = GetHeading();
        RouteToPoint(gpslatitude, gpslongitude, tglatitude, tglongitude, &tgdistance, &tgdir); // calculate heading and distance
        if (tgdistance < 5000)                                                                 // test if target is in range
        {
            CalcEngingSpeed(heading, tgdir, tgdistance, &speedbb, &speedsb);
        }
        else
        {
            CalcEngingSpeed(heading, heading, 0, &speedbb, &speedsb); // do notihing
        }
        GetNewGpsData(&gpslatitude, &gpslongitude);
    }
    if (millis() - previousTime > 1000)
    { // do stuff every second
        previousTime = millis();
        digitalWrite(led_pin, ledstatus);
        udateDisplay(speedsb, speedbb);
        ledstatus = !ledstatus;
        Serial.printf("\n\rlat:%3.5f long:%3.5f\n\r", gpslatitude, gpslongitude);
        Serial.printf("Compass heading: %3.0f Target heading: %3d \n\r", heading, tgdir);
        Serial.printf("Distance:%d direction:%d\n\r", tgdistance, tgdir);
        Serial.printf("Speed BB:%d Speed SB:%d\n\r", speedbb, speedsb);
        // displayGPSInfo();
        Serial.println();
    }
    delay(10);
}