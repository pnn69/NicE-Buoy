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
#include "../../dependency/command.h"

unsigned long timestamp;
static float heading = 0;
static double gpslatitude = 52.34567, gpslongitude = 4.567;                     // home
static double tglatitude = 52.29326976307006, tglongitude = 4.9328016467347435; // grasveld wsvop
// static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
static unsigned long tgdir = 0, tgdistance = 0;
unsigned long previousTime = 0;
int speedbb = 0, speedsb = 0;
bool ledstatus = false;

const byte numChars = 5;
char receivedChars[numChars]; // an array to store the received data

buoyDataType buoy[4];

bool serialPortDataIn(int *nr)
{
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;

    if (Serial.available() > 0)
    {
        rc = Serial.read();
        if (rc != endMarker)
        {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars)
            {
                ndx = numChars - 1;
            }
        }
        else
        {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            *nr = atoi(receivedChars); // new for this version
            return true;
        }
    }
    return false;
}

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
            unsigned long dist, dir;
            int sb, bb;
            Serial.printf("Messag recieved ");
            Serial.print("Sender:" + String(loraIn.sender) + " RSSI:" + String(loraIn.rssi) + " msg:" + msg + "\r\n");
            int id = loraIn.sender;
            switch (msg)
            {
            case (POSITION):
                Serial.println("Position recieved!");
                decode.toCharArray(tmparr, decode.length() + 1);
                double lat1, lon1;
                sscanf(tmparr, "%lf,%lf", &lat1, &lon1);
                buoy[loraIn.sender].gpslatitude = lat1;
                buoy[loraIn.sender].gpslongitude = lon1;
                buoy[loraIn.sender].rssi = loraIn.rssi;
                break;
            case (DIR_DISTANSE_TO_TARGET_POSITION):
                Serial.println("direction and distance target recieved!");
                decode.toCharArray(tmparr, decode.length() + 1);
                sscanf(tmparr, "%d,%d", &dir, &dist);
                buoy[loraIn.sender].tgdir = dir;
                buoy[loraIn.sender].tgdistance = dist;
                buoy[loraIn.sender].rssi = loraIn.rssi;
                break;
            case (DIR_DISTANSE_SBSPPEED_BBSPEED_TARGET_POSITION):
                Serial.println("direction and distance target recieved!");
                decode.toCharArray(tmparr, decode.length() + 1);
                sscanf(tmparr, "%d,%d,%d,%d", &dir, &dist, &sb, &bb);
                buoy[loraIn.sender].tgdir = dir;
                buoy[loraIn.sender].tgdistance = dist;
                buoy[loraIn.sender].speedsb = sb;
                buoy[loraIn.sender].speedbb = bb;
                buoy[loraIn.sender].rssi = loraIn.rssi;
                break;

            case ANCHOR_POSITION:
                Serial.println("New anchor position recieved!");
                Serial.println(String(loraIn.message));
                buoy[loraIn.sender].rssi = loraIn.rssi;
                break;
            default:
                Serial.println("unknown command: " + msg);
                // just for git test

                break;
            }
            udateDisplay();
        }
    }
    if (millis() - previousTime > 500)
    { // do stuff every second
        previousTime = millis();
        digitalWrite(led_pin, ledstatus);
        ledstatus = !ledstatus;
    }
    int nr;
    if (serialPortDataIn(&nr))
    {
        Serial.printf("New data on serial port: %d\n", nr);
        if(nr == 18){
            sendLoraSetTargetPosition();
        }
        if(nr == RESET){
            sendLoraReset();
        }

    }

    delay(1);
}