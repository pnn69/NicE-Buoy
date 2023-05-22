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
#include "datastorage.h"
#include "calculate.h"

unsigned long secstamp, msecstamp;
static float heading = 0;
static double gpslatitude = 52.34567, gpslongitude = 4.567;                     // home
static double tglatitude = 52.29326976307006, tglongitude = 4.9328016467347435; // grasveld wsvop
// static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
static unsigned long tgdir = 0, tgdistance = 0;
int speedbb = 0, speedsb = 0;
bool ledstatus = false;
char buoyID = 0;

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, false);
    InitMemory();
    BuoyID(0, &buoyID);
    InitLora();
    InitCompass();
    InitGps();
    initSSD1306();
    xTaskCreate(EscTask, "EscTask", 2400, NULL, 5, NULL);
    xTaskCreate(IndicatorTask, "IndicatorTask", 2400, NULL, 5, NULL);
    Serial.printf("BuoyID = %d\n\r", buoyID);
    Serial.println("Setup done.");
    secstamp = millis();
    msecstamp = millis();
}

void loop()
{
    if (loraOK)
    {
        int msg = polLora();
        if (msg)
        {
            Serial.printf("Messag recieved ");
            Serial.print("RSSI:" + String(loraIn.rssi) +" msg:" +msg + "\r\n");
            switch (msg)
            {
            case POSITION:
                sendLoraPos(1, gpslatitude, gpslongitude);
                break;
            case GET_DIR_DISTANSE_ANCHER_POSITION:
                sendLoraDirHeadingAncher(1, tgdir, tgdistance);
                break;
            default:
                Serial.println("unknown command");
                break;
            }
        }
    }
    if (millis() - secstamp > 100)
    { // do stuff every 100 milisecond
        msecstamp = millis();
        heading = CompassAverage(GetHeading());
        RouteToPoint(gpslatitude, gpslongitude, tglatitude, tglongitude, &tgdistance, &tgdir); // calculate heading and distance
        if (tgdistance < 5000)                                                                 // test if target is in range
        {
            CalcEngingSpeed(heading, tgdir, tgdistance, &speedbb, &speedsb);
        }
        else
        {
            CalcEngingSpeed(heading, heading, 0, &speedbb, &speedsb); // do notihing
        }
        Message snd_msg;
        snd_msg.speedbb = speedbb;
        snd_msg.speedsb = speedsb;
        xQueueSend(escspeed, (void *)&snd_msg, 10);
        GetNewGpsData(&gpslatitude, &gpslongitude);
    }

    if (millis() - secstamp > 500)
    { // do stuff every second
        secstamp = millis();
        digitalWrite(led_pin, ledstatus);
        ledstatus = !ledstatus;
        udateDisplay(speedsb, speedbb, tgdistance, tgdir);
    }
    delay(1);
}