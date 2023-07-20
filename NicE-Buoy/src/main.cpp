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
#include "general.h"
#include "webinterface.h"
#include "../../dependency/command.h"

static unsigned long secstamp, sec05stamp, msecstamp, updatestamp, hstamp, sec5stamp;
// static double tglatitude = 52.29326976307006, tglongitude = 4.9328016467347435; // grasveld wsvop
static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
// static unsigned long tgdir = 0, tgdistance = 0, cdir = 0;
bool ledstatus = false;
char buoyID = 0;
byte status = IDLE, lstatus = 0;
const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;
int dataNumber = 0; // new for this version
static bool blink = false;
int bootCount = 0;
buoyDataType buoy;

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

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, false);
    InitMemory();
    // Bootcnt(&bootCount, true);
    GetMemoryBuoyID(&buoyID);
    LastStatus(&status, false, &buoy.tglatitude, &buoy.tglongitude);
    lstatus = status;
    if (InitLora())
    {
        Serial.println("Lora Module OK!");
    }
    if (InitCompass())
    {
        Serial.println("Compas OK!");
    }
    if (InitGps())
    {
        Serial.println("GPS OK!");
    }
    initSSD1306();
    xTaskCreate(EscTask, "EscTask", 2400, NULL, 25, NULL);
    xTaskCreate(IndicatorTask, "IndicatorTask", 2400, NULL, 5, NULL);
    // websetup();
    Serial.printf("BuoyID = %d\n\r", buoyID);
    Serial.printf("Status = %d\n\r", status);
    secstamp = millis();
    msecstamp = millis();
    hstamp = millis();
    LMessage snd_msg;
    snd_msg.ledstatus1 = CRGB::Green;
    snd_msg.ledstatus2 = CRGB::Red;
    // for (int t = 0; t < 11; t++)
    // {
    //     snd_msg.speedbb = t * 10;
    //     snd_msg.speedsb = t * 10;
    //     if (xQueueSend(indicatorque, (void *)&snd_msg, 10) != pdTRUE)
    //     {
    //         Serial.println("Error sending speed to indicatorque");
    //     }
    //     delay(500);
    // }
    if (xQueueSend(indicatorque, (void *)&snd_msg, 10) != pdTRUE)
    {
        Serial.println("Error sending speed to indicatorque");
    }
    //     delay(500);
    // }
    LastStatus(&status, false, &buoy.tglatitude, &buoy.tglongitude); // get last status and restore
    Serial.println("Setup done.");
}

void loop()
{
    webloop();
    if (loraOK)
    {
        polLora();
    }
    /*
     do stuff every 100 milisecond
    */
    if (millis() - msecstamp >= 10)
    {
        msecstamp = millis();
        digitalWrite(LED_PIN, ledstatus);
        ledstatus = false;
        if (status != lstatus)
        {
            LastStatus(&status, true, &buoy.tglatitude, &buoy.tglongitude);
            lstatus = status;
        }
    }

    if (millis() - hstamp >= 100)
    {
        hstamp = millis();
        buoy.mheading = CompassAverage(GetHeading());
        // buoy.mheading =GetHeading();
    }

    // do stuff every 0.5 second
    if (millis() - sec05stamp >= 500)
    {
        sec05stamp = millis();
        GetNewGpsData();
        /*
        Do stuff depending on the status of the buoy
        */
        switch (status)
        {
        case UNLOCK:
            status = IDLE;
        case IDLE:
            buoy.speed = 0;
            buoy.speedbb = 0;
            buoy.speedsb = 0;
            break;
        case REMOTE:
            CalcEngingSpeed(buoy.cdir, buoy.mheading, buoy.cspeed, &buoy.speedbb, &buoy.speedsb);
            break;
        case LOCKED:
            RouteToPoint(gpsdata.dlat, gpsdata.dlon, buoy.tglatitude, buoy.tglongitude, &buoy.tgdistance, &buoy.tgdir); // calculate heading and
            // Serial.print("dist:");
            // Serial.print(buoy.tgdistance);
            // Serial.print(" dir:");
            // Serial.println(buoy.tgdir);
            if (buoy.tgdistance < 5000) // test if target is in range
            {
                buoy.speed = CalcEngingSpeedBuoy(buoy.tgdir, buoy.mheading, buoy.tgdistance, &buoy.speedbb, &buoy.speedsb);
            }
            else
            {
                buoy.speed = CalcEngingSpeedBuoy(buoy.tgdir, buoy.mheading, 0, &buoy.speedbb, &buoy.speedsb); // do notihing
            }
            break;
        default:
            buoy.speed = 0;
            buoy.speedbb = 0;
            buoy.speedsb = 0;
        }
        /*
        Sending data to ESC
        */
        Message snd_msg;
        snd_msg.speedbb = buoy.speedbb;
        snd_msg.speedsb = buoy.speedsb;
        blink = !blink;
        if (blink == true & gpsdata.fix == true)
        {
            snd_msg.ledstatus1 = CRGB::Blue;
        }
        else
        {
            snd_msg.ledstatus1 = CRGB::Black;
        }
        if (status == LOCKED)
        {
            snd_msg.ledstatus2 = CRGB::Red;
        }
        else if (status == IDLE)
        {
            snd_msg.ledstatus2 = CRGB::Green;
        }
        else if (status == REMOTE)
        {
            snd_msg.ledstatus2 = CRGB::Pink;
        }
        else if (status == DOC)
        {
            snd_msg.ledstatus2 = CRGB::Orange;
        }

        xQueueSend(escspeed, (void *)&snd_msg, 10);

        /*
        Update dislpay
        */
        udateDisplay(buoy.speedsb, buoy.speedbb, buoy.tgdistance, buoy.tgdir, (unsigned long)buoy.mheading, gpsvalid);
    }

    // do stuff every second
    if (millis() - secstamp >= 1000)
    {
        secstamp = millis();
        // loraMenu(GPS_LAT_LON_FIX_HEADING_SPEED);
    }
    /*
     do stuff every 5 sec
    */
    if (millis() - sec5stamp >= 5000)
    {
        sec5stamp = millis();
        // Serial.printf("bootCount:%d\r\n", bootCount);
        loraMenu(GPS_LAT_LON_FIX_HEADING_SPEED_MHEADING);
    }

    int nr;
    if (serialPortDataIn(&nr))
    {
        Serial.printf("New data on serial port: %d\n", nr);
    }

    delay(1);
}