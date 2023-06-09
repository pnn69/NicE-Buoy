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

unsigned long secstamp, msecstamp, updatestamp;
// static double tglatitude = 52.29326976307006, tglongitude = 4.9328016467347435; // grasveld wsvop
//  static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
// static unsigned long tgdir = 0, tgdistance = 0, cdir = 0;
bool ledstatus = false;
char buoyID = 0;
byte status = IDLE; // IDLE
const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;
int dataNumber = 0; // new for this version

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
    GetMemoryBuoyID(&buoyID);
    InitLora();
    InitCompass();
    InitGps();
    initSSD1306();
    xTaskCreate(EscTask, "EscTask", 2400, NULL, 25, NULL);
    xTaskCreate(IndicatorTask, "IndicatorTask", 2400, NULL, 5, NULL);
    // websetup();
    Serial.printf("BuoyID = %d\n\r", buoyID);
    Serial.printf("Status = %d\n\r", status);
    secstamp = millis();
    msecstamp = millis();
    status = IDLE; // IDLE
    buoy.tglatitude = 52.29326976307006;
    buoy.tglongitude = 4.9328016467347435; // grasveld wsvop
    // static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
    // SetMemoryDockPos(52.29308075283747, 4.932570409845357);
    LMessage snd_msg;
    snd_msg.ledstatus[0] = CRGB::Red;
    snd_msg.ledstatus[1] = CRGB::Blue;
    if (xQueueSend(indicatorque, (void *)&snd_msg, 10) != pdTRUE)
    {
        Serial.println("Error sending speed to indicatorque");
    }
    Serial.println("Setup done.");
}

void loop()
{
    // webloop();
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
        buoy.mheading = CompassAverage(GetHeading());
    }

    // do stuff every 0.5 second
    if (millis() - secstamp >= 500)
    {
        secstamp = millis();
        if (GetNewGpsData(&buoy.gpslatitude, &buoy.gpslongitude))
        {
            // GpsAverage(&gpslatitude, &gpslongitude);
        }
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
            CalcEngingSpeed(buoy.cdir, (unsigned long)buoy.mheading, buoy.cspeed, &buoy.speedbb, &buoy.speedsb);
            break;
        case LOCKED:
            RouteToPoint(buoy.gpslatitude, buoy.gpslongitude, buoy.tglatitude, buoy.tglongitude, &buoy.tgdistance, &buoy.tgdir); // calculate heading and
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
        if (gpsvalid)
        {
            snd_msg.ledstatus[0] = CRGB::Green;
            snd_msg.ledstatus[1] = CRGB::Blue;
        }
        else
        {
            snd_msg.ledstatus[0] = CRGB::Red;
            snd_msg.ledstatus[1] = CRGB::Blue;
        }
        xQueueSend(escspeed, (void *)&snd_msg, 10);
        // LMessage snd_st;
        // if(gpsvalid){
        //     snd_st.stbb = CRGB:: Green;
        // }else{
        //     snd_st.stbb = CRGB:: Red;
        // }
        // xQueueSend(indicatorqueSt, (void *)&snd_st, 10);
        // digitalWrite(LED_PIN, ledstatus);
        /*
        Update dislpay
        */
        udateDisplay(buoy.speedsb, buoy.speedbb, buoy.tgdistance, buoy.tgdir, (unsigned long)buoy.mheading, gpsvalid);
    }

    int nr;
    if (serialPortDataIn(&nr))
    {
        Serial.printf("New data on serial port: %d\n", nr);
    }

    delay(1);
}