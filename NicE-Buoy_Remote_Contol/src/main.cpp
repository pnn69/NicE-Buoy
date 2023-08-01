/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
https://www.lilygo.cc/products/lora3
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
#include "webinterface.h"
#include "adc.h"
#include "gps.h"
#include "../../dependency/command.h"

unsigned long timestamp, msecstamp, hsecstamp, sec5stamp;
static float heading = 0;
static double gpslatitude = 52.34567, gpslongitude = 4.567;                     // home
static double tglatitude = 52.29326976307006, tglongitude = 4.9328016467347435; // grasveld wsvop
// static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
static unsigned long tgdir = 0, tgdistance = 0;
unsigned long previousTime = 0;
int speedbb = 0, speedsb = 0;
bool ledstatus = false;
static bool ackOK[4];
static bool switch_IDLE = false;
static bool switch_REMOTE = false;

const byte numChars = 5;
char receivedChars[numChars]; // an array to store the received data

buoyDataType buoy[NR_BUOYS];

// bool serialPortDataIn(int *nr)
// {
//     static byte ndx = 0;
//     char endMarker = '\n';
//     char rc;

//     if (Serial.available() > 0)
//     {
//         rc = Serial.read();
//         if (rc != endMarker)
//         {
//             receivedChars[ndx] = rc;
//             ndx++;
//             if (ndx >= numChars)
//             {
//                 ndx = numChars - 1;
//             }
//         }
//         else
//         {
//             receivedChars[ndx] = '\0'; // terminate the string
//             ndx = 0;
//             *nr = atoi(receivedChars); // new for this version
//             return true;
//         }
//     }
//     return false;
// }

void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(SWITCH_PIN_REMOTE_LOCK, INPUT_PULLUP);
    digitalWrite(LED_PIN, false);
    initSSD1306();
    Wire.begin();
    InitLora();
    websetup();
    readAdc();
    InitGps();
    adc.newdata = false;
    Serial.println("Setup done.");
    delay(1000);

    buoy[1].status = IDLE;
    buoy[2].status = IDLE;
    buoy[3].status = IDLE;
    buoy[1].ackOK = true;
    buoy[2].ackOK = true;
    buoy[3].ackOK = true;

    timestamp = millis();
    msecstamp = millis();
    hsecstamp = millis();
    sec5stamp = millis();
}

void loop()
{
    webloop();

    if (polLora())
    {
        udateDisplay();
        notify = loraIn.sender;
    }

    /*
    runs each 10 msec
    */
    if (millis() - msecstamp >= 10)
    {
        msecstamp = millis();
        digitalWrite(LED_PIN, ledstatus);
        ledstatus = false;
    }

    /*
    runs each 5 msec
    */
    if (millis() - sec5stamp >= 5000)
    {
        sec5stamp = millis();
    }

    /*
     runs each 100 msec
     read adc values and send new setting if has been updated by user
     */
    if (millis() - hsecstamp >= 100)
    {
        hsecstamp = millis();
        // if(SWITCH_REMOTE == 0)
        // {
        //     buoy[1].status = REMOTE;
        //     buoy[1].cmnd = SAIL_DIR_SPEED;
        // }
        // if(SWITCH_LOCK == 0)       {
        //     buoy[1].status = LOCKING;
        //     buoy[1].cmnd = TARGET_POSITION;
        // }
        if (digitalRead(SWITCH_PIN_REMOTE_LOCK) == 1)
        {
            if (switch_REMOTE != 0)
            {
                switch_REMOTE = 0;
                buoy[1].cdir = 0;
                buoy[1].cspeed = 0;
                buoy[1].speed = 0;
                buoy[1].speedbb = 0;
                buoy[1].speedsb = 0;
                buoy[1].status = REMOTE;
                buoy[1].cmnd = SAIL_DIR_SPEED;
                buoy[1].ackOK = false;
                buoy[1].gsa = SET;
                radiobutton[1] = 6;
                notify = true;
            }
        }
        else
        {
            if (switch_REMOTE != 1)
            {
                switch_REMOTE = 1;
                buoy[1].cdir = 0;
                buoy[1].cspeed = 0;
                buoy[1].speed = 0;
                buoy[1].speedbb = 0;
                buoy[1].speedsb = 0;
                buoy[1].status = LOCKING;
                buoy[1].cmnd = TARGET_POSITION;
                buoy[1].ackOK = false;
                buoy[1].gsa = SET;
                radiobutton[1] = 2;
                notify = true;
            }
        }
        readAdc();
        // for (int i = 1; i < NR_BUOYS; i++)
        // {
        int i = 1;
        if (buoy[i].status == REMOTE)
        {
            if (adc.newdata == true)
            {
                // Serial.printf("New data from ADC! speed:%d rudder:%d\r\n", adc.speed, adc.rudder);
                buoy[i].cspeed = adc.speed;
                buoy[i].cdir = adc.rudder;
                buoy[i].cmnd = SAIL_DIR_SPEED;
                buoy[i].gsa = SET;
                adc.newdata = false;
                while (loraMenu(i))
                    ;
            }
        }
        //        }
    }

    if (millis() - previousTime >= 1000)
    { // do stuff every second
        previousTime = millis();
        for (int i = 1; i < NR_BUOYS; i++)
        {
            if (buoy[i].status == LOCKING)
            {
                buoy[i].cmnd = TARGET_POSITION;
                buoy[i].gsa = SET;
                buoy[i].ackOK = true;
                while (loraMenu(i))
                    ;
            }else if(buoy[i].status == LOCKED){
                buoy[i].cmnd = DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION;
                buoy[i].gsa = GET;
                buoy[i].ackOK = false;
                while (loraMenu(i))
                    ;
            }
        }
        GetNewGpsData();
        if (gpsdata.fix == true)
        {
            buoy[0].cmnd = DGPS;
            while (loraMenu(0))
                ;
        }
        // Serial.printf("Batt voltage %0.2f\r\n", adc.vbat);

        /*
            repeat last command again.
        */
        for (int i = 1; i < NR_BUOYS; i++)
        {
            if (buoy[i].ackOK == false)
            {
                loraMenu(i);
            }
        }
    }

    
    // int nr;
    // if (serialPortDataIn(&nr))
    // {
    //     Serial.printf("New data on serial port: %d\n", nr);
    // }

    delay(1);
}