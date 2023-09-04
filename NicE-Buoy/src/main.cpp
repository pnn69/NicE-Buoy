/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
http://www.lilygo.cn/prod_view.aspx?TypeId=50003&Id=1130&FId=t3:50003:3
https://github.com/Xinyuan-LilyGO/TTGO-LoRa-Series
https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/T3_V1.6.1.pdf
433MHz is SX1278
*/
#include <Arduino.h>
#include "general.h"
#include "freertos/task.h"
#include "compass.h"
#include "adc.h"
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
#include "webinterface.h"
#include "../../dependency/command.h"
#include "io23017.h"

static unsigned long secstamp, sec05stamp, msecstamp, hstamp, sec5stamp;
// static double tglatitude = 52.29326976307006, tglongitude = 4.9328016467347435; // grasveld wsvop
// static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
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
switchStatus frontsw;
bool FrontLed = false;
char keypressed = 0;

MessageSP snd_sp;
MessageST snd_st;
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

/**********************************************************************************************************************************************************/
/* setup */
/**********************************************************************************************************************************************************/
void setup()
{
    Serial.begin(115200);
    initSSD1306();
    Wire.begin();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, false);
    InitMemory();
    initMCP23017();
    // Bootcnt(&bootCount, true);
    MemoryBuoyID(&buoyID, true);
    LastStatus(&status, &buoy.tglatitude, &buoy.tglongitude, true);
    lstatus = status;
    if (InitLora())
    {
        Serial.println("Lora Module OK!");
    }
    if (!InitCompass())
    {
        Serial.println("Compas OK!");
    }
    if (InitGps())
    {
        Serial.println("GPS OK!");
    }
    xTaskCreate(EscTask, "EscTask", 2400, NULL, 25, NULL);
    xTaskCreate(IndicatorTask, "IndicatorTask", 2400, NULL, 5, NULL);
    // websetup();
    Serial.printf("BuoyID = %d\n\r", buoyID);
    Serial.printf("Status = %d\n\r", status);
    secstamp = millis();
    msecstamp = millis();
    hstamp = millis();
    snd_st.ledstatus = CRGB(0, 0, 200);
    snd_st.ledstatusbb = CRGB(200, 0, 0);
    snd_st.ledstatussb = CRGB(0, 200, 0);
    if (xQueueSend(indicatorqueSt, (void *)&snd_st, 10) != pdTRUE)
    {
        Serial.println("Error sending speed to statusque");
    }
    for (int i = 0; i <= 100; i++)
    {
        snd_sp.speedbb = i;
        snd_sp.speedsb = i;
        xQueueSend(indicatorqueSp, (void *)&snd_sp, 10); // send to indicator
    }
    for (int i = 100; i >= -100; i--)
    {
        snd_sp.speedbb = i;
        snd_sp.speedsb = i;
        xQueueSend(indicatorqueSp, (void *)&snd_sp, 10); // send to indicator
    }
    for (int i = -100; i <= 0; i++)
    {
        snd_sp.speedbb = i;
        snd_sp.speedsb = i;
        xQueueSend(indicatorqueSp, (void *)&snd_sp, 10); // send to indicator
    }
    snd_st.ledstatus = CRGB(0, 0, 0);
    snd_st.ledstatusbb = CRGB(0, 0, 0);
    snd_st.ledstatussb = CRGB(0, 0, 0);
    if (xQueueSend(indicatorqueSt, (void *)&snd_st, 10) != pdTRUE)
    {
        Serial.println("Error sending speed to statusque");
    }
    delay(500);
    adc_switch(); // read out switches
    if (frontsw.switch1upact)
    {
        delay(500);
        adc_switch(); // read out switches
        if (frontsw.switch1upact)
        {
            CalibrateCompass(); //calibrate compass
        }
    }
    LastStatus(&status, &buoy.tglatitude, &buoy.tglongitude, true); // get last status and restore
    Serial.println("Setup done.");
}

/**********************************************************************************************************************************************************/
/* main loop */
/**********************************************************************************************************************************************************/

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
        if (status != lstatus) // store status in memory
        {
            LastStatus(&status, &buoy.tglatitude, &buoy.tglongitude, true);
        }
    }

    if (millis() - hstamp >= 100)
    {
        hstamp = millis();
        //buoy.mheading = CompassAverage(GetHeading());
        buoy.mheading = GetHeading();
        if (gpsdata.fix == false)
        {
            FrontLed = !FrontLed; // blink led
            MessageST snd_st;
            snd_st.ledstatus = CRGB(0, 0, 200 * FrontLed);
            if (xQueueSend(indicatorqueSt, (void *)&snd_st, 10) != pdTRUE)
            {
                Serial.println("Error sending speed to indicatorque");
            }

            //            digitalWrite(LEDSTRIP, FrontLed);
        }
    }

    // do stuff every 0.5 second
    if (millis() - sec05stamp >= 500)
    {
        sec05stamp = millis();

        if (gpsdata.fix == true)
        {
            if (status != LOCKED)
            {
                FrontLed = !FrontLed; // blink led
            }
            else if (status == LOCKED)
            {
                FrontLed = 1; // Led on
            }
            else
            {
                FrontLed = 0; // Led off
            }
            snd_st.ledstatus = CRGB(0, 0, 200 * FrontLed);
            if (xQueueSend(indicatorqueSt, (void *)&snd_st, 10) != pdTRUE)
            {
                Serial.println("Error sending speed to indicatorque");
            }

            // digitalWrite(LEDSTRIP1, FrontLed);
        }
        //         if (keypressed == 10)
        //         {
        //             if (gpsdata.fix == 1)
        //             {
        //                 int tmp;
        //                 RouteToPoint(gpsdata.dlat, gpsdata.dlon, buoy.tglatitude, buoy.tglongitude, &buoy.tgdistance, &buoy.tgdir); // calculate heading and
        //                 if (buoy.tgdir < 180 && buoy.mheading < 180)
        //                 {
        //                     tmp = (int)buoy.tgdir - (int)buoy.mheading;
        //                 }
        //                 else if (buoy.tgdir < 180 && buoy.mheading > 180)
        //                 {
        //                     tmp = 360 - (int)buoy.tgdir - (int)buoy.mheading;
        //                 }
        //                 else
        //                 {
        //                     tmp = (int)buoy.tgdir - (int)buoy.mheading;
        //                 }

        //                 buoy.mcorrdir = tmp;

        //                 Serial.printf(" dirgps %d dir mag%d =correction %ld\r\n", buoy.tgdir, buoy.mheading, (int)buoy.mcorrdir);
        //                 status = IDLE;
        //                 Message snd_msg;
        //                 snd_msg.speedbb = 10;
        //                 snd_msg.speedsb = 10;
        //                 xQueueSend(escspeed, (void *)&snd_msg, 10);
        //                 delay(10);
        //                 snd_msg.speedbb = 0;
        //                 snd_msg.speedsb = 0;
        //                 xQueueSend(escspeed, (void *)&snd_msg, 10);
        //             }
        //         }

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
            CalcEngingSpeed(buoy.cdir, 0, buoy.cspeed, &buoy.speedbb, &buoy.speedsb);
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
        xQueueSend(escspeed, (void *)&snd_msg, 10); // update esc
        snd_sp.speedbb = buoy.speedbb;
        snd_sp.speedsb = buoy.speedsb;
        xQueueSend(indicatorqueSp, (void *)&snd_sp, 10); // send to led indicator

        blink = !blink;
        if (blink == true && gpsdata.fix == true)
        {
            // snd_msg.ledstatus = CRGB::Blue;
        }
        else
        {
            // snd_msg.ledstatus = CRGB::Black;
        }
        if (status == LOCKED)
        {
            // snd_msg.ledstatus2 = CRGB::Red;
        }
        else if (status == IDLE)
        {
            // snd_msg.ledstatus2 = CRGB::Green;
        }
        else if (status == REMOTE)
        {
            // snd_msg.ledstatus2 = CRGB::Pink;
        }
        else if (status == DOC)
        {
            // snd_msg.ledstatus2 = CRGB::Orange;
        }

        /*
        Update dislpay
        */
        udateDisplay(buoy.speedsb, buoy.speedbb, buoy.tgdistance, buoy.tgdir, (unsigned long)buoy.mheading, gpsvalid);
    }

    // // do stuff every second
    if (millis() - secstamp >= 1000)
    {
        secstamp = millis();
        Serial.printf("Get heading\r\n");
        float mag = GetHeading();
        Serial.printf("heading %f\r\n",mag);
        adc_switch();

        //  loraMenu(GPS_LAT_LON_FIX_HEADING_SPEED);
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
