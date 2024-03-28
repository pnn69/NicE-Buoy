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
#include "buzzer.h"

#define ESC_UPDATE_DELAY 25

static unsigned long secstamp, sec05stamp, msecstamp, escstamp, hstamp, sec5stamp, offeststamp;
// static double tglatitude = 52.29326976307006, tglongitude = 4.9328016467347435; // grasveld wsvop
// static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
// static unsigned long tgdir = 0, tgdistance = 0, cdir = 0;
bool nwloramsg = false;
bool ledswgrn = false;
bool ledswred = false;
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
int spdbb, spdsb;

MessageSP snd_sp;    /* Speed sb(-100%<>100%),bb(-100%<>100%) */
MessageSq snd_sq;    /* Ledstatus CRGB */
MessageBuzz snd_buz; /* on(true/false),time(msec),pauze(msec),repeat(x) */

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
    pinMode(BUZZERPIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(BUZZERPIN, BUZZEROFF);
    InitMemory();
    initMCP23017();
    // Bootcnt(&bootCount, true);
    MemoryBuoyID(&buoyID, true);
    LastStatus(&status, &buoy.tglatitude, &buoy.tglongitude, true);
    SWITCH_GRN_OFF;
    SWITCH_RED_ON;
    lstatus = status;
    BUTTON_LIGHT_ON;
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
    BUTTON_LIGHT_OFF;
    xTaskCreate(IndicatorTask, "IndicatorTask", 2400, NULL, 1, NULL);
    xTaskCreate(EscTask, "EscTask", 2400, NULL, 25, NULL);
    xTaskCreate(BuzzerTask, "BuzzerTask", 1024, NULL, 5, NULL);
    // websetup();
    Serial.printf("BuoyID = %d\n\r", buoyID);
    Serial.printf("Status = %d\n\r", status);
    secstamp = millis();
    msecstamp = millis();
    hstamp = millis();
    snd_sq.ledstatus = CRGB(0, 0, 20);
    xQueueSend(indicatorqueSt, (void *)&snd_sq, 10);
    snd_sq.ledstatus = CRGB(20, 0, 0);
    xQueueSend(indicatorqueBb, (void *)&snd_sq, 10);
    snd_sq.ledstatus = CRGB(0, 20, 0);
    xQueueSend(indicatorqueSb, (void *)&snd_sq, 10);
    BUTTON_LIGHT_ON;
    for (int i = 0; i <= 100; i++)
    {
        snd_sp.speedbb = i;
        snd_sp.speedsb = i;
        xQueueSend(indicatorqueSp, (void *)&snd_sp, 10); // send to indicator
    }
    BUTTON_LIGHT_OFF;
    for (int i = 100; i >= -100; i--)
    {
        snd_sp.speedbb = i;
        snd_sp.speedsb = i;
        xQueueSend(indicatorqueSp, (void *)&snd_sp, 10); // send to indicator
    }
    BUTTON_LIGHT_ON;
    for (int i = -100; i <= 0; i++)
    {
        snd_sp.speedbb = i;
        snd_sp.speedsb = i;
        xQueueSend(indicatorqueSp, (void *)&snd_sp, 10); // send to indicator
    }
    snd_sq.ledstatus = CRGB(0, 0, 0);
    if (xQueueSend(indicatorqueSt, (void *)&snd_sq, 10) != pdTRUE)
    {
        Serial.println("Error sending speed to statusque");
    }
    BUTTON_LIGHT_OFF;
    delay(500);
    BUTTON_LIGHT_ON;
    adc_switch(); // read out switches
    /*
     * Callibrate compass if key 1 is pressed or the key mountend on the fornt
     */
    if (frontsw.switch1upact || mcp.digitalRead(SWITCH1_GPA) == 0)
    {
        delay(500);
        adc_switch(); // read out switches
        if (frontsw.switch1upact || mcp.digitalRead(SWITCH1_GPA) == 0)
        {
            CalibrateCompass(); // calibrate compass
        }
    }
    LastStatus(&status, &buoy.tglatitude, &buoy.tglongitude, true); // get last status and restore
    snd_sq.ledstatus = CRGB(0, 0, 0);
    xQueueSend(indicatorqueSt, (void *)&snd_sq, 10);
    xQueueSend(indicatorqueBb, (void *)&snd_sq, 10);
    xQueueSend(indicatorqueSb, (void *)&snd_sq, 10);
    snd_buz.repeat = 3;
    snd_buz.time = 100;
    snd_buz.pauze = 25;
    xQueueSend(Buzzerque, (void *)&snd_buz, 10);
    Serial.println("Setup done.");
    delay(1000);
    BUTTON_LIGHT_OFF;
    SWITCH_RED_OFF;
}

/**********************************************************************************************************************************************************/
/* main loop */
/**********************************************************************************************************************************************************/

void loop()
{
    webloop();  /*update websocket*/
    if (loraOK) /*check incomming lora messages*/
    {
        polLora();
    }
    if (millis() - msecstamp > 10) /*10 msec loop*/
    {
        msecstamp = millis() - 1;
        digitalWrite(LED_PIN, !nwloramsg); /* If green led is turned on by incomming lora message. turn it off */
        nwloramsg = false;                 /*clear ledstatus*/
        adc_switch();                      /*read switch status*/
        if (status != lstatus)             // store status in memory
        {
            //     LastStatus(&status, &buoy.tglatitude, &buoy.tglongitude, true);
            lstatus = status;
            Serial.printf("Status updated in memory\r\n");
        }
    } /*Done 10 msec loop*/

    if (millis() - hstamp > 100) /*100 msec loop*/
    {
        hstamp = millis() - 1;
        GetNewGpsData();
        buoy.mheading = CompassAverage(GetHeading());
        // Serial.printf( "heading %f\r\n",buoy.mheading);
        if (gpsdata.fix == false)
        {
            FrontLed = !FrontLed; // blink led
            snd_sq.ledstatus = CRGB(200 * FrontLed, 0, 0);
            xQueueSend(indicatorqueSt, (void *)&snd_sq, 10);
        }
    }

    // do stuff every 0.5 second
    if (millis() - sec05stamp > 500)
    {
        sec05stamp = millis() - 1;

        if (gpsdata.fix == true)
        {
            snd_sq.ledstatus = CRGB(0, 10, 0);
            SWITCH_GRN_ON;
            ledswgrn = true;
        }
        else
        {
            snd_sq.ledstatus = CRGB(10, 0, 0);
            if (ledswgrn)
            {
                SWITCH_GRN_OFF;
                ledswgrn = false;
            }
            else
            {
                SWITCH_GRN_ON;
                ledswgrn = true;
            }
        }
        xQueueSend(indicatorqueSt, (void *)&snd_sq, 10); // update light status
        if (gpsdata.fix == true)
        {

            if (status == LOCKED)
            {
                FrontLed = 1; // Led on
            }
            else
            {
                FrontLed = 0; // Led off
            }
            snd_sq.ledstatus = CRGB(0, 200 * FrontLed, 0);
            xQueueSend(indicatorqueBb, (void *)&snd_sq, 10);
        }
        if (FRONTBUTTON_READ == 0 && status != CALIBRATE_OFFSET_MAGNETIC_COMPASS) // check if locked switch is set to active
        {
            snd_buz.time = 200;
            snd_buz.repeat = 0;
            snd_buz.pauze = 0;
            xQueueSend(Buzzerque, (void *)&snd_buz, 10);
            frontsw.switch1upcnt = 2;
            snd_buz.time = BUZZTIME;
            Serial.printf("Front button pressed\r\n");
            if (status == LOCKED)
            {
                status = IDLE;
                BUTTON_LIGHT_OFF;
            }
            else
            {
                if (gpsdata.fix == true)
                {
                    buoy.tglatitude = gpsdata.lat;
                    buoy.tglongitude = gpsdata.lon;
                    status = LOCKED;
                    BUTTON_LIGHT_ON;
                }
            }
            Message snd_msg;
            snd_msg.speedbb = 0;
            snd_msg.speedsb = 0;
            xQueueSend(escspeed, (void *)&snd_msg, 10);
            delay(1000);
            if (FRONTBUTTON_READ == 0 && status == IDLE && gpsdata.fix == true) // Do some checks before going in to calibration mode.
            {
                snd_buz.time = 500;
                snd_buz.repeat = 4;
                snd_buz.pauze = 25;
                xQueueSend(Buzzerque, (void *)&snd_buz, 10);
                status = CALIBRATE_OFFSET_MAGNETIC_COMPASS;
                offeststamp = millis();
            }
            Serial.printf("Status set to = %d\r\n", status);
        }

        /*
        Do stuff depending on the status of the buoy
        */
        switch (status)
        {
        case UNLOCK:
            status = IDLE;
            snd_sq.ledstatus = CRGB(0, 200, 0);
            xQueueSend(indicatorqueBb, (void *)&snd_sq, 10);
        case NO_POSITION:
        case IDLE:
            buoy.speed = 0;
            buoy.speedbb = 0;
            buoy.speedsb = 0;
            BUTTON_LIGHT_OFF;
            SWITCH_RED_OFF;
            break;
        case REMOTE:
            CalcEngingSpeed(buoy.cdir, 0, buoy.cspeed, &buoy.speedbb, &buoy.speedsb);
            BUTTON_LIGHT_OFF;
            if (mcp.digitalRead(MAINSSWITCH_LEDRED_GPB) == 0)
            {
                SWITCH_RED_ON;
            }
            else
            {
                SWITCH_RED_OFF;
            }
            break;
        case LOCKED:
            RouteToPoint(gpsdata.dlat, gpsdata.dlon, buoy.tglatitude, buoy.tglongitude, &buoy.tgdistance, &buoy.tgdir); // calculate heading and
            if (buoy.tgdistance < 5000)                                                                                 // test if target is in range
            {
                buoy.speed = CalcEngingSpeedBuoy(buoy.tgdir, buoy.mheading, buoy.tgdistance, &buoy.speedbb, &buoy.speedsb);
            }
            else
            {
                buoy.speed = CalcEngingSpeedBuoy(buoy.tgdir, buoy.mheading, 0, &buoy.speedbb, &buoy.speedsb); // do notihing
            }
            BUTTON_LIGHT_ON;
            SWITCH_RED_OFF;
            break;
        case CALIBRATE_OFFSET_MAGNETIC_COMPASS: // Calibrate offset magnetic compass due mouning errors
            SWITCH_RED_ON;
            SWITCH_GRN_OFF;
            buoy.speed = 0;
            buoy.speedbb = 0;
            buoy.speedsb = 0;
            if (BUTTON_LIGHT_READ)
            {
                BUTTON_LIGHT_OFF;
            }
            else
            {
                BUTTON_LIGHT_ON;
            }
            if (offeststamp + 5000 < millis())
            {
                if (FRONTBUTTON_READ == 0)
                {
                    snd_buz.time = 200;
                    snd_buz.repeat = 0;
                    snd_buz.pauze = 0;
                    xQueueSend(Buzzerque, (void *)&snd_buz, 10);
                    BUTTON_LIGHT_ON;
                    delay(5000);
                    if (FRONTBUTTON_READ == 0)
                    {
                        snd_buz.time = 100;
                        snd_buz.repeat = 25;
                        snd_buz.pauze = 25;
                        xQueueSend(Buzzerque, (void *)&snd_buz, 10);
                        RouteToPoint(gpsdata.dlat, gpsdata.dlon, buoy.tglatitude, buoy.tglongitude, &buoy.tgdistance, &buoy.tgdir); // calculate heading and
                        // callibratCompassOfest((int)(buoy.tgdir - GetHeadingRaw()));                                                 // store magnetic comensation
                        debugln("New offset stored");
                        delay(200);
                        GetHeadingRaw();
                    }
                    BUTTON_LIGHT_OFF;
                    SWITCH_RED_OFF;
                    SWITCH_GRN_ON;
                    status = IDLE;
                }
            }
            break;
        default:
            buoy.speed = 0;
            buoy.speedbb = 0;
            buoy.speedsb = 0;
            BUTTON_LIGHT_OFF;
        }

        blink = !blink;

        /*
        Update dislpay
        */
        udateDisplay(buoy.speedsb, buoy.speedbb, buoy.tgdistance, buoy.tgdir, (unsigned long)buoy.mheading, gpsvalid);
    }

    // // do stuff every second
    if (millis() - secstamp > 1000)
    {
        secstamp = millis() - 1;
        // loraMenu(GPS_LAT_LON_FIX_HEADING_SPEED);
    }
    /*
     do stuff every 5 sec
    */
    if (millis() - sec5stamp > 5000)
    {
        sec5stamp = millis() - 1;
        // Serial.printf("bootCount:%d\r\n", bootCount);
        loraMenu(GPS_LAT_LON_FIX_HEADING_SPEED_MHEADING);
        loraMenu(BATTERY_VOLTAGE_PERCENTAGE);
        Serial.printf("Batt percentage %0.1f%% voltage: %0.2f\r\n", buoy.vperc, buoy.vbatt);
    }

    int nr;
    if (serialPortDataIn(&nr))
    {
        Serial.printf("New data on serial port: %d\n", nr);
    }

    /*
     * Sending speed to ESC and indicator
     */
    if (millis() - escstamp > ESC_UPDATE_DELAY) /*ESC update loop*/
    {
        escstamp = millis() - 1; // to make it correct. > used instead of >=
        Message snd_msg;
        if (spdbb != buoy.speedbb)
        {
            if (spdbb < buoy.speedbb) // slowly go to setpoint
            {
                spdbb++;
            }
            else
            {
                spdbb--;
            }
        }
        if (spdsb != buoy.speedsb)
        {
            if (spdsb < buoy.speedsb)
            {
                spdsb++;
            }
            else
            {
                spdsb--;
            }
        }
        snd_msg.speedbb = spdbb;
        snd_msg.speedsb = spdsb;
        xQueueSend(escspeed, (void *)&snd_msg, 10); // update esc
        snd_sp.speedbb = spdbb;
        snd_sp.speedsb = spdsb;
        xQueueSend(indicatorqueSp, (void *)&snd_sp, 10); // send to led indicator
    }
}