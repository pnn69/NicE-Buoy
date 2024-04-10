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
#define ESC_TRIGGER 5 * 60 * 1000
#define BUTTON_SHORT 10
#define BUTTON_LONG 500

static unsigned long secstamp, sec05stamp, msecstamp, escstamp, hstamp, sec5stamp, offeststamp, esctrigger;
// static double tglatitude = 52.29326976307006, tglongitude = 4.9328016467347435; // grasveld wsvop
// static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop

// static unsigned long tgdir = 0, tgdistance = 0, cdir = 0;
bool nwloramsg = false;                         // Used to clear led status in main
double distanceChanged = 0;                     // chaged speed
char buoyID = 0;                                // buouy ID
byte status = IDLE;                             // Status buoy
const byte numChars = 32;                       //
char receivedChars[numChars];                   // an array to store the received data
boolean newData = false;                        //
int dataNumber = 0;                             // new for this version
static bool blink = false;                      // for blinking led
buoyDataType buoy;                              // buoy struckt
switchStatus frontsw;                           // led struckt
bool FrontLed = false;                          // status led
bool oneBeepLongpress = false;                  // on beep only indicator
int spdbb, spdsb;                               // speed
unsigned int sw_button_cnt, lst_button_cnt = 0; // button press counters

MessageSP snd_sp;    /* Speed sb(-100%<>100%),bb(-100%<>100%) */
MessageSq snd_sq;    /* Ledstatus CRGB */
MessageBuzz snd_buz; /* on(true/false),time(msec),pauze(msec),repeat(x) */

/*
not usede any more
onley for testing
*/
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
    color_printf(COLOR_PRINT_BLUE, "\r\nSetup running!\r\n");
    initSSD1306();
    Wire.begin();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, false);
    pinMode(BUZZERPIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(BUZZERPIN, BUZZEROFF);
    InitMemory();
    initMCP23017();
    InitGps();
    initCalculate();
    MemoryBuoyID(&buoyID, true);
    SWITCH_GRN_OFF;
    SWITCH_RED_ON;
    BUTTON_LIGHT_OFF;
    if (InitLora())
    {
        Serial.println("Lora Module OK!");
    }
    if (!InitCompass())
    {
        Serial.println("Compas OK!");
    }
    xTaskCreate(IndicatorTask, "IndicatorTask", 2400, NULL, 1, NULL);
    xTaskCreate(EscTask, "EscTask", 2400, NULL, 25, NULL);
    xTaskCreate(BuzzerTask, "BuzzerTask", 1024, NULL, 5, NULL);
    // websetup();
    Serial.printf("BuoyID = %d\n\r", buoyID);
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
    delay(400);
    BUTTON_LIGHT_ON;
    adc_switch(); // read out switches
    /*
     * Callibrate compass if key 1 is pressed or the key mountend on the fornt
     */
    if (frontsw.switch1upact || FRONTBUTTON_READ == PUSHED)
    {
        delay(100);
        adc_switch(); // read out switches
        if (frontsw.switch1upact || FRONTBUTTON_READ == PUSHED)
        {
            beepESC();
            CalibrateCompass(); // calibrate compass
            beepESC();
            offeststamp = millis();
            status = CALIBRATE_MAGNETIC_COMPASS;
        }
    }
    snd_sq.ledstatus = CRGB(0, 0, 0);
    xQueueSend(indicatorqueSt, (void *)&snd_sq, 10);
    xQueueSend(indicatorqueBb, (void *)&snd_sq, 10);
    xQueueSend(indicatorqueSb, (void *)&snd_sq, 10);
    snd_buz.repeat = 1;
    snd_buz.time = 100;
    snd_buz.pauze = 25;
    xQueueSend(Buzzerque, (void *)&snd_buz, 10);
    Serial.println("Setup done.");
    BUTTON_LIGHT_OFF;
    SWITCH_RED_ON;
    SWITCH_GRN_OFF;
    secstamp = millis();
    msecstamp = millis();
    hstamp = millis();
    // Postion Steiger WSOP
    // gpsdata.lat = 52.29308075283747;
    // gpsdata.lon = 4.932570409845357;
    // MemoryDockPos(&gpsdata.lat, &gpsdata.lon, false); // store default
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
        if (FRONTBUTTON_READ == PUSHED)
        {
            sw_button_cnt++;
            lst_button_cnt = sw_button_cnt;
            if (sw_button_cnt > BUTTON_LONG && oneBeepLongpress == false)
            {
                snd_buz.repeat = 5;
                snd_buz.time = 100;
                snd_buz.pauze = 25;
                xQueueSend(Buzzerque, (void *)&snd_buz, 10);
                beepESC();
                Serial.println("Button long pressed!!!");
                oneBeepLongpress = true;
            }
        }
        else
        {
            sw_button_cnt = 0;
            oneBeepLongpress = false;
            if (lst_button_cnt < BUTTON_SHORT)
            {
                lst_button_cnt = 0;
            }
        }
    } /*Done 10 msec loop*/

    if (millis() - hstamp > 100) /*100 msec loop*/
    {
        hstamp = millis() - 1;
        buoy.mheading = CompassAverage(GetHeading());
        GetNewGpsData();
    }

    // do stuff every 0.5 second
    if (millis() - sec05stamp > 500)
    {
        sec05stamp = millis() - 1;

        if (gpsdata.fix == true)
        {
            snd_sq.ledstatus = CRGB(0, 20, 0); // internal led GREEN color
            SWITCH_GRN_ON;                     // externel led GREEN (switch)
        }
        else
        {
            // no fix -> blink leds
            if (SWITCH_GRN_READ)
            {
                snd_sq.ledstatus = CRGB(20, 0, 0); // internal led RED color
                SWITCH_GRN_OFF;                    // external led off (switch)
            }
            else
            {
                snd_sq.ledstatus = CRGB(0, 20, 0); // internal led GREEN color
                SWITCH_GRN_ON;                     // externel led GREEN (switch)
            }
        }
        xQueueSend(indicatorqueSt, (void *)&snd_sq, 10); // update internal led

        /*
        key handeling
        */
        if (lst_button_cnt > BUTTON_SHORT && sw_button_cnt == 0)
        {
            buoy.speed = 0;
            buoy.speedbb = 0;
            buoy.speedsb = 0;
            if (lst_button_cnt > BUTTON_LONG) // Button long pressed?
            {
                if (status == CALIBRATE_OFFSET_MAGNETIC_COMPASS) // check if compass calibration mode is active
                {
                    if (gpsdata.fix == true) // GPS fix ok?
                    {
                        buoy.tglatitude = gpsdata.lat; // set current position as target postion.
                        buoy.tglongitude = gpsdata.lon;
                        snd_buz.time = 500;
                        snd_buz.repeat = 4;
                        snd_buz.pauze = 25;
                        xQueueSend(Buzzerque, (void *)&snd_buz, 10);
                        status = CALIBRATE_OFFSET_MAGNETIC_COMPASS;
                        offeststamp = millis(); // restart timer
                        Serial.printf("Status set to >CALIBRATE_OFFSET_MAGNETIC_COMPASS< = %d\r\n", status);
                        beepESC();
                    }
                }
                else if (gpsdata.fix == true) // Program doc position
                {
                    MemoryDockPos(&buoy.tglatitude, &buoy.tglongitude, false);
                    Serial.printf("Dock position stored!!!\r\n");
                    beepESC();
                    status = IDLE;
                }
            }
            else // short press on button
            {
                if (status != IDLE)
                {
                    status = IDLE;
                    BUTTON_LIGHT_OFF;
                    beepESC();
                    Serial.printf("Status set to >IDLE< = %d\r\n", status);
                }
                else
                {
                    if (gpsdata.fix == true)
                    {
                        buoy.tglatitude = gpsdata.lat;
                        buoy.tglongitude = gpsdata.lon;
                        BUTTON_LIGHT_ON;
                        beepESC();
                        if (status != CALIBRATE_MAGNETIC_COMPASS || status != CALIBRATE_OFFSET_MAGNETIC_COMPASS)
                        {
                            status = LOCKED;
                            Serial.printf("Status set to >LOCKED< = %d\r\n", status);
                        }
                    }
                }
            }
            lst_button_cnt = 0;
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
            buoy.tgdistance = 0;
            buoy.tgdir = 0;

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
                SWITCH_GRN_OFF;
            }
            else
            {
                SWITCH_RED_OFF;
                SWITCH_GRN_ON;
            }
            break;

        case DOCKED:
        case LOCKED:
            BUTTON_LIGHT_ON;
            SWITCH_RED_OFF;
            if (gpsdata.fix == true)
            {
                RouteToPoint(gpsdata.lat, gpsdata.lon, buoy.tglatitude, buoy.tglongitude, &buoy.tgdistance, &buoy.tgdir); // calculate heading and
                if (buoy.tgdistance < 2000)                                                                               // test if target is in range
                {
                    buoy.speed = CalcEngingSpeedBuoy(buoy.tgdir, buoy.mheading, buoy.tgdistance, &buoy.speedbb, &buoy.speedsb);
                }
                else
                {
                    buoy.speed = CalcEngingSpeedBuoy(buoy.tgdir, buoy.mheading, 0, &buoy.speedbb, &buoy.speedsb); // No speed out of range
                }
            }
            else
            {
                buoy.speedbb = 0; // no fix kill trusters
                buoy.speedsb = 0;
            }
            break;

            /*
            Callibration procedure:
            1 Press the push button during power on.
            2 Rotate buoy on all axes until you hear a beep.
            3 Place the buoy on a marked position.
            4 Wait until the green and red switch lights are burning continusly.
            5 Short press the button until you hear a beep.
            6 Place te buoy at least 30 meter from the marked position and point the front of the buoy to that marked point.
            7 Press the button again until a beep is sounding.
            Done!
            */
        case CALIBRATE_MAGNETIC_COMPASS:
            SWITCH_RED_ON;
            if (offeststamp + 100000 > millis())
            {
                status = IDLE;
            }
            if (BUTTON_LIGHT_READ)
            {
                BUTTON_LIGHT_OFF;
            }
            else
            {
                BUTTON_LIGHT_ON;
            }
            if (gpsdata.fix == true)
            {
                if (FRONTBUTTON_READ == PUSHED)
                {
                    delay(500);
                    {
                        if (FRONTBUTTON_READ == PUSHED)
                        {
                            buoy.tglatitude = gpsdata.lat;
                            buoy.tglongitude = gpsdata.lon;
                            status = CALIBRATE_OFFSET_MAGNETIC_COMPASS;
                            beepESC();
                            offeststamp = millis();
                        }
                    }
                }
            }
            break;

        case CALIBRATE_OFFSET_MAGNETIC_COMPASS: // Calibrate offset magnetic compass due mouning errors
            SWITCH_RED_ON;
            SWITCH_GRN_ON;
            if (BUTTON_LIGHT_READ)
            {
                BUTTON_LIGHT_OFF;
            }
            else
            {
                BUTTON_LIGHT_ON;
            }
            if (offeststamp + 10000 < millis())
            {
                if (FRONTBUTTON_READ == PUSHED)
                {
                    snd_buz.time = 200;
                    snd_buz.repeat = 0;
                    snd_buz.pauze = 0;
                    xQueueSend(Buzzerque, (void *)&snd_buz, 10);
                    BUTTON_LIGHT_ON;
                    delay(5000);
                    if (FRONTBUTTON_READ == PUSHED)
                    {
                        snd_buz.time = 100;
                        snd_buz.repeat = 25;
                        snd_buz.pauze = 25;
                        xQueueSend(Buzzerque, (void *)&snd_buz, 10);
                        RouteToPoint(gpsdata.lat, gpsdata.lon, buoy.tglatitude, buoy.tglongitude, &buoy.tgdistance, &buoy.tgdir); // calculate heading and
                        callibratCompassOfest((int)(buoy.tgdir - GetHeadingRaw()));                                               // store magnetic comensation
                        debugln("New offset stored");
                        delay(200);
                        GetHeadingRaw();
                        beepESC();
                        spdbb = 0;
                        spdsb = 0;
                    }
                    BUTTON_LIGHT_OFF;
                    SWITCH_RED_OFF;
                    SWITCH_GRN_ON;
                    status = IDLE;
                    Serial.printf("Status set to >IDLE< = %d\r\n", status);
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
        udateDisplay(buoy.speedsb, buoy.speedbb, (unsigned long)buoy.tgdistance, (unsigned int)buoy.tgdir, (unsigned int)buoy.mheading, gpsvalid);
    }

    if (distanceChanged != buoy.tgdistance)
    {
        if (status == LOCKED || status == DOCKED)
        {
            distanceChanged = buoy.tgdistance;
            loraMenu(DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_M_HEADING);
        }
    }
    /*
     do stuff every 5 sec
    */
    if (millis() - sec5stamp > 5000)
    {
        sec5stamp = millis() - 1;
        loraIn.recipient = 0xFE;
        loraMenu(GPS_LAT_LON_FIX_HEADING_SPEED_MHEADING); // pos heading speed to remote
        loraMenu(BATTERY_VOLTAGE_PERCENTAGE);             // bat voltage and percentage to remote
        if (status == LOCKED || status == DOCKED)
        {
            loraMenu(COMPUTE_PARAMETERS);
        }else{
            loraMenu(SBPWR_BBPWR);
        }
        // Serial.printf("Batt percentage %0.1f%% voltage: %0.2fV target distance %0.0lf target dir %0.0lf\r\n", buoy.vperc, buoy.vbatt, buoy.tgdistance, buoy.tgdir);
    }

    int nr;
    if (serialPortDataIn(&nr))
    {
        // Serial.printf("New data on serial port: %d\n", nr);
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
            esctrigger = millis();
            //            distanceChanged = true;
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
            //            distanceChanged = true;
            esctrigger = millis();
            if (spdsb < buoy.speedsb)
            {
                spdsb++;
            }
            else
            {
                spdsb--;
            }
        }
        if (millis() - esctrigger > ESC_TRIGGER)
        {
            if (buoy.speedsb == 0 && buoy.speedbb == 0)
            {
                triggerESC();
            }
            esctrigger = millis();
        }
        snd_msg.speedbb = spdbb;
        snd_msg.speedsb = spdsb;
        xQueueSend(escspeed, (void *)&snd_msg, 10); // update esc
        snd_sp.speedbb = spdbb;
        snd_sp.speedsb = spdsb;
        xQueueSend(indicatorqueSp, (void *)&snd_sp, 10); // send to led indicator
    }
}