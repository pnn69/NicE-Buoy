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

#define ESC_UPDATE_DELAY 10
#define ESC_TRIGGER 5 * 60 * 1000
#define BUTTON_SHORT 1
#define BUTTON_LONG 25
#define BUTTON_SUPER_LONG BUTTON_LONG * 4
static unsigned long secstamp, sec05stamp, msecstamp, gpsstamp, hstamp, sec5stamp, offeststamp, esctrigger;
// static double tglatitude = 52.29326976307006, tglongitude = 4.9328016467347435; // grasveld wsvop
// static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop

// static unsigned long tgdir = 0, tgdistance = 0, cdir = 0;
bool nwloramsg = false;         // Used to clear led status in main
double distanceChanged = 0;     // chaged distance
int speedChanged = 0;           // chaged speed
char buoyID = 0;                // buouy ID
byte status = IDLE;             // Status buoy
const byte numChars = 32;       //
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;        //
int dataNumber = 0;             // new for this version
static bool blink = false;      // for blinking led
buoyDataType buoy;              // buoy struckt
switchStatus frontsw;           // led struckt
bool FrontLed = false;          // status led
bool gpsLed = false;            // status led
bool copLed = false;            // status led
int spdbb = 0, spdsb = 0;       // speed
unsigned int sw_button_cnt = 0; // button press counters
unsigned int sw_button_set = 0; // button press counters
unsigned int msg_cnt = 0;       // msg out counter
MessageSP snd_sp;               /* Speed sb(-100%<>100%),bb(-100%<>100%) */
MessageSq snd_sq;               /* Ledstatus CRGB */
MessageBuzz snd_buz;            /* on(true/false),time(msec),pauze(msec),repeat(x) */
Message snd_msg;                /* ESC que struckt */
String msg_main = "";
char bufff_main[100];
Mlora loramsg;
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
    // initSSD1306();
    Wire.begin();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, false);
    pinMode(BUZZERPIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(BUZZERPIN, BUZZEROFF);
    InitMemory();
    initMCP23017();
    InitGps();
    InitCompass();
    /*only init memory once*/
    // rudderpid.kp = 1;
    // rudderpid.ki = 0.1;
    // rudderpid.kd = 0;
    // speedpid.kp = 20;
    // speedpid.ki = 4;
    // speedpid.kd = 0;
    // pidRudderParameters(&rudderpid.kp, &rudderpid.ki, &rudderpid.kd, false);
    // pidSpeedParameters(&speedpid.kp, &speedpid.ki, &speedpid.kd, false);
    // float max_mag[3] = { 576,466,754};
    // float min_mag[3] = {-535,-645,-382};
    // CompassCallibrationFactorsFloat(&max_mag[0], &max_mag[1], &max_mag[2], &min_mag[0], &min_mag[1], &min_mag[2], false); //  get callibration data
    // gpsdata.lat = 52.29308075283747;
    // gpsdata.lon = 4.932570409845357;
    // MemoryDockPos(&gpsdata.lat, &gpsdata.lon, false); // store default wsvop
    /*end init memory once*/
    initCalculate();
    initRudderPid();
    initSpeedPid();
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
    // adc_switch(); // read out switches
    xTaskCreate(IndicatorTask, "IndicatorTask", 2400, NULL, 1, NULL);
    xTaskCreate(EscTask, "EscTask", 2400, NULL, 25, NULL);
    xTaskCreate(BuzzerTask, "BuzzerTask", 1024, NULL, 5, NULL);
    xTaskCreate(LoraTask, "LoraTask", 4024, NULL, 10, NULL);
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
    BUTTON_LIGHT_ON;
    // adc_switch(); // read out switches
    /*
     * Callibrate compass if key 1 is pressed or the key mountend on the fornt
     */
    if (FRONTBUTTON_READ == PUSHED)
    {
        delay(100);
        if (FRONTBUTTON_READ == PUSHED)
        {
            status = CALIBRATE_MAGNETIC_COMPASS;
            beepESC();
            CalibrateCompass(); // calibrate compass
            beepESC();
            offeststamp = millis();
            status = IDLE;
        }
    }
    snd_sq.ledstatus = CRGB(0, 0, 0);
    xQueueSend(indicatorqueSt, (void *)&snd_sq, 10);
    xQueueSend(indicatorqueBb, (void *)&snd_sq, 10);
    xQueueSend(indicatorqueSb, (void *)&snd_sq, 10);
    snd_buz.repeat = 1;
    snd_buz.time = 100;
    snd_buz.pauze = 25;
    // xQueueSend(Buzzerque, (void *)&snd_buz, 10);
    BUTTON_LIGHT_OFF;
    SWITCH_RED_ON;
    SWITCH_GRN_OFF;
    secstamp = millis();
    msecstamp = millis();
    hstamp = millis();
    Serial.println("Paramters buoy!\r\n***");
    MemoryDockPos(&buoy.tglatitude, &buoy.tglongitude, true);
    Serial.printf("Doc positon: https://www.google.nl/maps/@%2.12lf,%2.12lf\r\n", buoy.tglatitude, buoy.tglongitude);
    CompassOffsetCorrection(&buoy.magneticCorrection, true);
    Serial.printf("Magnetic correction:%d\r\n", buoy.magneticCorrection);
    computeParameters(&buoy.minOfsetDist, &buoy.maxOfsetDist, &buoy.minSpeed, &buoy.maxSpeed, true);
    Serial.printf("Speed PID:  kp=%2.2lf ki=%2.2lf kd=%2.2lf\r\n", speedpid.kp, speedpid.ki, speedpid.kd);
    Serial.printf("Rudder PID: kp=%2.2lf ki=%2.2lf kd=%2.2lf\r\n", rudderpid.kp, rudderpid.ki, rudderpid.kd);
    Serial.printf("minOfsetDist=%d, maxOfsetDist=%d, minSpeed=%d, maxSpeed=%d\r\n", buoy.minOfsetDist, buoy.maxOfsetDist, buoy.minSpeed, buoy.maxSpeed);
    buoy.muteEsc = false; // enable esc
    Serial.println("***");
#ifdef WEBON
    websetup();
#endif
    status = IDLE;
    Serial.println("Setup done.");
    char bufff[30];
    strcpy(bufff, "Setup done!");
    udpsend(bufff);

    /**********************************************************************************************************************************************************/
    /* Only for testing */
    /**********************************************************************************************************************************************************/
    // rudderpid.kp = 0.2;
    // rudderpid.ki = 0.02;
    // rudderpid.kd = 0.1;
    // pidRudderParameters(&rudderpid.kp, &rudderpid.ki, &rudderpid.kd, false);
    /*
        Postion Steiger WSOP as target
    */

    // gpsdata.lat = 52.29308075283747;
    // gpsdata.lon = 4.932570409845357;
    // MemoryDockPos(&gpsdata.lat, &gpsdata.lon, false); // store default wsvop
    // mute esc
    // buoy.muteEsc = true;

    /*
        home posoition coordinates
        fake fix
        fake nr satalits
        disable gps
        set status to loking
    */
#if DEBUG
    gpsdata.lat = 52.32038;
    gpsdata.lon = 4.96563;
    gpsdata.fix = true;
    gpsdata.nrsats = 10;
    gpsactive = false; // disable gps
    status = LOCKED;
    buoy.magneticCorrection = 0;
#endif
}

/**********************************************************************************************************************************************************/
/* main loop */
/**********************************************************************************************************************************************************/

void loop()
{
    webloop();
    buoy.mheading = CompassAverage(GetHeading());
    // buoy.mheading = GetHeading();
    if (copLed == true)
    {
        copLed = false;
        snd_sq.ledstatus = CRGB(0, 0, 20);
        xQueueSend(indicatorqueBb, (void *)&snd_sq, 10);
    }
    else
    {
        copLed = true;
        snd_sq.ledstatus = CRGB(0, 0, 0);
        xQueueSend(indicatorqueBb, (void *)&snd_sq, 10);
    }

    if (status == LOCKED || status == DOCKED)
    {
        CalcRudderBuoy(buoy.tgdir, buoy.mheading, buoy.tgdistance, buoy.speed, &buoy.speedbb, &buoy.speedsb); // calculate power to thrusters
    }
    if (GetNewGpsData() == true)
    {
        if (status == LOCKED || status == DOCKED)
        {
            RouteToPoint(gpsdata.lat, gpsdata.lon, buoy.tglatitude, buoy.tglongitude, &buoy.tgdistance, &buoy.tgdir);
            buoy.speed = hooverPid(buoy.tgdistance);
            // CalcRudderBuoy(buoy.tgdir, buoy.mheading, buoy.tgdistance, buoy.speed, &buoy.speedbb, &buoy.speedsb); // calculate power to thrusters
        }
        if (gpsLed == true)
        {
            gpsLed = false;
            snd_sq.ledstatus = CRGB(0, 0, 20);
            xQueueSend(indicatorqueSb, (void *)&snd_sq, 10);
        }
        else
        {
            gpsLed = true;
            snd_sq.ledstatus = CRGB(0, 0, 0);
            xQueueSend(indicatorqueSb, (void *)&snd_sq, 10);
        }
    }

    /* If green led is turned on by incomming lora message. turn it off */
    if (millis() - msecstamp > 10) /*10 msec loop*/
    {
        msecstamp = millis() - 1;
        digitalWrite(LED_PIN, !nwloramsg);
        nwloramsg = false;
    } /*Done 10 msec loop*/

    if (millis() - hstamp > 100) /*100 msec loop*/
    {
        hstamp = millis() - 1;

        // String msg = "<245><" + String(status) + "><" + GPS_LAT_LON_NRSAT_FIX_HEADING_SPEED_MHEADING + "><" + SET + "><" + String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8) + "," + String(gpsdata.nrsats) + "," + String(gpsdata.fix) + "," + String((int)gpsdata.cource) + "," + String((int)gpsdata.speed) + "," + String(buoy.mheading) + ">";
        // char bufff[100];
        // strcpy(bufff, msg.c_str());
        // udpsend(bufff);

        if (FRONTBUTTON_READ == PUSHED)
        {
            sw_button_cnt++;
            if (sw_button_cnt == BUTTON_SHORT)
            {
                sw_button_set = BUTTON_SHORT;
                snd_buz.repeat = 0;
                snd_buz.time = 500;
                snd_buz.pauze = 0;
                xQueueSend(Buzzerque, (void *)&snd_buz, 10);
                Serial.println("Button pressed!!!");
            }
            if (sw_button_cnt == BUTTON_LONG)
            {
                sw_button_set = BUTTON_LONG;
                snd_buz.repeat = 0;
                snd_buz.time = 1000;
                snd_buz.pauze = 0;
                xQueueSend(Buzzerque, (void *)&snd_buz, 10);
                Serial.println("Button long pressed!!!");
                beepESC();
            }
            if (sw_button_cnt == BUTTON_SUPER_LONG)
            {
                sw_button_set = BUTTON_SUPER_LONG;
                snd_buz.repeat = 0;
                snd_buz.time = 2000;
                snd_buz.pauze = 0;
                xQueueSend(Buzzerque, (void *)&snd_buz, 10);
                Serial.println("Button super long pressed!!!");
                beepESC();
            }
        }
        else
        {
            sw_button_cnt = 0;
        }
    }

    // do stuff every 0.5 second
    if (sec05stamp + 500 < millis())
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
                SWITCH_GRN_ON; // externel led GREEN (switch)
            }
        }
        xQueueSend(indicatorqueSt, (void *)&snd_sq, 10); // update internal led

        /*
        key handeling
        */
        if (sw_button_set == BUTTON_SHORT && sw_button_cnt == 0) // short button press
        {
            if (status != IDLE) // switch to IDLE status
            {
                status = IDLE;
                BUTTON_LIGHT_OFF;
                beepESC();
                Serial.printf("Status set to >IDLE< = %d\r\n", status);
                buoy.speed = 0;
                buoy.speedbb = 0;
                buoy.speedsb = 0;
            }
            else if (gpsdata.fix == true) // Locking new position
            {
                buoy.tglatitude = gpsdata.lat;
                buoy.tglongitude = gpsdata.lon;
                BUTTON_LIGHT_ON;
                beepESC();
                if (status != CALIBRATE_MAGNETIC_COMPASS || status != CALIBRATE_OFFSET_MAGNETIC_COMPASS)
                {
                    initRudderPid();
                    status = LOCKED;
                    rudderpid.iintergrate = 0;
                    Serial.printf("Status set to >LOCKED< = %d\r\n", status);
                }
            }
        }
        else if (sw_button_set == BUTTON_LONG && sw_button_cnt == 0) // Button long pressed?
        {
            /*
            Set the dock position
            1 Place the buoy on the landing zone (Doc pos).
            2 Wait until the green switch light is burning continusly.
            3 Long press the button until you hear the beep. (ESC initilasing)
            Done!
            */
            if (gpsdata.fix == true && status != CALIBRATE_OFFSET_MAGNETIC_COMPASS && status != STORE_CALIBRATE_OFFSET_MAGNETIC_COMPASS && status != STORE_CALIBRATE_OFFSET_MAGNETIC_COMPASS)
            {
                MemoryDockPos(&gpsdata.lat, &gpsdata.lon, false);
                buoy.magneticCorrection = 0;
                beepESC();
            }
            status = IDLE;
        }
        /*
        Callibration procedure:
        3 Place the buoy on a marked position.
        4 Wait until the green switch light is burning continusly.
        5 Long press the button until you hear the second beep. (ESC initilasing) 2x
        6 Blue led and key(red) are blinking slowly
        7 Place te buoy at least 30 meter from the marked position and point the front of the buoy to that marked point.
        8 long Press the button again until until you hear the second beep again. (ESC initilasing) 2x
        Done !
        */
        else if (sw_button_set == BUTTON_SUPER_LONG && sw_button_cnt == 0) // Button long pressed?
        {
            if (status == IDLE)
            {
                if (gpsdata.fix == true) // GPS fix ok?
                {
                    Serial.println("\r\nStart calibrating new magnetic offset!");
                    MemoryDockPos(&buoy.tglatitude, &buoy.tglongitude, true);
                    buoy.magneticCorrection = 0; // reset callibration
                    status = STORE_CALIBRATE_OFFSET_MAGNETIC_COMPASS;
                    Serial.printf("Status set to >STORE_CALIBRATE_OFFSET_MAGNETIC_COMPASS<\r\n");
                }
            }
            else if (status == STORE_CALIBRATE_OFFSET_MAGNETIC_COMPASS) // check if compass calibration mode is active
            {
                Serial.println("\r\nStoring new magnetic offset!");
                RouteToPoint(gpsdata.lat, gpsdata.lon, buoy.tglatitude, buoy.tglongitude, &buoy.tgdistance, &buoy.tgdir); // calculate heading and speed
                Serial.printf("Mag heading %lf gps heading %f", buoy.mheading, buoy.tgdir);                               // show data
                int tmp = (int)(buoy.tgdir - buoy.mheading);                                                              // compute offset magnetic comass
                CompassOffsetCorrection(&tmp, false);                                                                     // store magnetic comensation
                CompassOffsetCorrection(&buoy.magneticCorrection, true);                                                  // retrieve from eeprom
                Serial.printf("New offset stored: %d\r\n", buoy.magneticCorrection);                                      // show stored data
                status = IDLE;
            }
        }
        if (sw_button_cnt == 0)
        {
            sw_button_set = 0;
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
            BUTTON_LIGHT_OFF;
            if (mcp.digitalRead(MAINSSWITCH_LEDRED_GPB) == 0)
            {
                SWITCH_RED_ON;
                SWITCH_GRN_OFF;
                BUTTON_LIGHT_ON;
            }
            else
            {
                SWITCH_RED_OFF;
                SWITCH_GRN_ON;
                BUTTON_LIGHT_OFF;
            }
            break;

        case DOCKED:
            if (mcp.digitalRead(MAINSSWITCH_LEDRED_GPB) == 0)
            {
                SWITCH_RED_ON;
                SWITCH_GRN_OFF;
                BUTTON_LIGHT_OFF;
            }
            else
            {
                SWITCH_RED_OFF;
                SWITCH_GRN_ON;
                BUTTON_LIGHT_ON;
            }
            if (gpsdata.fix == false) // && buoy.tgdistance < 2000)
            {
                buoy.speedbb = 0; // no fix kill trusters
                buoy.speedsb = 0;
                buoy.speed = 0;
            }
            break;

        case LOCKED:
            BUTTON_LIGHT_ON;
            SWITCH_RED_OFF;
            if (gpsdata.fix == false)
            {
                buoy.speedbb = 0; // no fix kill trusters
                buoy.speedsb = 0;
                buoy.speed = 0;
            }
            break;

        case STORE_CALIBRATE_OFFSET_MAGNETIC_COMPASS: // Calibrate offset magnetic compass due mouning errors
            if (blink == true)
            {
                if (SWITCH_RED_READ)
                {
                    SWITCH_RED_OFF;
                    SWITCH_GRN_ON;
                    BUTTON_LIGHT_OFF;
                }
                else
                {
                    SWITCH_RED_ON;
                    SWITCH_GRN_OFF;
                    BUTTON_LIGHT_ON;
                }
            }
            break;
        case LINEAR_CAL:
            int corr;
            if (linMagCalib(&corr) >= 0)
            {
                status = IDLE;
                if (corr > 0)
                {
                    Serial.printf("\r\n\n\nMagnetick offset now: %d degrees\r\n\n\n", corr);
                    msg_main = "\r\nMagnetick offset now: " + String(corr) + " degrees\r\n\n\n";
                }
                else
                {
                    Serial.printf("\r\n\n\nCallibrtion failure\r\n\n\n");
                    msg_main = "\r\n\n\nCallibrtion failure\r\n\n\n";
                }
            }
            else
            {
                // loraMenu(GPS_LAT_LON_NRSAT_FIX_HEADING_SPEED_MHEADING); // pos heading speed to remote
                loramsg.data = GPS_LAT_LON_NRSAT_FIX_HEADING_SPEED_MHEADING;
                xQueueSend(loradataout, (void *)&loramsg, 10); // update lora
                msg_main = String(buoy.magneticCorrection) + "," + String(buoy.mechanicCorrection);
            }
            strcpy(bufff_main, msg_main.c_str());
            udpsend(bufff_main);
            break;

        default:
            buoy.speed = 0;
            buoy.speedbb = 0;
            buoy.speedsb = 0;
            BUTTON_LIGHT_OFF;
        }
        udateDisplay(buoy.speedsb, buoy.speedbb, (unsigned long)buoy.tgdistance, (unsigned int)buoy.tgdir, (unsigned int)buoy.mheading, gpsvalid);
        // loraMenu(MAGNETIC_HEADING); // pos heading speed to remote
        /*Each second add magnetic heading for wind directon calulations*/
        if (blink == true)
        {
            addNewSampleInBuffer(buoy.winddir, BUFLENMHRG, buoy.mheading);
        }
        blink = !blink;
    }

    /*
    Send only updated changes
    */
    if (gpsstamp + 900 < millis())
    {
        gpsstamp = millis();
        if (status == LOCKED || status == DOCKED)
        {
            if (distanceChanged != buoy.tgdistance)
            {
                if (gpsdata.fix == true)
                {
                    distanceChanged = buoy.tgdistance;
                    loramsg.data = DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_M_HEADING;
                    xQueueSend(loradataout, (void *)&loramsg, 10); // update lora
                }
            }
        }
    }

    /*
     do stuff every 2,5 sec
     Send periodic status info
    */
    if (sec5stamp + 2500 < millis())
    {
        sec5stamp = millis() - 1;

        // double invertdir = buoy.tgdir;
        loramsg.data = GPS_LAT_LON_NRSAT_FIX_HEADING_SPEED_MHEADING;
        xQueueSend(loradataout, (void *)&loramsg, 10); // update lora
        loraIn.recipient = 0xFE;
        msg_cnt++;
        if (status == LOCKED || status == DOCKED || status == REMOTE)
        {
            String msg = String(buoy.tgdir) + "," + String(buoy.tgdistance) + "," + buoy.speed + "," + buoy.speedbb + "," + buoy.speedsb + "," + String(buoy.mheading, 0);
            char bufff[100];
            strcpy(bufff, msg.c_str());
            udpsend(bufff);
            loramsg.data = PID_RUDDER_PARAMETERS;
            xQueueSend(loradataout, (void *)&loramsg, 10); // update lora
            loramsg.data = PID_SPEED_PARAMETERS;
            xQueueSend(loradataout, (void *)&loramsg, 10); // update lora
        }
        switch (msg_cnt)
        {
        case 1:
            loramsg.data = COMPUTE_PARAMETERS;
            xQueueSend(loradataout, (void *)&loramsg, 10); // update lora
            // loramsg.data = GPS_LAT_LON_NRSAT_FIX_HEADING_SPEED_MHEADING;
            // xQueueSend(loradataout, (void *)&loramsg, 10); // update lora
            break;
        case 2:

            battVoltage(&buoy.vbatt, &buoy.vperc);
            loramsg.data = BATTERY_VOLTAGE_PERCENTAGE;
            xQueueSend(loradataout, (void *)&loramsg, 10); // update lora
            break;
        case 3:
            loramsg.data = SBPWR_BBPWR;
            xQueueSend(loradataout, (void *)&loramsg, 10); // update lora
            break;
        case 4:
            msg_cnt = 0;
            loramsg.data = WIND_DIR_DEV;
            xQueueSend(loradataout, (void *)&loramsg, 10); // update lora
            loramsg.data = COMPUTE_PARAMETERS;
            xQueueSend(loradataout, (void *)&loramsg, 10); // update lora

            if (buoy.vbatt <= 15)
            {
                Message snd_msg;
                snd_msg.speedbb = 0;
                snd_msg.speedsb = 0;
                xQueueSend(escspeed, (void *)&snd_msg, 10); // update esc
                delay(5000);
                adc_switch(); /*read switch status*/
                delay(100);
                adc_switch(); /*read switch status*/
                if (buoy.vbatt > 20)
                {
                    break;
                }
                status = DOCKED; // empty batt sailing home
            }
            break;
        }
    }

    int nr;
    if (serialPortDataIn(&nr))
    {
        Serial.printf("New data on serial port: %d\n", nr);
    }

    // trigger esc to prefent from beeping
    if (millis() - esctrigger > ESC_TRIGGER)
    {
        if (buoy.speedsb == 0 && buoy.speedbb == 0)
        {
            triggerESC();
        }
        esctrigger = millis();
    }

    /*
     * Sending speed to ESC and indicator
     */
    if (spdbb != buoy.speedbb || spdsb != buoy.speedsb)
    {
        esctrigger = millis();
        if (spdbb < buoy.speedbb) // slowly go to setpoint
        {
            spdbb++;
        }
        else if (spdbb > buoy.speedbb)
        {
            spdbb--;
        }
        if (spdsb < buoy.speedsb)
        {
            spdsb++;
        }
        else if (spdsb > buoy.speedsb)
        {
            spdsb--;
        }
    }
    snd_msg.speedbb = spdbb;
    snd_msg.speedsb = spdsb;
    if (buoy.muteEsc == true)
    {
        snd_msg.speedbb = 0;
        snd_msg.speedsb = 0;
    }
    xQueueSend(escspeed, (void *)&snd_msg, 10); // update esc
    snd_sp.speedbb = spdbb;
    snd_sp.speedsb = spdsb;
    xQueueSend(indicatorqueSp, (void *)&snd_sp, 10); // send to led indicator
}