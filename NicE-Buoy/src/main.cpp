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
#include "../../dependency/command.h"

unsigned long secstamp, msecstamp, updatestamp;
static float heading = 0;
static double gpslatitude = 0, gpslongitude = 0;                                //
static double tglatitude = 52.29326976307006, tglongitude = 4.9328016467347435; // grasveld wsvop
// static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
static unsigned long tgdir = 0, tgdistance = 0, cdir = 0;
int speedbb = 0, speedsb = 0, speed = 0, cspeed = 0;
bool ledstatus = false;
char buoyID = 0;
char status = IDLE; // IDLE
const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;
int dataNumber = 0; // new for this version

void recvWithEndMarker()
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
            newData = true;
        }
    }
}

void showNewNumber()
{
    if (newData == true)
    {
        dataNumber = 0;                   // new for this version
        dataNumber = atoi(receivedChars); // new for this version
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        Serial.print("Data as Number ... "); // new for this version
        Serial.println(dataNumber);          // new for this version
        newData = false;
    }
}

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, false);
    InitMemory();
    GetMemoryBuoyID(&buoyID);
    InitLora();
    InitCompass();
    InitGps();
    initSSD1306();
    xTaskCreate(EscTask, "EscTask", 2400, NULL, 5, NULL);
    xTaskCreate(IndicatorTask, "IndicatorTask", 2400, NULL, 5, NULL);
    Serial.printf("BuoyID = %d\n\r", buoyID);
    Serial.printf("Status = %d\n\r", status);
    Serial.println("Setup done.");
    secstamp = millis();
    msecstamp = millis();
    status = IDLE; // IDLE
    // SetMemoryDockPos(52.29308075283747, 4.932570409845357);
}

void loop()
{
    if (loraOK)
    {
        int msg = polLora();
        if (msg)
        {
            secstamp = 0;
            String decode = loraIn.message;
            char tmparr[100];
            int ddir;
            Serial.printf("Message recieved ");
            Serial.print("Sender:" + String(loraIn.sender) + " RSSI:" + String(loraIn.rssi) + " msg:" + msg + "\r\n");
            switch (msg)
            {
            case GET_POSITION:
                sendLoraPos(POSITION, gpslatitude, gpslongitude);
                break;
            case GET_DIR_DISTANSE_TARGET_POSITION:
                sendLoraDirDistanceTarget(tgdir, tgdistance);
                break;
            case SET_ANCHOR_POSITION_AS_TARGET_POSITION:
                // Serial.print("Set target to  ANCHOR position");
                GetMemoryAnchorPos(&tglatitude, &tglongitude);
                sendLoraPos(TARGET_POSITION, tglatitude, tglongitude);
                status = LOCKED;
                break;
            case SET_ANGHOR_POSITION:
                if (gpslatitude != 0 || gpslongitude != 0)
                {
                    SetMemoryAnchorPos(gpslatitude, gpslongitude);
                    tglatitude = gpslatitude;
                    tglongitude = gpslongitude;
                }
                status = LOCKED;
                break;

            case GOTO_DOC_POSITION:
                // Serial.print("Set target to  Doc position");
                GetMemoryDockPos(&tglatitude, &tglongitude);
                sendLoraPos(TARGET_POSITION, tglatitude, tglongitude);
                status = LOCKED;
                break;
            case SET_CURREND_POSITION_AS_ANCHOR_POSITION:
                if (gpslatitude != 0 || gpslongitude != 0)
                {
                    SetMemoryAnchorPos(gpslatitude, gpslongitude); // put current position in memory as anchor position
                }
                sendLoraPos(ANCHOR_POSITION, tglatitude, tglongitude);
                status = LOCKED;
                break;
            case SET_TARGET_POSITION:
                if (gpslatitude != 0 || gpslongitude != 0)
                {
                    tglatitude = gpslatitude;
                    tglongitude = gpslongitude;
                }
                status = LOCKED;
                break;
            case SET_SAIL_DIR_SPEED:
                decode.toCharArray(tmparr, decode.length() + 1);
                sscanf(tmparr, "%d,%d", &ddir, &cspeed);
                ddir = heading - ddir;
                if (ddir < 0)
                {
                    ddir = 360 - ddir;
                }
                if (ddir > 360)
                {
                    ddir = ddir - 360;
                }
                cdir = ddir;
                status = REMOTE;
                break;
            case SET_BUOY_MODE_IDLE:
                status = IDLE;
                break;

            case RESET:
                ESP.restart();
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
    }

    // do stuff every 0.5 second
    if (millis() - secstamp > 1000)
    {
        secstamp = millis();
        if (GetNewGpsData(&gpslatitude, &gpslongitude))
        {
            // GpsAverage(&gpslatitude, &gpslongitude);
        }
        RouteToPoint(gpslatitude, gpslongitude, tglatitude, tglongitude, &tgdistance, &tgdir); // calculate heading and distance
        if (tgdistance < 5000)                                                                 // test if target is in range
        {
            speed = CalcEngingSpeedBuoy(heading, tgdir, tgdistance, &speedbb, &speedsb);
        }
        else
        {
            speed = CalcEngingSpeedBuoy(heading, heading, 0, &speedbb, &speedsb); // do notihing
        }

        /*
        Do stuff depending on the status of the buoy
        */
        switch (status)
        {
        case IDLE:
            speedbb = 0;
            speedsb = 0;
            break;
        case REMOTE:
            CalcEngingSpeed(cdir, (unsigned long)heading, cspeed, &speedbb, &speedsb);
            updatestamp = 0;
            // Serial.printf("Sail tgdir:%d heading:%.0f speed:%d bb:%d sb:%d\r\n", cdir, heading, cspeed, speedbb, speedsb);
            break;
        case LOCKED:
            status = LOCKED;
            break;
        case UNLOCK:
            status = IDLE;
            break;
        case LOCK_ANCHOR_POS:
            GetMemoryAnchorPos(&tglatitude, &tglongitude);
            status = LOCKED;
            break;
        case LOCK_POS:
            tglatitude = gpslatitude;
            tglongitude = gpslongitude;
            status = LOCKED;
            break;
        default:
            Serial.println("unknown command");
            break;
        }
        Message snd_msg;
        snd_msg.speedbb = speedbb;
        snd_msg.speedsb = speedsb;
        xQueueSend(escspeed, (void *)&snd_msg, 10);
        digitalWrite(led_pin, ledstatus);
        ledstatus = !ledstatus;
        udateDisplay(speedsb, speedbb, tgdistance, tgdir, (unsigned long)heading, gpsvalid);
    }

    // do stuff every 5 second
    if (millis() - updatestamp > 1000)
    {
        updatestamp = millis();
        sendLoraDirDistanceSbSpeedBbSpeedTarget(tgdir, tgdistance, speed, speedbb, speedsb, (int)heading);
        delay(100);
    }

    recvWithEndMarker();
    showNewNumber();

    delay(1);
}