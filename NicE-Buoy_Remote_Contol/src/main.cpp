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
#include "webinterface.h"
#include "menu.h"
#include "../../dependency/command.h"

unsigned long timestamp, msecstamp;
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

buoyDataType buoy[NR_BUOYS];

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
    Serial.begin(115200);
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, false);
    initSSD1306();
    Wire.begin();
    InitLora();
    websetup();
    Serial.println("Setup done.");
    delay(1000);
// fill buoy with dummy data
#ifdef DEBUG
    buoy[1].gpslatitude = 52.1;
    buoy[1].gpslongitude = 4.1;
    buoy[1].rssi = loraIn.rssi - 11;
    buoy[2].gpslatitude = 52.2;
    buoy[2].gpslongitude = 4.2;
    buoy[2].rssi = loraIn.rssi - 22;
    buoy[3].gpslatitude = 52.3;
    buoy[3].gpslongitude = 4.3;
    buoy[3].rssi = loraIn.rssi - 33;
#endif
    timestamp = millis();
    msecstamp = millis();
}

void loop()
{
    webloop();
    if (loraOK)
    {
        int msg = polLora();
        if (msg)
        {
            String decode = loraIn.message;
            char tmparr[100];
            unsigned long dist, dir;
            int sp, sb, bb, he, st;
            // Serial.printf("Lora messag recieved ");
            // Serial.print("Sender:" + String(loraIn.sender) + " RSSI:" + String(loraIn.rssi) + " msg:" + msg + "\r\n");
            int id = loraIn.sender;
            switch (msg)
            {
            case (POSITION):
                // Serial.println("Position recieved!");
                decode.toCharArray(tmparr, decode.length() + 1);
                double lat1, lon1;
                sscanf(tmparr, "%lf,%lf", &lat1, &lon1);
                if (NR_BUOYS > loraIn.sender)
                {
                    buoy[loraIn.sender].gpslatitude = lat1;
                    buoy[loraIn.sender].gpslongitude = lon1;
                    buoy[loraIn.sender].rssi = loraIn.rssi;
                }
                break;
            case (DIR_DISTANSE_TO_TARGET_POSITION):
                // Serial.println("direction and distance target recieved!");
                decode.toCharArray(tmparr, decode.length() + 1);
                sscanf(tmparr, "%d,%d", &dir, &dist);
                if (NR_BUOYS > loraIn.sender)
                {
                    buoy[loraIn.sender].tgdir = dir;
                    buoy[loraIn.sender].tgdistance = dist;
                    buoy[loraIn.sender].rssi = loraIn.rssi;
                }
                break;
            case (DIR_DISTANSE_SPEED_SBSPPEED_BBSPEED_TARGET_POSITION):
                // Serial.println("direction and distance target recieved!");
                decode.toCharArray(tmparr, decode.length() + 1);
                sscanf(tmparr, "%d,%d,%d,%d,%d,%d", &dir, &dist, &sp, &sb, &bb, &he);
                if (NR_BUOYS > loraIn.sender)
                {
                    buoy[loraIn.sender].tgdir = dir;
                    buoy[loraIn.sender].mdir = he;
                    buoy[loraIn.sender].tgdistance = dist;
                    buoy[loraIn.sender].speed = sp;
                    buoy[loraIn.sender].speedsb = sb;
                    buoy[loraIn.sender].speedbb = bb;
                    buoy[loraIn.sender].rssi = loraIn.rssi;
                }
                break;

            case (DIR_DISTANSE_SPEED_SBSPPEED_BBSPEED_TARGET_POSITION_STATUS):
                // Serial.println("direction and distance target recieved!");
                decode.toCharArray(tmparr, decode.length() + 1);
                sscanf(tmparr, "%d,%d,%d,%d,%d,%d,%d", &dir, &dist, &sp, &sb, &bb, &he, &st);
                if (NR_BUOYS > loraIn.sender)
                {
                    buoy[loraIn.sender].tgdir = dir;
                    buoy[loraIn.sender].mdir = he;
                    buoy[loraIn.sender].tgdistance = dist;
                    buoy[loraIn.sender].speed = sp;
                    buoy[loraIn.sender].speedsb = sb;
                    buoy[loraIn.sender].speedbb = bb;
                    buoy[loraIn.sender].rssi = loraIn.rssi;
                }
                break;
            case SAIL_DIR_SPEED:
                decode.toCharArray(tmparr, decode.length() + 1);
                sscanf(tmparr, "%d,%d,%d,%d", &he, &sp, &sb, &bb);
                if (NR_BUOYS > loraIn.sender)
                {
                    buoy[loraIn.sender].tgdir = 0;
                    buoy[loraIn.sender].mdir = he;
                    buoy[loraIn.sender].tgdistance = 0;
                    buoy[loraIn.sender].speed = sp;
                    buoy[loraIn.sender].speedsb = sb;
                    buoy[loraIn.sender].speedbb = bb;
                    buoy[loraIn.sender].rssi = loraIn.rssi;
                }
                break;
            case TARGET_POSITION_SET:
                Serial.println("New anchor position SET!");
                buoy[loraIn.sender].status = LOCKED;
                break;

            case ANCHOR_POSITION:
                Serial.println("New anchor position recieved!");
                Serial.println(String(loraIn.message));
                if (NR_BUOYS > loraIn.sender)
                {
                    buoy[loraIn.sender].rssi = loraIn.rssi;
                }
                break;
            default:
                Serial.println("unknown command: " + msg);
                break;
            }
            udateDisplay();
            notify = loraIn.sender;
        }
    }
    /*
    runs each 10 msec
    */
    if (millis() - msecstamp >= 10)
    {
        msecstamp = millis();
        digitalWrite(led_pin, ledstatus);
        ledstatus = false;
    }

    if (millis() - previousTime >= 1000)
    { // do stuff every second
        previousTime = millis();
        if (buoy[1].status == REMOTE)
        {
            menu(REMOTE, 1);
        }
        else if (buoy[1].status == LOCKING)
        {
            menu(SET_TARGET_POSITION, 1);
        }
        else if (buoy[1].status == LOCKED)
        {
            menu(GET_DIR_DISTANSE_SPEED_SBSPPEED_BBSPEED_TARGET_POSITION_STATUS, 1);
        }
        else
        {
            sendLoraSetGetPosition(1);
        }
    }
    int nr;
    if (serialPortDataIn(&nr))
    {
        Serial.printf("New data on serial port: %d\n", nr);
        menu(nr, 1);
    }

    delay(1);
}