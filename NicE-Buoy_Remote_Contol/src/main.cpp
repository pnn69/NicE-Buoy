/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
https://www.lilygo.cc/products/lora3
https://github.com/Xinyuan-LilyGO/TTGO-LoRa-Series
https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/T3_V1.6.1.pdf
433MHz is SX1278
*/
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <WiFi.h>
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

#define SHORTKEYDELAY 2
#define LONGKEYDELAY 100
#define MAX_REPEAT 10

unsigned long timestamp, msecstamp, hsecstamp, sec5stamp, esctamp, checkAckStamp;
// static double gpslatitude = 52.34567, gpslongitude = 4.567;                     // home
// static double tglatitude = 52.29326976307006, tglongitude = 4.9328016467347435; // grasveld wsvop
//  static double tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
// static unsigned long tgdir = 0, tgdistance = 0;
unsigned long previousTime = 0;
int speedbb = 0, speedsb = 0;
unsigned int button_cnt = 0;
unsigned int lst_button_cnt = 0;
bool ledstatus = false;
int maxRepeatCount[NR_BUOYS + 1];
String str = "";
char bufferin[100];
char bufferout[8][100];
buoyDataType buoy[NR_BUOYS];
BluetoothSerial SerialBT;

void setup()
{
    char ssidl[26];
    char buf[40];
    byte mac[6];
    WiFi.macAddress(mac);
    strcpy(ssidl, "NicE_Buoy_Control");
    sprintf(buf, "%s-%02x:%02x:%02x:%02x:%02x:%02x", ssidl, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    SerialBT.begin(buf); // Bluetooth device name
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(SW_P_1, INPUT_PULLUP);
    pinMode(SW_P_2, OUTPUT);
    digitalWrite(LED_PIN, false);
    digitalWrite(SW_P_2, false);
    initSSD1306();
    Wire.begin();
    InitLora();
    // websetup();
    readAdc();
    // InitGps();
    color_printf(COLOR_PRINT_BLUE, "Setup running");
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
    checkAckStamp = millis();
}

void loop()
{
    // webloop();

    if (polLora())
    {
        notify = loraIn.sender;
    }
    /*
        check if messages are acknowleged.
        if not resend
        */
    if (checkAckStamp + 250 < millis())
    {
        checkAckStamp = millis();
        for (int i = 1; i < NR_BUOYS; i++)
        {
            if (buoy[i].ackOK == false && maxRepeatCount[i] < MAX_REPEAT)
            {
                Serial.printf("<Repeat command %d><%d times>", buoy[i].cmnd, maxRepeatCount[i]);
                loraMenu(i);
                maxRepeatCount[i]++;
                checkAckStamp = 750 + millis();
            }
            else
            {
                maxRepeatCount[i] = 0;
                buoy[i].ackOK = true;
            }
        }
    }

    /*
    runs each 10 msec
    */
    if (millis() - msecstamp >= 10)
    {
        msecstamp = millis();
        digitalWrite(LED_PIN, ledstatus);
        ledstatus = false;
        if (digitalRead(SW_P_1) == PUSHED)
        {
            button_cnt++;
            lst_button_cnt = button_cnt;
        }
        else
        {
            button_cnt = 0;
            if (lst_button_cnt < SHORTKEYDELAY)
            {
                lst_button_cnt = 0;
            }
        }
    }

    if (button_cnt > SHORTKEYDELAY)
    {
        if (button_cnt > LONGKEYDELAY)
        {
            String out = "LONG";
            showDip(3, out);
        }
        else
        {
            String out = "SHORT";
            showDip(3, out);
        }
    }
    else
    {
        updateDisplay();
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
        readAdc();
        if (button_cnt == 0 && lst_button_cnt > SHORTKEYDELAY)
        {
            /*
            Pushbutton short pressed here
            action taken depends on the position of the switch (LEFT MID RIGHT)
            */
            if (lst_button_cnt < LONGKEYDELAY)
            {
                if (sw_pos == SW_LEFT)
                {
                    buoy[1].status = LOCKING;
                    buoy[1].cmnd = TARGET_POSITION;
                    buoy[1].gsa = SET;
                    buoy[1].ackOK = false;
                    loraMenu(1);
                    String out = "LOCKED";
                    showDip(3, out);
                    delay(2000);
                    notify = true;
                }
                else if (sw_pos == SW_MID)
                {
                    buoy[1].cdir = 0;
                    buoy[1].cspeed = 0;
                    buoy[1].speed = 0;
                    buoy[1].speedbb = 0;
                    buoy[1].speedsb = 0;
                    buoy[1].status = IDLE;
                    buoy[1].cmnd = BUOY_MODE_IDLE;
                    buoy[1].gsa = SET;
                    buoy[1].ackOK = false;
                    loraMenu(1);
                    radiobutton[1] = 6;
                    notify = true;
                    String out = "IDLE";
                    showDip(4, out);
                    delay(2000);
                }
                else if (sw_pos == SW_RIGHT)
                {
                    adc.newdata = true;
                    radiobutton[1] = 2;
                    notify = true;
                    buoy[1].cmnd = SAIL_DIR_SPEED;
                    buoy[1].status = REMOTE;
                    loraMenu(1);
                    String out = "REMOTE";
                    showDip(3, out);
                    delay(2000);
                }
            }
            /*
            Pushbutton long pressed here
            action taken depends on the position of the switch (LEFT MID RIGHT)
            */
            else
            {
                if (sw_pos == SW_LEFT)
                {
                    buoy[1].status = DOCKING;
                    buoy[1].cmnd = DOC_POSITION;
                    buoy[1].gsa = SET;
                    buoy[1].ackOK = false;
                    loraMenu(1);
                    String out = "SAILING\r\nTO DOCK";
                    showDip(3, out);
                    delay(3000);
                    notify = true;
                }

                else if (sw_pos == SW_MID)
                {
                    String out = "No\r\n\r\nActon";
                    showDip(2, out);
                    delay(1000);
                }
                else if (sw_pos == SW_RIGHT) // sail to dock positon
                {
                    String out = "No\r\n\r\nActon";
                    showDip(2, out);
                    delay(1000);
                }
            }
            lst_button_cnt = 0;
        }

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
                loraMenu(i);
            }
        }
    }

    if (millis() - previousTime >= 50000)
    { // do stuff every second
        previousTime = millis();
        for (int i = 1; i < NR_BUOYS; i++)
            if (buoy[i].ackOK == true)
            {
                if (buoy[i].status == LOCKED || buoy[i].status == DOCKED)
                {
                    buoy[i].cmnd = DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_M_HEADING;
                    buoy[i].gsa = GET;
                    loraMenu(i);
                }
                else if (buoy[i].status == REMOTE)
                {
                    // Serial.printf("New data from ADC! speed:%d rudder:%d\r\n", adc.speed, adc.rudder);
                    buoy[i].cspeed = adc.speed;
                    buoy[i].cdir = adc.rudder;
                    buoy[i].cmnd = SAIL_DIR_SPEED;
                    buoy[i].gsa = SET;
                    adc.newdata = false;
                    loraMenu(i);
                }
            }
    }
    /*
    use ^ as delimiter
    buoyID^cmnd^SET^msg
    */
    if (Serial.available())
    {
        Serial.setTimeout(500);
        str = Serial.readString();
        str.toCharArray(bufferin, str.length() + 1);
    }
    if (SerialBT.connected() && SerialBT.available())
    {
        SerialBT.setTimeout(500);
        str = SerialBT.readString();
        str.toCharArray(bufferin, str.length() + 1);
    }
    if (str != "")
    {
        int i = 0;
        //'ID'DEST'msgID'GSA'MSG'
        // Serial.println("Data in:" + String(bufferin));
        char *token = strtok(bufferin, "^");
        while (token != NULL && i < 6)
        {
            strcpy(bufferout[i], token);
            token = strtok(NULL, "^");
            i++;
        }
        // Serial.printf("Buf[0] %s\r\n", bufferout[0]);
        if (strstr(bufferout[0], "*"))
        { //"^*^1^23^1^maffe data^1"
            int nr = 1;
            int t;
            sscanf(bufferout[1], "%d", &nr);
            sscanf(bufferout[2], "%d", &t);
            buoy[nr].cmnd = (byte)t;
            sscanf(bufferout[3], "%d", &t);
            buoy[nr].gsa = (byte)t;
            buoy[nr].string = String(bufferout[4]);
            sscanf(bufferout[5], "%d", &t);
            buoy[nr].ackOK = (bool)t;
            Serial.printf("RS2332 command:<buoynr:%d><cmnd:%d><gsa:%d><msg:", nr, buoy[nr].cmnd, buoy[nr].gsa);
            Serial.print(buoy[nr].string);
            Serial.printf("><ACK:%d>\r\n", (int)buoy[nr].ackOK);
            loraMenu(1);
        }
        str = "";
    }
    delay(1);
}