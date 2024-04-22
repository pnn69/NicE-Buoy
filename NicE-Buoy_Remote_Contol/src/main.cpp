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

buoyDataType buoy[NR_BUOYS];

void setup()
{
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
                checkAckStamp = 500 + millis();
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
        char bufferin[100];
        char bufferout[4][100];
        Serial.setTimeout(500);
        String str = Serial.readString();
        str.toCharArray(bufferin, str.length() + 1);
        int i = 0;
        Serial.println("Dat in:" + String(bufferin));
        char *token = strtok(bufferin, "^");
        while (token != NULL && i < 4)
        {
            strcpy(bufferout[i], token);
            token = strtok(NULL, "^");
            i++;
        }
        
        /*
        1_CHANGE_POS_DIR_DIST'SET'-90,10
        1'CHANGE_POS_DIR_DIST'SET'90,10
        1'CHANGE_POS_DIR_DIST'SET'0,10
        1'CHANGE_POS_DIR_DIST'SET'180,10
        */
        if (strstr(bufferout[1], "CHANGE_POS_DIR_DIST"))
        {
            Serial.println("CHANGE_POS_DIR_DIST");
            buoy[1].string = String(bufferout[3]);
            buoy[1].cmnd = CHANGE_LOCK_POS_DIR_DIST;
            if (strstr(bufferout[2], "SET"))
            {
                buoy[1].gsa = SET;
            }
            else
            {
                buoy[1].gsa = ACK;
            }
            buoy[1].ackOK = false;
            loraMenu(1);
        }
        /*
            1'PID_RUDDER_PARAMETERS'SET'10,1.4,5
        */
        if (strstr(bufferout[1], "PID_SPEED_PARAMETERS"))
        {
            Serial.println("PID_SPEED_PARAMETERS (default) 1'PID_RUDDER_PARAMETERS'SET'20,0.4,0");
            buoy[1].string = String(bufferout[3]);
            buoy[1].cmnd = PID_SPEED_PARAMETERS;
            if (strstr(bufferout[2], "SET"))
            {
                buoy[1].gsa = SET;
            }
            else
            {
                buoy[1].gsa = ACK;
            }
            buoy[1].ackOK = false;
            loraMenu(1);
        }
        if (strstr(bufferout[1], "PID_RUDDER_PARAMETERS"))
        {
            Serial.println("PID_RUDDER_PARAMETERS (default) 1'PID_RUDDER_PARAMETERS'SET'0.2,0.02,0.1'");
            buoy[1].string = String(bufferout[3]);
            buoy[1].cmnd = PID_RUDDER_PARAMETERS;
            if (strstr(bufferout[2], "SET"))
            {
                buoy[1].gsa = SET;
            }
            else
            {
                buoy[1].gsa = ACK;
            }
            buoy[1].ackOK = false;
            loraMenu(1);
        }
    }
    delay(1);
}