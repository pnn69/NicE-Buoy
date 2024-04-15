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
const byte numChars = 5;
int maxRepeatCount[NR_BUOYS + 1];
char receivedChars[numChars]; // an array to store the received data

buoyDataType buoy[NR_BUOYS];

/*
Read out serial port and return data after <cr>
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
                Serial.printf("Repeat command %d, %d times\r\n", buoy[i].cmnd, maxRepeatCount[i]);
                while (loraMenu(i))
                    ;
                maxRepeatCount[i]++;
                checkAckStamp = 1500 + millis();
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
                    String out = "LOCKED";
                    showDip(3, out);
                    delay(1000);
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
                    radiobutton[1] = 6;
                    notify = true;
                    String out = "IDLE";
                    showDip(4, out);
                    delay(1000);
                }
                else if (sw_pos == SW_RIGHT)
                {
                    String out = "REMOTE";
                    showDip(3, out);
                    delay(1000);
                    adc.newdata = true;
                    radiobutton[1] = 2;
                    notify = true;
                    buoy[1].cmnd = SAIL_DIR_SPEED;
                    buoy[1].status = REMOTE;
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
                    String out = "SAILING\r\nTO DOCK";
                    showDip(3, out);
                    delay(1000);
                    notify = true;
                }

                else if (sw_pos == SW_MID)
                {
                    buoy[1].string = "0,-0.1,0,0";
                    buoy[1].cmnd = PID_PARAMETERS;
                    buoy[1].gsa = SET;
                    buoy[1].ackOK = false;
                    String out = "PID ki\r\n -0.1";
                    showDip(4, out);
                    delay(1000);
                }
                else if (sw_pos == SW_RIGHT) // sail to dock positon
                {
                    buoy[1].string = "0,0.1,0,0";
                    buoy[1].cmnd = PID_PARAMETERS;
                    buoy[1].gsa = SET;
                    buoy[1].ackOK = false;
                    String out = "PID ki\r\n +0.1";
                    showDip(4, out);
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
                while (loraMenu(i))
                    ;
                checkAckStamp = millis();
                delay(200);
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
                    while (loraMenu(i))
                        ;
                    checkAckStamp = millis();
                }
                else if (buoy[i].status == REMOTE)
                {
                    // Serial.printf("New data from ADC! speed:%d rudder:%d\r\n", adc.speed, adc.rudder);
                    buoy[i].cspeed = adc.speed;
                    buoy[i].cdir = adc.rudder;
                    buoy[i].cmnd = SAIL_DIR_SPEED;
                    buoy[i].gsa = SET;
                    adc.newdata = false;
                    while (loraMenu(i))
                        ;
                    checkAckStamp = millis();
                }
            }
    }

    if (Serial.available())
    {
        char nr;
        // if (serialPortDataIn(&nr))
        // {
        //     Serial.printf("New data on serial port: %d\n", nr);
        // }
        nr = Serial.read();
        Serial.printf("New data on serial port: %c\n", nr);
        switch (nr)
        {
        case '1':
            buoy[1].cmnd = GPS_DUMMY;
            buoy[1].dataout = 1;
            loraMenu(1);
            buoy[1].cmnd = 0;
            break;
        case '2':
            buoy[1].cmnd = GPS_DUMMY;
            buoy[1].dataout = 0;
            loraMenu(1);
            buoy[1].cmnd = 0;
            break;
        case '+':
            buoy[1].cmnd = GPS_DUMMY_DELTA_LAT_LON;
            buoy[1].dataout = 1;
            loraMenu(1);
            buoy[1].cmnd = 0;
            break;
        case '-':
            buoy[1].cmnd = GPS_DUMMY_DELTA_LAT_LON;
            buoy[1].dataout = 0;
            loraMenu(1);
            buoy[1].cmnd = 0;
            break;
        case 'e':
            buoy[1].cmnd = ESC_ON_OFF;
            buoy[1].dataout = 1;
            loraMenu(1);
            buoy[1].cmnd = 0;
            break;
        case 'r':
            buoy[1].cmnd = ESC_ON_OFF;
            buoy[1].dataout = 0;
            loraMenu(1);
            buoy[1].cmnd = 0;
            break;
        }
    }

    delay(1);
}