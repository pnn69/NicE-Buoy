#include <Arduino.h>
#include <RoboTone.h>
#include <RoboCompute.h>
#include "main.h"
#include "freertos/task.h"
#include "io_sub.h"
#include "subwifi.h"
#include "datastorage.h"
#include "leds.h"
#include "esc.h"
#include "compass.h"
#include "buzzer.h"
#include "adc.h"
#define HOST_NAME "RoboBuoySub"

// pid subparameter;
static RoboStruct mainData;
static char udpInMain[MAXSTRINGLENG];
Message esc;
static int8_t buoyId;
static int subStatus = IDLE;
static LedData mainLedStatus;
static PwrData mainPwrData;
static Buzz mainBuzzerData;
static int wifiConfig = 0;

int buttonState = 0;              // Current state of the button
int lastButtonState = 0;          // Previous state of the button
unsigned long lastPressTime = 0;  // Time of the last press
unsigned long debounceDelay = 50; // Debounce time in milliseconds
int pressCount = 0;               // Count the number of button presses
bool debounce = false;            // Debouncing flag

int countKeyPressesWithTimeout()
{
    // Get the current time
    unsigned long currentTime = millis();
    buttonState = digitalRead(BUTTON_PIN);
    // Check if the button is pressed and it's a new press (debounce)
    if (buttonState == HIGH && lastButtonState == LOW && !debounce)
    {
        pressCount++;                // Increment the button press count
        debounce = true;             // Set debounce flag
        delay(debounceDelay);        // Simple debouncing by adding a delay
        lastPressTime = currentTime; // Record the time of the last press
    }
    // Reset debounce flag if the button is released
    if (buttonState == LOW)
    {
        debounce = false;
    }
    // If more than 2 seconds have passed without a press, return the count
    if ((currentTime - lastPressTime) > 500 && pressCount > 0)
    {
        int finalPressCount = pressCount; // Store the current press count
        pressCount = 0;                   // Reset the press count after returning
        return finalPressCount;           // Return the number of key presses
    }
    lastButtonState = buttonState; // Save the last button state
    return -1;                     // Return -1 if 0.5 seconds haven't passed yet
}

void calibrateNorthCompas(void)
{
    mainBuzzerData.hz = 1000;
    mainBuzzerData.repeat = 5;
    mainBuzzerData.pause = 50;
    mainBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&mainBuzzerData, 10); // update util led
    mainPwrData.bb = CRGB::Orange;
    mainPwrData.sb = CRGB::Orange;
    mainPwrData.blinkBb = BLINK_FAST;
    mainPwrData.blinkSb = BLINK_FAST;
    xQueueSend(ledPwr, (void *)&mainPwrData, 10); // update util led
    delay(1000);
    calibrateMagneticNorth();
    mainPwrData.bb = CRGB::Black;
    mainPwrData.sb = CRGB::Black;
    mainPwrData.blinkBb = BLINK_OFF;
    mainPwrData.blinkSb = BLINK_OFF;
    xQueueSend(ledPwr, (void *)&mainPwrData, 10); // update util led
    mainBuzzerData.hz = 1000;
    mainBuzzerData.repeat = 5;
    mainBuzzerData.pause = 50;
    mainBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&mainBuzzerData, 10); // update util led
}

void calibrateParametersCompas(void)
{
    mainLedStatus.color = CRGB::DarkBlue;
    mainPwrData.bb = CRGB::DarkBlue;
    mainPwrData.sb = CRGB::DarkBlue;
    mainLedStatus.blink = BLINK_FAST;
    mainPwrData.blinkBb = BLINK_FAST;
    mainPwrData.blinkSb = BLINK_FAST;
    xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
    xQueueSend(ledPwr, (void *)&mainPwrData, 10);      // update util led
    mainBuzzerData.hz = 1000;
    mainBuzzerData.repeat = 10;
    mainBuzzerData.pause = 50;
    mainBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&mainBuzzerData, 10); // update util led
    delay(1000);
    CalibrateCompass();
    mainBuzzerData.hz = 1000;
    mainBuzzerData.repeat = 10;
    mainBuzzerData.pause = 50;
    mainBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&mainBuzzerData, 10); // update util led
    mainLedStatus.color = CRGB::Black;
    mainPwrData.bb = CRGB::Black;
    mainPwrData.sb = CRGB::Black;
    mainLedStatus.blink = BLINK_OFF;
    mainPwrData.blinkBb = BLINK_OFF;
    mainPwrData.blinkSb = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
    xQueueSend(ledPwr, (void *)&mainPwrData, 10);      // update util led
}

void setup()
{
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT);
    printf("\r\nSetup running!\r\n");
    printf("Robobuoy Sub Version: %0.1f\r\n", SUBVERSION);
    delay(10);
    initbuzzerqueue();
    initledqueue();
    initescqueue();
    initwifiqueue();
    initMemory();
    initcompassQueue();
    InitCompass();
    // buoyId = 2;
    // memBuoyId(&buoyId, false);
    memBuoyId(&buoyId, GET);
    Serial.print("Buoy ID: ");
    Serial.println(buoyId);
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 1000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(EscTask, "EscTask", 2400, NULL, configMAX_PRIORITIES - 10, NULL, 1);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(CompassTask, "CompasaTask", 2000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    if (digitalRead(BUTTON_PIN) == true)
    {
        delay(100);
        if (digitalRead(BUTTON_PIN) == true)
        {
            wifiConfig = 1;
        }
    }
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8000, &wifiConfig, configMAX_PRIORITIES - 5, NULL, 0);
    /*
    Get sored settings
    */

    // mainData.ps = 20;
    // mainData.is = 0.1;
    // mainData.ds = 0;
    // pidSpeedParameters(mainData, SET);
    // mainData.pr = 0.5;
    // mainData.ir = 0.02;
    // mainData.dr = 0;
    // pidRudderParameters(mainData, SET);
    // mainData.minOfsetDist = 2;
    // mainData.maxOfsetDist = 8;
    // mainData.minSpeed = 0;
    // mainData.maxSpeed = 80;
    // computeParameters(mainData, SET);
    // mainData.minOfsetDist = 2;
    // mainData.maxOfsetDist = 8;
    // mainData.minSpeed = 0;
    // mainData.maxSpeed = 80;
    // computeParameters(mainData, SET);
    mainData = pidSpeedParameters(mainData, GET);
    mainData = pidRudderParameters(mainData, GET);
    mainData = computeParameters(mainData, GET);
    Serial.println("Compute Parameters Speed P:" + String(mainData.kps) +
                   " I:" + String(mainData.kis) +
                   " Rudder p:" + String(mainData.kpr) +
                   " I:" + String(mainData.kir) +
                   " MAxspeed:" + String(mainData.maxSpeed) +
                   " MINspeed:" + String(mainData.minSpeed) +
                   " Maxoffsetdistance:" + String(mainData.maxOfsetDist) +
                   " MINoffsetdistance:" + String(mainData.minOfsetDist));
    Serial.println("Setup done!");
}

void handelKeyPress(void)
{
    int presses = countKeyPressesWithTimeout();
    if (presses > 0)
    {
        Serial.print("Number of key presses: ");
        Serial.println(presses);
        if (presses == 5) // Calibrate compas north
        {
            beep(1, buzzer);
            calibrateNorthCompas();
            presses = -1;
        }
        if (presses == 10) // Calibrate compas
        {
            beep(1, buzzer);
            calibrateParametersCompas();
            presses = -1;
        }
        if (presses != -1)
        {
            beep(-1, buzzer);
        }
    }
}

void loop(void)
{
    String dataIn = "";
    String dataOut = "";
    int msg = -1;
    unsigned long nextSamp = millis();
    unsigned long accuSamp = millis();
    unsigned long logtimer = millis();
    float vbat = 0;
    int speedbb = 50, speedsb = 0;
    int presses = 0;
    beep(1000, buzzer);
    delay(500);
    mainLedStatus.color = CRGB::Blue;
    mainLedStatus.blink = BLINK_SLOW;
    xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
    mainPwrData.bb = CRGB::Black;
    mainPwrData.sb = CRGB::Black;
    mainPwrData.blinkBb = BLINK_OFF;
    mainPwrData.blinkSb = BLINK_OFF;
    xQueueSend(ledPwr, (void *)&mainPwrData, 10); // update util led

    /*
        Main loop
    */
    while (true)
    {
        if (xQueueReceive(compass, (void *)&mainData.dirMag, 2) == pdTRUE)
        {
            if (subStatus == TOPCALCRUDDER)
            {
                mainData = CalcRudderBuoy(mainData); // calculate power to thrusters
                if (esc.speedbb != mainData.speedBb || esc.speedsb != mainData.speedSb)
                {
                    esc.speedbb = mainData.speedBb;
                    esc.speedsb = mainData.speedSb;
                    xQueueSend(escspeed, (void *)&esc, 10);
                }
            }
        }
        handelKeyPress();
        // New data recieved on udp channel
        if (xQueueReceive(udpIn, (void *)&udpInMain, 0) == pdTRUE)
        {
            String in = String(udpInMain);
            if (verifyCRC(in))
            {
                mainData = RoboDecode(in, mainData);
                switch (mainData.cmd)
                {
                case TOPSPBBSPSB:
                    esc.speedbb = mainData.speedBb;
                    esc.speedsb = mainData.speedSb;
                    xQueueSend(escspeed, (void *)&esc, 10);
                    subStatus = TOPSPBBSPSB;
                    break;
                case TOPCALCRUDDER:
                    if (subStatus != TOPCALCRUDDER)
                    {
                        subStatus = TOPCALCRUDDER;
                        mainLedStatus.color = CRGB::Red;
                        mainLedStatus.blink = BLINK_OFF;
                        xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
                        mainPwrData.bb = CRGB::Black;
                        mainPwrData.sb = CRGB::Black;
                        mainPwrData.blinkBb = BLINK_OFF;
                        mainPwrData.blinkSb = BLINK_OFF;
                        xQueueSend(ledPwr, (void *)&mainPwrData, 10); // update util led
                    }
                    mainData = hooverPid(mainData);
                    break;
                case TOPIDLE:
                    if (subStatus != TOPIDLE)
                    {
                        subStatus = TOPIDLE;
                        esc.speedbb = 0;
                        esc.speedsb = 0;
                        xQueueSend(escspeed, (void *)&esc, 10);
                        beep(2, buzzer);
                        mainLedStatus.color = CRGB::Blue;
                        mainLedStatus.blink = BLINK_SLOW;
                        xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
                        delay(500);
                        mainPwrData.bb = CRGB::Black;
                        mainPwrData.sb = CRGB::Black;
                        mainPwrData.blinkBb = BLINK_OFF;
                        mainPwrData.blinkSb = BLINK_OFF;
                        xQueueSend(ledPwr, (void *)&mainPwrData, 10); // update util led
                    }
                    break;
                case PIDRUDDERSET:
                    pidRudderParameters(mainData, SET);
                    break;
                case PIDSPEEDSET:
                    pidSpeedParameters(mainData, SET);
                    break;
                case PIDRUDDER:
                    mainData.cmd = PIDRUDDER;
                    dataOut = RoboCode(mainData);
                    xQueueSend(udpOut, (void *)&mainData, 10); // update WiFi
                    break;
                case PIDSPEED:
                    mainData.cmd = PIDSPEED;
                    dataOut = RoboCode(mainData);
                    xQueueSend(udpOut, (void *)&mainData, 10); // update WiFi
                    break;
                case PING:
                    mainData.cmd = PONG;
                    xQueueSend(udpOut, (void *)&mainData, 10); // update WiFi
                    break;
                default:
                    printf("Unkonwn command %d\n\r", mainData.cmd);
                    break;
                }
            }
        }

        if (nextSamp < millis())
        {
            nextSamp = 500 + millis();
            mainData.cmd = SUBDIRSPEED;
            xQueueSend(udpOut, (void *)&mainData, 10); // update WiFi
        }
        if (accuSamp < millis())
        {
            accuSamp = 5000 + millis();
            mainData.cmd = SUBACCU;
            battVoltage(mainData.subAccuV, mainData.subAccuP);
            xQueueSend(udpOut, (void *)&mainData, 10); // update WiFi
        }
        if (logtimer < millis())
        {
            logtimer = millis() + 5000;
            if (subStatus == TOPCALCRUDDER)
            {
                printf("hdg:%6.2f dir%6.2f dist%5.2f speed:%4d ", mainData.dirMag, mainData.tgDir, mainData.tgDist, mainData.speed);
                // printf(" buoy.iintergrater:%f pr%f ir%f err%f", mainData.iintergrater, mainData.pr, mainData.ir,mainData.lastErrr);
                printf("Dir:%6.1f Speed:%4d BB:%2d SB:%2d is:%5.1f ir:%5.1f\r\n", smallestAngle(mainData.dirMag, mainData.tgDir), mainData.speed, mainData.speedBb, mainData.speedSb, mainData.is, mainData.ir);
            }
            else
            {
                printf("Hdg:%5.1f\r\n", mainData.dirMag);
                logtimer = millis() + 250;
            }
        }
    }
    vTaskDelay(1);
}
