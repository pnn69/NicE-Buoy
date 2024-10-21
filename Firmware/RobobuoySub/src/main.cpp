#include <Arduino.h>
#include <RoboTone.h>
#include <RoboCalc.h>
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
Message esc;
static int8_t buoyId;
static int subStatus = IDLE;
static UdpData mainUdpOutMsg;
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
    initGpsQueue();
    InitCompass();
    // buoyId = 2;
    // memBuoyId(&buoyId, false);
    memBuoyId(&buoyId, true);
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
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 4000, &wifiConfig, configMAX_PRIORITIES - 5, NULL, 0);
    // String setupPid = String(PIDRUDDERSET) + ",20,0.1,0";
    speed.p = 20;
    speed.i = 0.1;
    speed.d = 0;
    speed.minOfsetDist = 2;
    speed.maxOfsetDist = 2000;
    speed.maxSpeed = 80;
    speed.minSpeed = 0;
    rudder.p = 0.5;
    rudder.i = 0.02;
    rudder.d = 0;
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
    double mDir;
    beep(1000, buzzer);

    /*
        Main loop
    */
    while (true)
    {
        xQueueReceive(compass, (void *)&mDir, 0);
        handelKeyPress();
        // New data recieved on udp channel
        if (xQueueReceive(udpIn, (void *)&dataIn, (TickType_t)10) == pdTRUE)
        {
            Serial.println("Data in main1: " + dataIn);
            if (verifyCRC(dataIn))
            {
                Serial.println("Data in main2: " + dataIn);
                int msg = RoboDecode(dataIn, mainData);
                switch (mainData.cmd)
                {
                case TOPSPBBSPSB:
                    esc.speedbb = mainData.speedBb;
                    esc.speedsb = mainData.speedSb;
                    xQueueSend(escspeed, (void *)&esc, 10);
                    subStatus = TOPSPBBSPSB;
                    break;
                case TOPCALCRUDDER:
                    subStatus = TOPCALCRUDDER;
                    break;
                case TOPIDLE:
                    esc.speedbb = 0;
                    esc.speedsb = 0;
                    xQueueSend(escspeed, (void *)&esc, 10);
                    subStatus = TOPIDLE;
                    beep(2, buzzer);
                    break;
                case PIDRUDDERSET:
                    rudder.p = mainData.p;
                    rudder.i = mainData.i;
                    rudder.d = mainData.d;
                    rudder.kp = 0;
                    rudder.ki = 0;
                    rudder.kd = 0;
                    pidRudderParameters(&rudder.p, &rudder.i, &rudder.d, false);
                    break;
                case PIDRUDDER:
                    mainData.cmd = PIDRUDDER;
                    mainData.p = rudder.p;
                    mainData.i = rudder.i;
                    mainData.d = rudder.d;
                    mainData.kp = rudder.kp;
                    mainData.ki = rudder.ki;
                    mainData.kd = rudder.kd;
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
            nextSamp = 1100 + millis();
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
        if (subStatus == TOPCALCRUDDER)
        {
            mainData.speedSet = hooverPid(mainData.tgDist, speed);
            printf("hdg:%3f dir%3d dist%2.0f speedset:%d\r\n", mDir, mainData.tgDir, mainData.tgDist, mainData.speedSet);
            CalcRudderBuoy(mDir, (double)mainData.tgDir, mainData.tgDist, mainData.speedSet, &mainData.speedBb, &mainData.speedSb, rudder); // calculate power to thrusters
            if (esc.speedbb != mainData.speedBb || esc.speedsb != mainData.speedSb)
            {
                esc.speedbb = mainData.speedBb;
                esc.speedsb = mainData.speedSb;
                xQueueSend(escspeed, (void *)&esc, 10);
            }
            printf("Dir:%d Speed:%d BB:%d SB:%d\r\n", mainData.tgDir, mainData.speedSet, esc.speedbb, esc.speedsb);
            subStatus = 0;
        }
        if (logtimer < millis())
        {
            logtimer = millis() + 5000;
            printf("Hdg:%3f Speed:%d BB:%3d SB:%3d\r\n", mDir, mainData.speedSet, esc.speedbb, esc.speedsb);
        }
    }
    vTaskDelay(1);
}
