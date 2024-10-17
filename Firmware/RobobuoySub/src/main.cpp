#include <Arduino.h>
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

static RoboStruct mainData;
Message esc;
static int8_t buoyId;
static int subStatus = TOPIDLE;
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
    InitCompass();
    // buoyId = 2;
    // memBuoyId(&buoyId, false);
    memBuoyId(&buoyId, true);
    Serial.print("Buoy ID: ");
    Serial.println(buoyId);
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 1000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(EscTask, "EscTask", 2400, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    if (digitalRead(BUTTON_PIN) == true)
    {
        delay(100);
        if (digitalRead(BUTTON_PIN) == true)
        {
            wifiConfig = 1;
        }
    }
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 4000, &wifiConfig, configMAX_PRIORITIES - 2, NULL, 0);
}

void handelKeyPress(int presses)
{
    Serial.print("Number of key presses: ");
    Serial.println(presses);
    if (presses == 5) // Calibrate compas north
    {
        beep(1);
        calibrateNorthCompas();
        presses = -1;
    }
    if (presses == 10) // Calibrate compas
    {
        beep(-1);
        calibrateParametersCompas();
        presses = -1;
    }
    if (presses != -1)
    {
        beep(false);
    }
}

void loop(void)
{
    int msg = -1;
    unsigned long nextSamp = millis();
    unsigned long accuSamp = millis();
    float vbat = 0;
    int speedbb = 50, speedsb = 0;
    int presses = 0;
    beep(1000);

    /*
        Main loop
    */
    while (true)
    {
        float tmp = GetHeadingAvg();
        printf("Magheading:%0.2f", tmp);
        mainData.dirMag = (int)tmp;
        // mainData.dirMag = (int)GetHeadingAvg();
        presses = countKeyPressesWithTimeout();
        if (presses >= 0)
        {
            handelKeyPress(presses);
        }
        // New data recieved on udb channel
        if (xQueueReceive(udpIn, (void *)&mainData, 0) == pdTRUE)
        {
            switch (mainData.cmd)
            {
            case TOPSPBBSPSB:
                esc.speedbb = mainData.speedBb;
                esc.speedsb = mainData.speedSb;
                xQueueSend(escspeed, (void *)&esc, 10);
                subStatus = TOPSPBBSPSB;
                break;
            case TOPDIRSPEED:
                CalcRudderBuoy(mainData.dirMag, mainData.tgDir, mainData.tgDist, mainData.speedSet, &mainData.speedBb, &mainData.speedSb); // calculate power to thrusters
                esc.speedbb = mainData.speedBb;
                esc.speedsb = mainData.speedSb;
                xQueueSend(escspeed, (void *)&esc, 10);
                subStatus = TOPDIRSPEED;
                break;
            case TOPIDLE:
                esc.speedbb = 0;
                esc.speedsb = 0;
                xQueueSend(escspeed, (void *)&esc, 10);
                subStatus = TOPIDLE;
                beep(-1);
                break;
            case PING:
                printf("PING\r\n");
                mainData.cmd = PONG;
                xQueueSend(udpOut, (void *)&mainData, 10); // update WiFi
                break;
            }
        }

        if (nextSamp < millis())
        {
            nextSamp = 250 + millis();
            mainData.cmd = SUBDIRSPEED;
            xQueueSend(udpOut, (void *)&mainData, 10); // update WiFi
            printf("Heding= %03.2f\r\n", mainData.dirMag);
        }
        if (accuSamp < millis())
        {
            accuSamp = 5000 + millis();
            if (udpOut != NULL && uxQueueSpacesAvailable(udpOut) > 0)
            {
                mainData.cmd = SUBACCU;
                xQueueSend(udpOut, (void *)&mainData, 10); // update WiFi
            }
        }
        if (subStatus == TOPDIRSPEED)
        {
            CalcRudderBuoy(mainData.dirMag, mainData.tgDir, mainData.tgDist, mainData.speedSet, &mainData.speedBb, &mainData.speedSb); // calculate power to thrusters
            if (esc.speedbb != mainData.speedBb || esc.speedsb != mainData.speedSb)
            {
                esc.speedbb = mainData.speedBb;
                esc.speedsb = mainData.speedSb;
                xQueueSend(escspeed, (void *)&esc, 10);
            }
        }
    }
    vTaskDelay(1);
}
