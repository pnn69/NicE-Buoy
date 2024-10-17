#include <Arduino.h>
#include "main.h"
#include "io_top.h"
#include "robolora.h"
#include "leds.h"
#include "topwifi.h"
#include "gps.h"
#include "datastorage.h"
#include "buzzer.h"

static RoboStruct mainData;
static RoboStruct subData;
static LedData mainCollorStatus;
static LedData mainCollorUtil;
static LedData mainCollorGps;
static UdpData mainUdpOut;
static UdpData mainUdpIn;
static Buzz mainBuzzerData;
static GpsDataType mainGpsData;
static GpsNav mainNavData;
static int status;
static int wifiConfig = 0;
int8_t buoyId;
static int msg;
unsigned long udpTimer = millis();

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
    digitalWrite(BUTTON_LIGHT_PIN, !buttonState);
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

void handelKeyPress(void)
{
    int presses = countKeyPressesWithTimeout();
    switch (presses)
    {
    case 1: // lock / unlock
        if ((status != LOCKED) && (mainGpsData.fix == true))
        {
            mainNavData.tgLat = mainGpsData.lat;
            mainNavData.tgLon = mainGpsData.lon;
            status = LOCKED;
            beep(1);
        }
        else
        {
            status = IDLE;
            beep(2);
            mainData.cmd = TOPIDLE;
            xQueueSend(udpOut, (void *)&mainData, 0); // update WiFi
        }
        break;
    case 3:
        beep(1000);
        break;
    case 4:
        beep(1000);
        break;
    case 10:
        //Get doc position
        memDockPos(&mainNavData.tgLat, &mainNavData.tgLon, true);
        status = DOCKED;
        beep(1);
        break;
    case 15:
        // store currend location as doc location
        if (mainGpsData.fix == true)
        {
            memDockPos(&mainGpsData.lat, &mainGpsData.lon, false);
            beep(1);
        }
        else
        {
            beep(-1);
        }
    default:
        beep(-1);
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_LIGHT_PIN, OUTPUT);
    delay(100);
    printf("\r\nSetup running!\r\n");
    printf("Robobuoy Top Version: %0.1f\r\n", TOPVERSION);
    initMemory();
    // buoyId = 1;
    // memBuoyId(&buoyId, false);
    initbuzzerqueue();
    initledqueue();
    initwifiqueue();
    initgpsqueue();
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 1000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4024, NULL, 20, NULL, 1);
    xTaskCreatePinnedToCore(GpsTask, "GpsTask", 2000, NULL, configMAX_PRIORITIES - 8, NULL, 1);
    memBuoyId(&buoyId, true);
    Serial.print("Buoy ID: ");
    Serial.println(buoyId);
    if (digitalRead(BUTTON_PIN) == true)
    {
        delay(100);
        if (digitalRead(BUTTON_PIN) == true)
        {
            wifiConfig = 1; // put in to pair mode
        }
    }
    else
    {
        wifiConfig = 0; // Setup normal accespoint
    }
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 4000, &wifiConfig, configMAX_PRIORITIES - 5, NULL, 0);
    // mainCollorStatus.color = CRGB ::OrangeRed;
    // mainCollorStatus.blink = BLINK_OFF;
    // xQueueSend(ledStatus, (void *)&mainCollorStatus, 10); // update util led
    Serial.println("Main task running!");
}

void loop(void)
{
    int presses = -1;
    beep(1000);
    while (true)
    {
        handelKeyPress();
        if (udpTimer < millis())
        {
            udpTimer = millis() + 500;
            mainData.cmd = PING;
            xQueueSend(udpOut, (void *)&mainData, 0); // update WiFi
        }
        if (xQueueReceive(gpsQue, (void *)&mainGpsData, 0) == pdTRUE) // New gps data
        {
            if ((status == LOCKED) || status == DOCKED)
            {
                RouteToPoint(mainGpsData.lat, mainGpsData.lon, mainNavData.tgLat, mainNavData.tgLon, &mainNavData.tgDist, &mainNavData.tgDir);
                mainData.cmd = TOPDIRDIST;
                xQueueSend(udpOut, (void *)&mainData, 0); // update WiFi
            }
        }
        if (xQueueReceive(udpIn, (void *)&subData, 0) == pdTRUE)
        {
            Serial.println(String(subData.cmd));
            switch (subData.cmd)
            {
            case SUBDATA:
                mainData.subAccuV = subData.subAccuV;
                mainData.subAccuP = subData.subAccuP;
                mainData.dirMag = subData.dirMag;
                mainData.speedSb = subData.speedSb;
                mainData.speedBb = subData.speedBb;
                break;
            case SUBACCU:
                mainData.subAccuV = subData.subAccuV;
                mainData.subAccuP = subData.subAccuP;
                break;
            case SUBDIR:
                mainData.dirMag = subData.dirMag;
                break;
            case SUBDIRSPEED:
                mainData.dirMag = subData.dirMag;
                mainData.speedSb = subData.speedSb;
                mainData.speedBb = subData.speedBb;
                printf("Mag heading %d\r\n", mainData.dirMag);
                break;
            case PONG:
                printf("PONG\r\n");
                break;
            }
        }
    }
    delay(1);
}
