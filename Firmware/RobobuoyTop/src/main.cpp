#include <Arduino.h>
#include <RoboTone.h>
#include "main.h"
#include "io_top.h"
#include "leds.h"
#include "topwifi.h"
#include "gps.h"
#include "datastorage.h"
#include "buzzer.h"
#include "adc.h"
#include "loratop.h"

#define BUFLENMHRG 60 // one sampel each sec so 60 sec for stabilisation

static extBuoyParameters extBuoyData[2]; // max 2 other buoys
static char loraTransmitt[MAXSTRINGLENG];
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
static unsigned int status = IDLE;
static int wifiConfig = 0;
int8_t buoyId;
static int msg;
unsigned long udpTimer = millis();
double winddir[3 + BUFLENMHRG]; // winddir[0]=avarage winddir[1]=pionter winddir[2]=standarddeviation winddir[3..BUFLENWINDSPEED + 3)]=data

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

int handelKeyPress(int stat)
{
    int presses = countKeyPressesWithTimeout();
    if (presses > 0)
    {
        switch (presses)
        {
        case 1: // lock / unlock
            if ((stat != LOCKED) && (mainGpsData.fix == true))
            {
                mainNavData.tgLat = mainGpsData.lat;
                mainNavData.tgLon = mainGpsData.lon;
                stat = LOCKED;
                beep(1, buzzer);
            }
            else
            {
                stat = IDLE;
                beep(2, buzzer);
                mainData.cmd = TOPIDLE;
                xQueueSend(udpOut, (void *)&mainData, 0); // update WiFi
            }
            break;
        case 3:
            // compute start line
            beep(1000, buzzer);
            break;
        case 10:
            // Get doc position
            memDockPos(&mainNavData.tgLat, &mainNavData.tgLon, true);
            stat = DOCKED;
            beep(1, buzzer);
            break;
        case 15:
            // store currend location as doc location
            if (mainGpsData.fix == true)
            {
                memDockPos(&mainGpsData.lat, &mainGpsData.lon, false);
                beep(1, buzzer);
            }
            else
            {
                beep(-1, buzzer);
            }
        default:
            beep(-1, buzzer);
            break;
        }
    }
    return stat;
}

void buttonLight(unsigned long &timer, int &bl, int status, bool f)
{
    if (timer < millis())
    {
        timer = millis() + bl;
        if (status == LOCKED || status == DOCKED)
        {
            digitalWrite(BUTTON_LIGHT_PIN, LOW);
            bl = 2000;
            mainCollorGps.color = CRGB::Green;
            mainCollorGps.blink = BLINK_OFF;
            xQueueSend(ledGps, (void *)&mainCollorGps, 0); // update WiFi
        }
        else
        {
            digitalWrite(BUTTON_LIGHT_PIN, !digitalRead(BUTTON_LIGHT_PIN));
            if (f == true)
            {
                if (bl != 1000)
                {
                    mainCollorGps.color = CRGB::Green;
                    mainCollorGps.blink = BLINK_SLOW;
                    xQueueSend(ledGps, (void *)&mainCollorGps, 0); // update WiFi
                    bl = 1000;
                }
            }
            else
            {
                if (bl != 100)
                {
                    mainCollorGps.color = CRGB::DarkRed;
                    mainCollorGps.blink = BLINK_FAST;
                    xQueueSend(ledGps, (void *)&mainCollorGps, 0); // update WiFi
                    bl = 100;
                }
            }
        }
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
    initbuzzerqueue();
    initledqueue();
    initwifiqueue();
    initgpsqueue();
    initloraqueue();

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
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 4000, &wifiConfig, configMAX_PRIORITIES - 2, NULL, 0);
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    Serial.println("Main task running!");
    if (extBuoyData[0].mac == 0)
    {
        printf("No data recieved from other buoy yet\r\n");
    }
}

void loop(void)
{
    int presses = -1;
    int blink = BLINK_FAST;
    double lastLat, lastLon;
    mainCollorGps.color = CRGB::DarkRed;
    mainCollorGps.blink = BLINK_FAST;
    xQueueSend(ledGps, (void *)&mainCollorGps, 0); // update GPS led
    unsigned long buttonBlinkTimer = millis();
    unsigned long logtimer = millis();
    unsigned long postimer = millis();
    beep(1000, buzzer);
    /*
        Main loop
    */
    bool firstfix = false;
    while (true)
    {
        status = handelKeyPress(status);
        buttonLight(buttonBlinkTimer, blink, status, mainGpsData.fix);
        /*
        New GPS datat
        */
        if (xQueueReceive(gpsQue, (void *)&mainGpsData, 0) == pdTRUE) // New gps data
        {
            if (mainGpsData.fix == true && firstfix == false)
            {
                // beep(2000, buzzer);
                firstfix = true;
            }
            if ((status == LOCKED) || status == DOCKED)
                if (postimer < millis())
                {
                    postimer = millis() + 250;
                    {
                        RouteToPoint(mainGpsData.lat, mainGpsData.lon, mainNavData.tgLat, mainNavData.tgLon, &mainData.tgDist, &mainData.tgDir);
                        mainData.cmd = TOPCALCRUDDER;
                        xQueueSend(udpOut, (void *)&mainData, 0); // update WiFi
                    }
                }
        }
        /*
        New sub data
        */
        if (xQueueReceive(udpIn, (void *)&subData, 0) == pdTRUE)
        {
            switch (subData.cmd)
            {
            case SUBACCU:
                mainData.subAccuV = subData.subAccuV;
                mainData.subAccuP = subData.subAccuP;
                // printf("Accu %2.1f Perc %d%%\r\n", mainData.subAccuV, mainData.subAccuP);
                break;
            case SUBDIR:
                mainData.dirMag = subData.dirMag;
                // printf("Mag heading %d\r\n", mainData.dirMag, mainData.speedBb, mainData.speedSb);
                break;
            case SUBDIRSPEED:
                mainData.dirMag = subData.dirMag;
                mainData.speedSb = subData.speedSb;
                mainData.speedBb = subData.speedBb;
                // printf("Mag heading %d bb:%d%% sb:%d%%\r\n", mainData.dirMag, mainData.speedBb, mainData.speedSb);
                break;
            case PONG:
                break;
            case TOPID:
                mainData.id = subData.id;
                //printf("Buoy-ID:%012llx\n\r", mainData.id);
                Serial.println("Lora-ID:" + String(mainData.id, HEX));
                break;
            default:
                printf("Command %d not supported!\r\n", subData.cmd);
            }
        }
        if (logtimer < millis())
        {
            logtimer = millis() + 1000;
            // RouteToPoint(mainGpsData.lat, mainGpsData.lon, mainNavData.tgLat, mainNavData.tgLon, &mainNavData.tgDist, &mainNavData.tgDir);
            printf("Lat: %2.8f Lon:%2.8f tgLat: %2.8f tgLon:%2.8f tgDist:%2f tgDir:%2f\r\n", mainGpsData.lat, mainGpsData.lon, mainNavData.tgLat, mainNavData.tgLon, mainData.tgDist, mainData.tgDir);
            battVoltage(mainData.topAccuV, mainData.topAccuP);
            printf("Vtop: %1.1fV %3d%% Vsub: %1.1fV %3d%%\r\n", mainData.topAccuV, mainData.topAccuP, mainData.subAccuV, mainData.subAccuP);
            if (status == LOCKED)
            {
                addNewSampleInBuffer(winddir, BUFLENMHRG, mainData.dirMag);
            }
            // LORABUOYPOS     // ID,MSG,STATUS,LAT,LON.mDir,wDir,wStd,BattPecTop,BattPercBott
            String loraString = String(mainData.id,HEX) + "," + String(LORABUOYPOS) + "," + String(status) + "," + String(mainData.lat) + "," + String(mainData.lng) + "," + String(mainData.dirMag) + "," + String(mainData.dirMag) + "," + String(mainData.wDir) + "," + String(mainData.wStd) + "," + String(mainData.topAccuP) + "," + String(mainData.subAccuP) + "," + String(mainData.speedSet);
            if (loraString.length() < MAXSTRINGLENG-1)
            {
                loraString.toCharArray(loraTransmitt,loraString.length() + 1);
                xQueueSend(loraOut, (void *)&loraTransmitt, 10); // send out trough Lora
            }
            else
            {
                printf("Lora string to long: %d\r\n", loraString.length());
            }
        }
    }
}
