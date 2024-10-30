#include <Arduino.h>
#include "main.h"
#include "io_top.h"
#include "leds.h"
#include "topwifi.h"
#include "gps.h"
#include "datastorage.h"
#include "buzzer.h"
#include "adc.h"
#include "loratop.h"
#include "roboRaceTrack.h"

// #define BUFLENMHRG 60 // one sampel each sec so 60 sec for stabilisation
RoboStruct buoyPara[3] = {};
static RoboWindStruct wind;
static lorabuf LoraTx;
static lorabuf loraRx;
static RoboStruct mainData;
static RoboStruct subData;
static RoboStruct loraData;
static LedData mainCollorStatus;
static LedData mainCollorUtil;
static LedData mainCollorGps;
static UdpData mainUdpOut;
static UdpData mainUdpIn;
static Buzz mainBuzzerData;
static GpsDataType mainGpsData;
static GpsNav mainNavData;
static int status = IDLE;
static int wifiConfig = 0;
uint64_t buoyId;
static int msg;
unsigned long udpTimer = millis();
unsigned long loraTimer = millis();

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
            if ((stat != LOCKED) || (stat != DOCKED))
            {
                stat = LOCKING;
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
            stat = COMPUTESTART;
            beep(1000, buzzer);
            break;
        case 6:
            stat = COMPUTETRACK;
            beep(1000, buzzer);
            break;
        case 10:
            // Get doc position
            stat = DOCKING;
            break;
        case 15:
            stat = STOREASDOC;
            break;
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
            xQueueSend(ledGps, (void *)&mainCollorGps, 0); // update led
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
    xTaskCreatePinnedToCore(GpsTask, "GpsTask", 2000, NULL, configMAX_PRIORITIES - 8, NULL, 1);
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
    // 52.32035583 Lon:4.96552433
    buoyPara[0].id = 0Xec64c99779b8;
    buoyPara[0].tgLat = 52.0;
    buoyPara[0].tgLng = 4.0;
    buoyPara[1].id = 4524;
    buoyPara[1].tgLat = 52.42035583;
    buoyPara[1].tgLng = 4.86552433;
    buoyPara[2].id = 5283;
    buoyPara[2].tgLat = 52.62035583;
    buoyPara[2].tgLng = 4.76552433;
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
    unsigned long udptimer = millis();
    // beep(1000, buzzer);
    /*
        Main loop
    */
    bool firstfix = false;
    while (true)
    {
        status = handelKeyPress(status);
        if (status == LOCKING)
        {
            if (mainGpsData.fix == true)
            {
                beep(1, buzzer);
                mainData.tgLat = mainData.lat;
                mainData.tgLng = mainData.lng;
                AddDataToBuoyBase(mainData); // store positon for later calculations Track positioning
                status = LOCKED;
            }
            else
            {
                beep(2, buzzer);
                status = IDLE;
            }
        }
        if (status == DOCKING)
        {
            beep(1, buzzer);
            memDockPos(mainData, true);
            status = DOCKED;
        }
        if (status == COMPUTESTART)
        {
            // recalcStarLine(double winddir, double *tgLat1, double *tgLng1, double *tgLat2, double *tgLng2)
            // send loradata
            status = LOCKED;
        }
        if (status == COMPUTETRACK)
        {
            reCalcTrack(buoyPara);
            // send loradata
            status = LOCKED;
        }
        buttonLight(buttonBlinkTimer, blink, status, mainGpsData.fix);
        /*
        New GPS datat
        */
        if (xQueueReceive(gpsQue, (void *)&mainGpsData, 0) == pdTRUE) // New gps data
        {
            if (mainGpsData.fix == true && firstfix == false)
            {
                beep(2000, buzzer);
                firstfix = true;
            }
            if (mainGpsData.fix == true)
            {
                mainData.lat = mainGpsData.lat;
                mainData.lng = mainGpsData.lng;
            }
            if ((status == LOCKED) || status == DOCKED)
                if (postimer < millis())
                {
                    postimer = millis() + 250;
                    {
                        RouteToPoint(mainGpsData.lat, mainGpsData.lng, mainNavData.tgLat, mainNavData.tgLon, &mainData.tgDist, &mainData.tgDir);
                        mainData.cmd = TOPCALCRUDDER;
                        xQueueSend(udpOut, (void *)&mainData, 0); // update WiFi
                        udptimer = millis();
                    }
                }
        }
        /*
        New Lora data
        */
        if (xQueueReceive(loraIn, (void *)&loraRx, 0) == pdTRUE) // new lora data
        {
            loraData.id = loraRx.id;
            Serial.println("LORA data in: " + String(loraRx.id, HEX) + "," + String(loraRx.data));
            String data = addBeginAndEndToString(String(loraRx.data));
            loraData = RoboDecode(data, loraData);
            //  ID,MSG,ACK,STATUS,LAT,LON.mDir,wDir,wStd,BattPecTop,BattPercBott,speedbb,speedsb
            if (loraData.msg == LORALOCKPOS)
            {
                if (AddDataToBuoyBase(loraData) == false)
                {
                    Serial.println("No data added");
                }
            }
            buoyPara[0].wDir = mainData.wDir;
        }
        /*
        New data from udp
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
                Serial.print("ID=");
                Serial.println(subData.id, HEX);
                mainData.id = subData.id;
                buoyId = subData.id;
                Serial.println("Id set to:" + String((uint64_t)subData.id, HEX));
                break;
            default:
                printf("Command %d not supported!\r\n", subData.cmd);
            }
        }
        if (logtimer < millis())
        {
            logtimer = millis() + 1000;
            wind = addNewSampleInBuffer(wind, mainData.dirMag); // add sample to buffer for wind direction calculation.
            wind = deviationWindRose(wind);
            mainData.wStd = wind.wStd;
            mainData.wDir = wind.wDir; // averige wind dir

            RouteToPoint(mainGpsData.lat, mainGpsData.lng, mainNavData.tgLat, mainNavData.tgLon, &mainNavData.tgDist, &mainNavData.tgDir);
            printf("Lat: %2.8f Lon:%2.8f tgLat: %2.8f tgLon:%2.8f tgDist:%.2f tgDir:%.2f mDir:%.2f wDir:%.2f wStd:%.2f ", mainGpsData.lat, mainGpsData.lng, mainNavData.tgLat, mainNavData.tgLon, mainData.tgDist, mainData.tgDir, mainData.dirMag, mainData.wDir, mainData.wStd);
            battVoltage(mainData.topAccuV, mainData.topAccuP);
            printf("Vtop: %1.1fV %3d%% Vsub: %1.1fV %3d%%\r\n", mainData.topAccuV, mainData.topAccuP, mainData.subAccuV, mainData.subAccuP);
        }
        if (loraTimer < millis())
        {
            loraTimer = millis() + 9000 + random(1000, 2000);
            String loraString = String(mainData.id, HEX);
            if (mainData.lstmsg != LORABUOYPOS)
            {
                mainData.lstmsg = LORABUOYPOS;
                //  ID,MSG,ACK,STATUS,LAT,LON.mDir,wDir,wStd,BattPecTop,BattPercBott,speedbb,speedsb
                loraString +=
                    "," + String(LORABUOYPOS) +
                    "," + String(LORAUPD) +
                    "," + String(status) +
                    "," + String(mainData.lat, 8) +
                    "," + String(mainData.lng, 8) +
                    "," + String(mainData.dirMag, 0) +
                    "," + String(mainData.wDir, 0) +
                    "," + String(mainData.wStd, 2) +
                    "," + String(mainData.topAccuP) +
                    "," + String(mainData.subAccuP) +
                    "," + String(mainData.speedBb) +
                    "," + String(mainData.speedSb);
            }
            else
            {
                mainData.lstmsg = LORALOCKPOS;
                //  ID,MSG,ACK,STATUS,LAT,LON
                loraString +=
                    "," + String(LORALOCKPOS) +
                    "," + String(LORAUPD) +
                    "," + String(status) +
                    "," + String(mainData.lat, 8) +
                    "," + String(mainData.lng, 8);
            }
            if (loraString.length() < MAXSTRINGLENG - 1)
            {
                Serial.println( "Lora msg out:" + loraString);
                loraString.toCharArray(LoraTx.data, loraString.length() + 1);
                LoraTx.id = mainData.id;
                LoraTx.ack = LORAUPD;
                LoraTx.msg = mainData.lstmsg;
                xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
            }
            else
            {
                printf("Lora string to long: %d\r\n", loraString.length());
            }
        }
        if (status == IDLE)
        {
            if (udptimer + 1000 < millis())
            {
                udptimer = millis();
                mainData.cmd = PING;
                xQueueSend(udpOut, (void *)&mainData, 0); // Keep the watchdog in sub happy
            }
        }
        if (status == LOCKED || status == DOCKED) // Keep the watchdog in sub happy
        {
            if (udptimer + 1000 < millis())
            {
                mainData.cmd = PING;
                udptimer = millis();
            }
        }
    }
}
