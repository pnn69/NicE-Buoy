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
static char udpDataIn[MAXSTRINGLENG];
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
static int status = IDLE;
static int wifiConfig = 0;
unsigned long buoyId;
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
            if ((stat != LOCKED) && (stat != DOCKED))
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
        case 2:
            stat = COMPUTESTART;
            beep(1000, buzzer);
            break;
        case 3:
            stat = COMPUTETRACK;
            beep(1000, buzzer);
            break;
        case 6:
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
    buoyId = initwifiqueue();
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
    buoyPara[0].mac = 0;
    buoyPara[1].mac = 0;
    buoyPara[2].mac = 0;
    buoyPara[0].tgLat = 0;
    buoyPara[1].tgLat = 0;
    buoyPara[2].tgLat = 0;
    buoyPara[0].tgLng = 0;
    buoyPara[1].tgLng = 0;
    buoyPara[2].tgLng = 0;
    // buoyPara[1].tgLat = 52.42035583;
    // buoyPara[1].tgLng = 4.86552433;
    // buoyPara[2].tgLat = 52.62035583;
    // buoyPara[2].tgLng = 4.76552433;
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
        //***************************************************************************************************
        //      keypress check
        //***************************************************************************************************
        status = handelKeyPress(status);
        //***************************************************************************************************
        //      status actions
        //***************************************************************************************************
        switch (status)
        {
        case LOCKING:
            if (mainGpsData.fix == true)
            {
                beep(1, buzzer);
                mainData.mac = buoyId;
                mainData.tgLat = mainData.lat;
                mainData.tgLng = mainData.lng;
                AddDataToBuoyBase(mainData); // store positon for later calculations Track positioning
                // IDr,IDs,ACK,MSG,LAT,LON
                String loraString = RoboCode(mainData, LORALOCKPOS); // notify others
                LoraTx.macIDr = -2;                                  // Send to all
                LoraTx.macIDs = buoyId;                              // msg is comming from me
                LoraTx.msg = LORALOCKPOS;
                LoraTx.ack = LORASET;
                loraString.toCharArray(LoraTx.data, loraString.length() + 1);
                xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
                status = LOCKED;
            }
            else
            {
                beep(2, buzzer);
                status = IDLE;
            }
            break;

        case DOCKING:
            beep(1, buzzer);
            memDockPos(mainData, true);
            status = DOCKED;
            break;

        case COMPUTESTART:
            if (recalcStarLine(buoyPara) == true)
            {
                beep(1, buzzer);
                status = LORASENDDATA;
            }
            else
            {
                beep(-1, buzzer);
                status = LOCKED;
            }
            break;
        case COMPUTETRACK:
            if (reCalcTrack(buoyPara) == true)
            {
                beep(1, buzzer);
                status = LORASENDDATA;
            }
            else
            {
                beep(-1, buzzer);
                status = LOCKED;
            }
            break;
        case LORASENDDATA:
            printf("winddir = (%.2f)\r\n", buoyPara[0].wDir);
            posPrint(buoyPara[0].trackPos);
            printf("n = (%.12f,%.12f)\r\n", buoyPara[0].tgLat, buoyPara[0].tgLng);
            if (buoyPara[1].trackPos != -1 && buoyPara[1].trackPos != 0)
            {
                posPrint(buoyPara[1].trackPos);
                printf("n = (%.12f,%.12f)\r\n\r\n", buoyPara[1].tgLat, buoyPara[1].tgLng);
                // IDr,IDs,ACK,MSG,LAT,LON
                String loraString = RoboCode(buoyPara[1], LORALOCKPOS);
                LoraTx.macIDr = buoyPara[1].mac;
                LoraTx.macIDs = buoyId;
                LoraTx.msg = LORALOCKPOS;
                LoraTx.ack = LORAGETACK;
                loraString.toCharArray(LoraTx.data, loraString.length() + 1);
                xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
                mainData.tgLat = buoyPara[0].tgLat;
                mainData.tgLng = buoyPara[0].tgLng;
            }
            if (buoyPara[2].trackPos != -1 && buoyPara[2].trackPos != 0)
            {
                posPrint(buoyPara[2].trackPos);
                printf("n = (%.12f,%.12f)\r\n\r\n", buoyPara[1].tgLat, buoyPara[1].tgLng);
                String loraString = RoboCode(buoyPara[2], LORALOCKPOS);
                LoraTx.macIDr = buoyPara[2].mac;
                LoraTx.macIDs = buoyId;
                LoraTx.msg = LORALOCKPOS;
                LoraTx.ack = LORAGETACK;
                loraString.toCharArray(LoraTx.data, loraString.length() + 1);
                xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
                mainData.tgLat = buoyPara[0].tgLat;
                mainData.tgLng = buoyPara[0].tgLng;
            }
            printf("Track pos:%d\r\n", buoyPara[0].trackPos);
            status = LOCKED;
            break;
        default:
            break;
        }

        buttonLight(buttonBlinkTimer, blink, status, mainGpsData.fix);
        //****************************************************************************************************
        //      New GPS data
        //***************************************************************************************************
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
            if ((status == LOCKED) || status == DOCKED) // update sub onley when locked or docked
                if (postimer < millis())
                {
                    postimer = millis() + 250;
                    {
                        RouteToPoint(mainGpsData.lat, mainGpsData.lng, mainData.tgLat, mainData.tgLng, &mainData.tgDist, &mainData.tgDir);
                        mainData.cmd = TOPCALCRUDDER;
                        xQueueSend(udpOut, (void *)&mainData, 0); // update WiFi
                        udptimer = millis();
                    }
                }
        }
        //****************************************************************************************************
        //      New Lora data
        //***************************************************************************************************
        if (xQueueReceive(loraIn, (void *)&loraRx, 0) == pdTRUE) // new lora data
        {
            // MSG,<data>
            String loraStringIn = String(loraRx.data);
            loraData = RoboDecode(loraStringIn, loraData);
            loraData.mac = loraRx.macIDs; // store ID
            if (loraRx.macIDr == buoyId)  // yes i got a position update
            {
            }
            if (loraRx.msg == LORALOCKPOS)
            {
                if (AddDataToBuoyBase(loraData) == false)
                {
                    Serial.println("No data added");
                }
            }
            buoyPara[0].wDir = mainData.wDir;
        }
        //***************************************************************************************************
        //      New WiFi data
        //***************************************************************************************************
        if (xQueueReceive(udpIn, (void *)&udpDataIn, 0) == pdTRUE)
        {
            mainData = RoboDecode(udpDataIn, mainData);
        }
        //***************************************************************************************************
        //      Timer routines
        //***************************************************************************************************
        if (logtimer < millis())
        {
            logtimer = millis() + 1000;
            wind = addNewSampleInBuffer(wind, mainData.dirMag); // add sample to buffer for wind direction calculation.
            wind = deviationWindRose(wind);
            mainData.wStd = wind.wStd;
            mainData.wDir = wind.wDir; // averige wind dir

            RouteToPoint(mainData.lat, mainData.lng, mainData.tgLat, mainData.tgLng, &mainData.tgDist, &mainData.tgDir);
            printf("Status:%d Lat: %2.8f Lon:%2.8f tgLat: %2.8f tgLon:%2.8f tgDist:%.2f tgDir:%.2f mDir:%.2f wDir:%.2f wStd:%.2f ", status, mainData.lat, mainData.lng, mainData.tgLat, mainData.tgLng, mainData.tgDist, mainData.tgDir, mainData.dirMag, mainData.wDir, mainData.wStd);
            battVoltage(mainData.topAccuV, mainData.topAccuP);
            printf("Vtop: %1.1fV %3d%% Vsub: %1.1fV %3d%%\r\n", mainData.topAccuV, mainData.topAccuP, mainData.subAccuV, mainData.subAccuP);
        }
        if (loraTimer < millis())
        {
            loraTimer = millis() + 29000 + random(1000, 2000);
            // loraTimer = millis() + 9000 + random(1000, 2000);
            RoboStruct tmpLora;
            LoraTx.msg = -1;
            LoraTx.macIDr = -2;
            String loraString = "";
            LoraTx.macIDs = buoyId;
            LoraTx.ack = LORAINF;
            switch (mainData.loralstmsg)
            {
            case 1:
                //  IDr,IDs,MSG,ACK,STATUS,LAT,LON.mDir,wDir,wStd,BattPecTop,BattPercBott,speedbb,speedsb
                LoraTx.msg = LORABUOYPOS;
                tmpLora.cmd = LORABUOYPOS;
                loraString = RoboCode(mainData, LORABUOYPOS);
                mainData.loralstmsg = 2;
                break;
            case 2:
                //  IDr,IDs,MSG,ACK,tgDir,tgDist
                LoraTx.msg = LORADIRDIST;
                tmpLora.cmd = LORADIRDIST;
                loraString = RoboCode(mainData, LORADIRDIST);
                mainData.loralstmsg = 3;
                break;
            case 3:
                //  IDr,IDs,MSG,ACK,tgLat,tgLng
                LoraTx.msg = LORALOCKPOS;
                tmpLora.cmd = LORALOCKPOS;
                loraString = RoboCode(mainData, LORALOCKPOS);
                mainData.loralstmsg = 1;
                break;
            default:
                mainData.loralstmsg = 1;
                break;
            }
            if (loraString.length() < MAXSTRINGLENG - 1 && LoraTx.msg != -1) // send only if msg is set
            {
                loraString.toCharArray(LoraTx.data, loraString.length() + 1);
                xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
            }
            else
            {
                Serial.println("Lora error case:"+ String(mainData.loralstmsg) + " cmd:" + String(tmpLora.cmd) + "  Data:" + loraString + "  String length:" + loraString.length());
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