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
unsigned long buoyIdAll = BUOYIDALL;
static int msg;
static int presses = -1;
static int blink = BLINK_FAST;
static unsigned long loraTimerOut = millis();
static unsigned long loraTimerIn = millis();
static unsigned long buttonBlinkTimer = millis();
static unsigned long logtimer = millis();
static unsigned long remotetimer = millis();
static unsigned long updateSubtimer = millis();

int buttonState = 0;             // Current state of the button
int lastButtonState = 0;         // Previous state of the button
unsigned long lastPressTime = 0; // Time of the last press
unsigned long debounceDelay = 0; // Debounce time in milliseconds
int pressCount = 0;              // Count the number of button presses
bool debounce = false;           // Debouncing flag

//***************************************************************************************************
//      keypress detection
//***************************************************************************************************
int countKeyPressesWithTimeoutAndLongPressDetecton()
{
    // Get the current time
    unsigned long currentTime = millis();
    if (currentTime < debounceDelay)
    {
        return -1; // debounce
    }
    buttonState = digitalRead(BUTTON_PIN);
    // Check if the button is pressed and it's a new press (debounce)
    if (buttonState == HIGH && lastButtonState == LOW && !debounce)
    {
        pressCount++;                     // Increment the button press count
        debounce = true;                  // Set debounce flag
        beep(10, buzzer);                 // short high pitch beep
        debounceDelay = currentTime + 50; // Simple debouncing by adding a delay
        lastPressTime = currentTime;      // Record the time of the last press
    }
    // Reset debounce flag if the button is released
    if (buttonState == LOW)
    {
        debounce = false;
    }
    // If more than 2 seconds have passed without a press, return the count
    if ((currentTime - lastPressTime) > 500 && pressCount > 0 && buttonState == LOW)
    {
        if (pressCount == 0x100) // previous detecion was a long press
        {
            pressCount = 0; // Reset the press count after returning
            return -1;      // Return -1 if 0.5 seconds haven't passed yet
        }
        int finalPressCount = pressCount; // Store the current press count
        pressCount = 0;                   // Reset the press count after returning
        return finalPressCount;           // Return the number of key presses
    }
    else if ((currentTime - lastPressTime) > 3000 && pressCount == 1 && buttonState == HIGH)
    {

        pressCount = 0x0100;
        return 0X100;
    }
    lastButtonState = buttonState; // Save the last button state
    return -1;                     // Return -1 if 0.5 seconds haven't passed yet
}

//***************************************************************************************************
//      key press stuff
//***************************************************************************************************
int handelKeyPress(int stat)
{
    int presses = countKeyPressesWithTimeoutAndLongPressDetecton();
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
                stat = IDELING;
            }
            break;
        case 2:
            stat = COMPUTESTART;
            break;
        case 3:
            stat = COMPUTETRACK;
            break;
        case 10:
            stat = STOREASDOC;
            break;
        case 0x0100:
            // Sail to dock position
            stat = DOCKING;
            break;
        default:
            beep(-1, buzzer);
            break;
        }
    }
    return stat;
}

//***************************************************************************************************
//      Light button control
//***************************************************************************************************
void buttonLight(int status, bool fix)
{
    if (buttonBlinkTimer < millis())
    {
        buttonBlinkTimer = millis() + blink;
        if (status == LOCKED || status == DOCKED)
        {
            digitalWrite(BUTTON_LIGHT_PIN, LOW);
            blink = 2000;
            mainCollorGps.color = CRGB::Green;
            mainCollorGps.blink = BLINK_OFF;
            xQueueSend(ledGps, (void *)&mainCollorGps, 0); // update led
        }
        else
        {
            digitalWrite(BUTTON_LIGHT_PIN, !digitalRead(BUTTON_LIGHT_PIN));
            if (fix == true)
            {
                if (blink != 1000)
                {
                    mainCollorGps.color = CRGB::Green;
                    mainCollorGps.blink = BLINK_SLOW;
                    xQueueSend(ledGps, (void *)&mainCollorGps, 0); // update WiFi
                    blink = 1000;
                }
            }
            else
            {
                if (blink != 100)
                {
                    mainCollorGps.color = CRGB::DarkRed;
                    mainCollorGps.blink = BLINK_FAST;
                    xQueueSend(ledGps, (void *)&mainCollorGps, 0); // update WiFi
                    blink = 100;
                }
            }
        }
    }
}

//***************************************************************************************************
//      status actions
//***************************************************************************************************
int handelStatus(int status)
{
    switch (status)
    {
    case IDELING:
        beep(2, buzzer);
        mainData.cmd = IDLE;
        xQueueSend(udpOut, (void *)&mainData, 0); // update WiFi
        status = IDLE;
        break;
    case LOCKING:
        if (mainGpsData.fix == true)
        {
            beep(1, buzzer);
            mainData.mac = buoyId;
            mainData.tgLat = mainData.lat;
            mainData.tgLng = mainData.lng;
            AddDataToBuoyBase(mainData, buoyPara); // store positon for later calculations Track positioning
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
        buoyPara[3] = calcTrackPos(buoyPara);
        printf("\r\nwinddir = (%.2f)\r\n", buoyPara[0].wDir);
        trackPosPrint(buoyPara[0].trackPos);
        printf(" = (%.12f,%.12f)\r\n", buoyPara[0].tgLat, buoyPara[0].tgLng);
        trackPosPrint(buoyPara[1].trackPos);
        printf(" = (%.12f,%.12f)\r\n", buoyPara[1].tgLat, buoyPara[1].tgLng);
        trackPosPrint(buoyPara[2].trackPos);
        printf(" = (%.12f,%.12f)\r\n", buoyPara[2].tgLat, buoyPara[2].tgLng);
        if (recalcStarLine(buoyPara) == true)
        {
            beep(1, buzzer);
            status = LORASENDTRACK;
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
            status = LORASENDTRACK;
        }
        else
        {
            beep(-1, buzzer);
            status = LOCKED;
        }
        break;
    case LORASENDTRACK:
        if (buoyPara[0].trackPos != -1 && buoyPara[1].trackPos != 0)
        {
            trackPosPrint(buoyPara[0].trackPos);
            printf("n = (%.12f,%.12f)\r\n\r\n", buoyPara[0].tgLat, buoyPara[0].tgLng);
            String loraString = RoboCode(buoyPara[0], LORALOCKPOS);
            LoraTx.macIDr = buoyPara[0].mac;
            LoraTx.macIDs = buoyId;
            LoraTx.msg = LORALOCKPOS;
            LoraTx.ack = LORAGETACK;
            loraString.toCharArray(LoraTx.data, loraString.length() + 1);
            xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
        }

        if (buoyPara[1].trackPos != -1 && buoyPara[1].trackPos != 0)
        {
            trackPosPrint(buoyPara[1].trackPos);
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
            trackPosPrint(buoyPara[2].trackPos);
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
        status = LOCKED;
        break;
    default:
        break;
    }
    return status;
}

//***************************************************************************************************
//      Timer routines
//***************************************************************************************************
void handleTimerRoutines(RoboStruct timer)
{
    if (updateSubtimer < millis())
    {
        updateSubtimer = 250 + millis();
        // updateSubtimer = 10000 + millis();

        timer.cmd = PING;
        if (timer.status == LOCKED || timer.status == DOCKED || timer.status == REMOTE) // Send course info
        {
            timer.cmd = status;
        }
        if (timer.status == IDLE)
        {
            timer.cmd = timer.status;
            timer.speed = 0;
            timer.dirSet = 0;
            timer.tgDir = 0;
            timer.tgDist = 0;
        }
        xQueueSend(udpOut, (void *)&timer, 0); // Keep the watchdog in sub happy
    }

    if (loraTimerOut < millis())
    {
        loraTimerOut = millis() + 29000 + random(1000, 2000);
        // loraTimerOut= millis() + 9000 + random(1000, 2000);
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
            if (status == LOCKED || status == DOCKED)
            {
                //  IDr,IDs,MSG,ACK,tgDir,tgDist
                LoraTx.msg = LORADIRDIST;
                tmpLora.cmd = LORADIRDIST;
                loraString = RoboCode(mainData, LORADIRDIST);
            }
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
            Serial.println("Lora error case:" + String(mainData.loralstmsg) + " cmd:" + String(tmpLora.cmd) + "  Data:" + loraString + "  String length:" + loraString.length());
        }
    }

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
}

//***************************************************************************************************
//      Setup
//***************************************************************************************************
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
    buoyPara[0].tgLat = 0;
    buoyPara[0].tgLng = 0;
    buoyPara[1].mac = 0;
    buoyPara[1].tgLat = 0;
    buoyPara[1].tgLng = 0;
    // buoyPara[2].mac = 0;
    // buoyPara[2].tgLat = 0;
    // buoyPara[2].tgLng = 0;
    buoyPara[2].mac = 80;
    buoyPara[2].tgLat = 52.32287446097399;
    buoyPara[2].tgLng = 4.9638194546392445;
}

//***************************************************************************************************
//      Main loop
//***************************************************************************************************
void loop(void)
{
    mainCollorGps.color = CRGB::DarkRed;
    mainCollorGps.blink = BLINK_FAST;
    xQueueSend(ledGps, (void *)&mainCollorGps, 0); // update GPS led
    // beep(1000, buzzer);
    bool firstfix = false;
    while (true)
    {
        //***************************************************************************************************
        //      Timer routines
        //***************************************************************************************************
        handleTimerRoutines(mainData);
        //***************************************************************************************************
        //      Check front key
        //***************************************************************************************************
        status = handelKeyPress(status);
        //***************************************************************************************************
        //      status actions
        //***************************************************************************************************
        status = handelStatus(status);
        //***************************************************************************************************
        //      Light button control
        //***************************************************************************************************
        buttonLight(status, mainGpsData.fix);
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
            {
                RouteToPoint(mainGpsData.lat, mainGpsData.lng, mainData.tgLat, mainData.tgLng, &mainData.tgDist, &mainData.tgDir);
                mainData.cmd = TOPCALCRUDDER;
                xQueueSend(udpOut, (void *)&mainData, 0); // update WiFi
            }
        }
        //***************************************************************************************************
        //      New Lora data
        //***************************************************************************************************
        if (xQueueReceive(loraIn, (void *)&loraRx, 0) == pdTRUE) // new lora data
        {
            if (loraRx.macIDr == buoyId || loraRx.macIDr == BUOYIDALL) // yes for me
            {
                // MSG,<data>
                String loraStringIn = String(loraRx.data);
                loraData = RoboDecode(loraStringIn, loraData);
                loraData.mac = loraRx.macIDs; // store ID
                switch (loraRx.msg)
                {
                case IDELING:
                    if (mainGpsData.fix == true)
                    {
                        beep(1, buzzer);
                        mainData.mac = buoyId;
                        mainData.tgLat = mainData.lat;
                        mainData.tgLng = mainData.lng;
                        AddDataToBuoyBase(mainData, buoyPara); // store positon for later calculations Track positioning
                        // IDr,IDs,ACK,MSG,LAT,LON
                        String loraString = RoboCode(mainData, LORALOCKPOS); // notify others
                        LoraTx.macIDr = BUOYIDALL;                           // Send to all
                        LoraTx.macIDs = buoyId;                              // msg is comming from me
                        LoraTx.msg = LORALOCKPOS;
                        LoraTx.ack = LORASET;
                        loraString.toCharArray(LoraTx.data, loraString.length() + 1);
                        xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
                        status = LOCKED;
                    }
                    break;
                case LORADIRDIST:
                    mainData.tgDir = loraData.tgDir;
                    mainData.tgDist = loraData.tgDist;
                    status = LORADIRDIST;
                    loraTimerIn = 5000 + millis(); // 5 sec timeout
                case LORADIRSPEED:
                    mainData.tgDir = loraData.tgDir;
                    mainData.tgDist = loraData.speedSet;
                    status = REMOTE;
                    loraTimerIn = 5000 + millis(); // 5 sec timeout
                    break;
                case LORALOCKPOS:
                    loraData.lastLoraComm = millis();
                    AddDataToBuoyBase(loraData, buoyPara);
                    break;
                case DOCKING:
                    printf("Status set to DOCKING (by lora input)\r\n");
                    status = DOCKING;
                    break;
                case LOCKING:
                    printf("Status set to LOCKING (by lora input)\r\n");
                        status = LOCKING;
                    break;
                default:
                    break;
                }
            }
        }
        else
        {
            if (loraTimerIn < millis() && status == REMOTE) // wdg remote contolled status
            {
                status = IDLE; // stop if there are no updates for more than 5 seconds
            }
        }
        //***************************************************************************************************
        //      New WiFi data
        //***************************************************************************************************
        if (xQueueReceive(udpIn, (void *)&udpDataIn, 0) == pdTRUE)
        {
            mainData = RoboDecode(udpDataIn, mainData);
        }
    }
}