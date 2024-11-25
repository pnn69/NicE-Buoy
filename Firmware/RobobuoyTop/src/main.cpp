#include <Arduino.h>
#include <PID_v1.h>
#include "main.h"
#include "io_top.h"
#include "leds.h"
#include "topwifi.h"
#include "gps.h"
#include "datastorage.h"
#include "buzzer.h"
#include "adc.h"
#include "loratop.h"
#include "calibrate.h"

// #define BUFLENMHRG 60 // one sampel each sec so 60 sec for stabilisation
RoboStruct buoyPara[3] = {};
static RoboWindStruct wind;
static RoboStruct udpDataIn;
static RoboStruct LoraTx;
static RoboStruct loraRx;
static RoboStruct mainData;
static RoboStruct subData;
static RoboStruct loraData;
static LedData mainCollorStatus;
static LedData mainCollorUtil;
static LedData mainCollorGps;
static RoboStruct mainUdpOut;
static RoboStruct mainUdpIn;
static Buzz mainBuzzerData;
static GpsDataType mainGpsData;
static int wifiConfig = 0;
unsigned long buoyId;
static int msg;
static int presses = -1;
static int blink = BLINK_FAST;
static int loralstmsg = 0;
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
//  new pid stuff
//***************************************************************************************************
double Setpoint, Input, Output;
double Kp = 20, Ki = 0.05, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//***************************************************************************************************
//      Setup
//***************************************************************************************************
void setup()
{
    Serial.begin(115200);
    Setpoint = 0;
    myPID.SetMode(AUTOMATIC); // turn the PID on
    myPID.SetOutputLimits(-100, 100);

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
    mainData.mac = buoyId;
    mainData.IDs = buoyId;
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
    // defautls(mainData);
    mainData.status = IDLE;
    mainData.maxSpeed = 80;
}

//***************************************************************************************************
//  RudderPID
//***************************************************************************************************
RoboStruct speedPid(RoboStruct speed)
{
    Input = speed.tgDist - speed.minOfsetDist;
    Setpoint = 0;
    myPID.Compute();
    speed.speedSet = (int)Output;
    double Ki = myPID.GetKi();
    printf("tgDist:%.0f speedSet:%d Ki:%.4f\r\n", speed.tgDist - speed.minOfsetDist, speed.speedSet, Ki);
    return speed;
}

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
RoboStruct handelKeyPress(RoboStruct key)
{
    int presses = countKeyPressesWithTimeoutAndLongPressDetecton();
    if (presses > 0)
    {
        switch (presses)
        {
        case 1: // lock / unlock
            if ((key.status != LOCKED) && (key.status != DOCKED))
            {
                key.status = LOCKING;
            }
            else
            {
                key.status = IDELING;
            }
            break;
        case 2:
            key.status = COMPUTESTART;
            break;
        case 3:
            key.status = COMPUTETRACK;
            break;
        case 10:
            key.status = STOREASDOC;
            break;
        case 0x0100:
            // Sail to dock position
            key.status = DOCKING;
            break;
        default:
            beep(-1, buzzer);
            break;
        }
    }
    return key;
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
//  status actions
//***************************************************************************************************
RoboStruct handelStatus(RoboStruct stat, GpsDataType gps)
{
    LoraTx = stat;
    LoraTx.IDs = buoyId;
    LoraTx.IDr = BUOYIDALL;
    switch (stat.status)
    {
    case IDELING:
        beep(2, buzzer);
        stat.cmd = IDLE;
        xQueueSend(udpOut, (void *)&stat, 0); // update WiFi
        stat.status = IDLE;
        break;
    case LOCKING:
        if (gps.fix == true)
        {
            beep(1, buzzer);
            stat.mac = buoyId;
            stat.tgLat = stat.lat;
            stat.tgLng = stat.lng;
            AddDataToBuoyBase(stat, buoyPara); // store positon for later calculations Track positioning
            LoraTx = stat;
            // IDr,IDs,ACK,MSG,LAT,LON
            LoraTx.IDr = BUOYIDALL; // Send to all
            LoraTx.IDs = buoyId;    // msg is comming from me
            LoraTx.cmd = LOCKPOS;
            LoraTx.ack = LORASET;
            xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
            stat.status = LOCKED;
        }
        else
        {
            beep(2, buzzer);
            stat.status = IDLE;
        }
        break;

    case DOCKING:
        beep(1, buzzer);
        stat = memDockPos(stat, true);
        printf("Retreved data for docking tgLat:%.8f tgLng:%.8f\r\n", stat.tgLat, stat.tgLng);
        stat.status = DOCKED;
        break;

    case COMPUTESTART:
        buoyPara[0].wDir = stat.wDir;
        buoyPara[3] = calcTrackPos(buoyPara);
        for (int i = 0; i < 3; i++)
        {
            trackPosPrint(buoyPara[i].trackPos);
            printf(" = (%.12f,%.12f)\r\n", buoyPara[i].tgLat, buoyPara[i].tgLng);
        }
        buoyPara[3] = recalcStarLine(buoyPara);
        if ((buoyPara[0].trackPos != -1 && buoyPara[1].trackPos != -1) || (buoyPara[0].trackPos != -1 && buoyPara[2].trackPos != -1) || (buoyPara[1].trackPos != -1 && buoyPara[2].trackPos != -1))
        {
            beep(1, buzzer);
            stat.status = SENDTRACK;
            printf("#Send track info\r\n");
        }
        else
        {
            beep(-1, buzzer);
            stat.status = LOCKED;
        }
        break;
    case COMPUTETRACK:
        buoyPara[0].wDir = stat.wDir;
        for (int i = 0; i < 3; i++)
        {
            trackPosPrint(buoyPara[i].trackPos);
            printf(" = (%.12f,%.12f)\r\n", buoyPara[i].tgLat, buoyPara[i].tgLng);
        }
        buoyPara[3] = reCalcTrack(buoyPara);
        if (buoyPara[0].trackPos != -1 || buoyPara[1].trackPos != -1 || buoyPara[2].trackPos != -1)
        {
            beep(1, buzzer);
            stat.status = SENDTRACK;
        }
        else
        {
            beep(-1, buzzer);
            stat.status = LOCKED;
        }
        break;
    case SENDTRACK:
        for (int i = 0; i < 3; i++)
        {
            if (buoyPara[i].trackPos != 0 && buoyPara[i].IDs != 0 && buoyPara[i].IDs != buoyId)
            {
                memcpy(&LoraTx, &buoyPara[i], sizeof(RoboStruct));
                trackPosPrint(buoyPara[i].trackPos);
                printf("n = (%.12f,%.12f)\r\n", buoyPara[i].tgLat, buoyPara[i].tgLng);
                LoraTx.IDr = buoyPara[i].IDs;
                LoraTx.IDs = buoyId;
                LoraTx.cmd = SETLOCKPOS;
                LoraTx.ack = LORAGETACK;
                xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
            }
            if (buoyPara[i].IDs == buoyId)
            {
                stat.tgLat = buoyPara[i].tgLat;
                stat.tgLng = buoyPara[i].tgLng;
            }
        }
        stat.status = LOCKED;
        break;
    case CALIBRATE_OFFSET_MAGNETIC_COMPASS:
        stat = compasOffestCallicration(stat);
        break;

    default:
        break;
    }
    return stat;
}

//***************************************************************************************************
//  Timer routines
//***************************************************************************************************
RoboStruct handleTimerRoutines(RoboStruct timer)
{
    memcpy(&LoraTx, &timer, sizeof(RoboStruct));
    LoraTx.IDs = buoyId;
    LoraTx.IDr = BUOYIDALL;
    LoraTx.ack = LORAINF;

    if ((timer.status == LOCKED || timer.status == DOCKED) && timer.loralstmsg + 5000 < millis())
    {
        timer.loralstmsg = millis() + random(0, 150);
        LoraTx.cmd = BUOYPOS;
        xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
        LoraTx.cmd = SUBPWR;
        xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
    }

    // if (timer.lastLoraOut + 29000 + random(1000, 2000) < millis())
    if (timer.lastLoraOut + 2000 < millis())
    {
        timer.lastLoraOut = millis();
        LoraTx.cmd = BUOYPOS;
        LoraTx.ack = LORAINF;
        switch (loralstmsg)
        {
        case 1:
            if (timer.status != LOCKED || timer.status != DOCKED)
            {
                //  IDr,IDs,MSG,ACK,STATUS,LAT,LON.mDir,wDir,wStd,BattPecTop,BattPercBott,speedbb,speedsb
                LoraTx.cmd = BUOYPOS;
            }
            loralstmsg = 2;
            break;
        case 2:
            if (timer.status == LOCKED || timer.status == DOCKED)
            {
                //  IDr,IDs,MSG,ACK,tgDir,tgDist
                LoraTx.cmd = DIRDIST;
                loralstmsg = 3;
                break;
            }
        case 3:
        {
            LoraTx.IDr = BUOYIDALL;
            LoraTx.cmd = LOCKPOS;
            printf("tglat:%.8f tglng:%.8f \r\n",LoraTx.tgLat,LoraTx.tgLng);
            loralstmsg = 1;
        }
        break;
        default:
            loralstmsg = 1;
            break;
        }
        xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
    }

    if (timer.lastUdpOut + 250 < millis())
    {
        timer.lastUdpOut = millis();
        timer.cmd = PING;
        if (timer.status == LOCKED || timer.status == DOCKED)
        {
            RouteToPoint(timer.lat, timer.lng, timer.tgLat, timer.tgLng, &timer.tgDist, &timer.tgDir);
            timer = hooverPid(timer);
            timer.cmd = DIRSPEED;
        }
        if (timer.status == REMOTE) // Send course info
        {
            timer.cmd = DIRSPEED;
        }
        xQueueSend(udpOut, (void *)&timer, 0); // Keep the watchdog in sub happy
        timer.lastUdpOut = millis();
    }

    if (logtimer < millis())
    {
        logtimer = millis() + 1000;
        wind = addNewSampleInBuffer(wind, timer.dirMag); // add sample to buffer for wind direction calculation.
        wind = deviationWindRose(wind);
        timer.wStd = wind.wStd;
        timer.wDir = wind.wDir; // averige wind dir
        battVoltage(timer.topAccuV, timer.topAccuP);

        // RouteToPoint(timer.lat, timer.lng, timer.tgLat, timer.tgLng, &timer.tgDist, &timer.tgDir);
        // printf("Status:%d Lat: %.0f Lon:%.0f tgLat: %.0f tgLon:%.0f tgDist:%.2f tgDir:%.0f mDir:%.0f wDir:%.0f wStd:%.2f ", timer.status, timer.lat, timer.lng, timer.tgLat, timer.tgLng, timer.tgDist, timer.tgDir, timer.dirMag, timer.wDir, timer.wStd);
        // printf("Vtop: %1.1fV %3d%% Vsub: %1.1fV %d%% BB:%02d SB:%02d\r\n", timer.topAccuV, timer.topAccuP, timer.subAccuV, timer.subAccuP, timer.speedBb, timer.speedSb);
    }
    return timer;
}

//***************************************************************************************************
//      Main loop
//***************************************************************************************************
void loop(void)
{
    mainCollorGps.color = CRGB::DarkRed;
    mainCollorGps.blink = BLINK_FAST;
    xQueueSend(ledGps, (void *)&mainCollorGps, 0); // update GPS led
    mainCollorUtil.blink = BLINK_SLOW;
    mainCollorUtil.color = CRGB::DarkOrange;
    xQueueSend(ledUtil, (void *)&mainCollorUtil, 0); // update GPS led
    // beep(1000, buzzer);
    bool firstfix = false;
    mainData.lastLoraOut = millis();
    while (true)
    {
        //***************************************************************************************************
        //      Timer routines
        //***************************************************************************************************
        mainData = handleTimerRoutines(mainData);
        //***************************************************************************************************
        //      Check front key
        //***************************************************************************************************
        mainData = handelKeyPress(mainData);
        //***************************************************************************************************
        //      status actions
        //***************************************************************************************************
        mainData = handelStatus(mainData, mainGpsData);
        //***************************************************************************************************
        //      Light button control
        //***************************************************************************************************
        buttonLight(mainData.status, mainGpsData.fix);
        //****************************************************************************************************
        //      New GPS data
        //***************************************************************************************************
        if (xQueueReceive(gpsQue, (void *)&mainGpsData, 0) == pdTRUE) // New gps data
        {
            mainData.gpsLat = mainGpsData.lat;
            mainData.gpsLng = mainGpsData.lng;
            mainData.gpsFix = mainGpsData.fix;
            mainData.gpsSat = mainGpsData.nrsats;
            if (mainGpsData.fix == true && firstfix == false)
            {
                beep(2000, buzzer);
                firstfix = true;
                mainData.lat = mainGpsData.lat;
                mainData.lng = mainGpsData.lng;
                LoraTx.IDr = BUOYIDALL;
                LoraTx.cmd = BUOYPOS;
                LoraTx.ack = LORAINF;
                xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
            }
            if (mainGpsData.fix == true)
            {
                mainData.lat = mainGpsData.lat;
                mainData.lng = mainGpsData.lng;
                if ((mainData.status == LOCKED) || mainData.status == DOCKED) // update sub onley when locked or docked
                {
                    RouteToPoint(mainData.lat, mainData.lng, mainData.tgLat, mainData.tgLng, &mainData.tgDist, &mainData.tgDir);
                    mainData = hooverPid(mainData);
                    mainData.cmd = DIRSPEED;
                    mainData.IDs = buoyId;
                    xQueueSend(udpOut, (void *)&mainData, 0); // update WiFi
                    mainData.lastUdpOut = millis();
                }
            }
        }

        //***************************************************************************************************
        //      New Lora data
        //***************************************************************************************************
        if (xQueueReceive(loraIn, (void *)&loraRx, 1) == pdTRUE) // new lora data
        {
            if (loraRx.IDr == BUOYIDALL)
            {
                if (mainGpsData.fix == true && mainData.status != LOCKED && loraRx.cmd == LOCKING)
                {
                    printf("#Status set to LOCKING (by lora input)\r\n");
                    beep(1, buzzer);
                    mainData.mac = buoyId;
                    mainData.tgLat = mainData.lat;
                    mainData.tgLng = mainData.lng;
                    buoyPara[3] = AddDataToBuoyBase(mainData, buoyPara); // store positon for later calculations Track positioning
                                                                         // IDr,IDs,ACK,MSG,LAT,LON
                    LoraTx.IDr = BUOYIDALL;                              // Send to all
                    LoraTx.IDs = buoyId;                                 // msg is comming from me
                    LoraTx.cmd = LOCKPOS;
                    LoraTx.ack = LORASET;
                    mainData.status = LOCKED;
                }
            }
            if (loraRx.IDr == buoyId) // yes for me
            {

                if (loraRx.IDr == buoyId)
                {
                    mainData.lastLoraIn = millis(); // message was for me so update last lora tranmission.
                }
                loraData.mac = loraRx.IDs; // store ID
                switch (loraRx.cmd)
                {
                case LOCKING:
                    if (mainGpsData.fix == true && mainData.status != LOCKED)
                    {
                        printf("#Status set to LOCKING (by lora input)\r\n");
                        beep(1, buzzer);
                        mainData.mac = buoyId;
                        mainData.tgLat = mainData.lat;
                        mainData.tgLng = mainData.lng;
                        buoyPara[3] = AddDataToBuoyBase(mainData, buoyPara); // store positon for later calculations Track positioning
                                                                             // IDr,IDs,ACK,MSG,LAT,LON
                        LoraTx.IDr = BUOYIDALL;                              // Send to all
                        LoraTx.IDs = buoyId;                                 // msg is comming from me
                        LoraTx.cmd = LOCKPOS;
                        LoraTx.ack = LORASET;
                        mainData.status = LOCKED;
                    }
                    break;
                case DIRDIST:
                    mainData.tgDir = loraData.tgDir;
                    mainData.tgDist = loraData.tgDist;
                    mainData.status = DIRDIST;
                    loraTimerIn = 5000 + millis(); // 5 sec timeout
                case DIRSPEED:
                    mainData.tgDir = loraData.tgDir;
                    mainData.tgDist = loraData.speedSet;
                    mainData.status = REMOTE;
                    loraTimerIn = 5000 + millis(); // 5 sec timeout
                    break;
                case LOCKPOS: // store new data into position database
                    buoyPara[3] = AddDataToBuoyBase(loraRx, buoyPara);
                    break;
                case SETLOCKPOS: // store new data into position database
                    mainData.tgLat = loraData.tgLat;
                    mainData.tgLng = loraData.tgLng;
                    mainData.lat = loraData.tgLat;
                    mainData.lng = loraData.tgLng;
                    buoyPara[3] = AddDataToBuoyBase(loraRx, buoyPara);
                    mainData.status = LOCKING;
                    break;
                case DOCKING:
                    printf("#Status set to DOCKING (by lora input)\r\n");
                    mainData.status = DOCKING;
                    break;
                case IDELING:
                    printf("#Status set to IDLE (by lora input)\r\n");
                    mainData.status = IDELING;
                    break;
                case PIDRUDDERSET:
                    pidRudderParameters(loraData, false);
                    loraData.cmd = PIDRUDDERSET;
                    xQueueSend(udpOut, (void *)&loraData, 0); // Keep the watchdog in sub happy
                    mainData.pr = loraData.pr;
                    mainData.ir = loraData.ir;
                    mainData.dr = loraData.dr;
                    mainData.kir = 0;
                    break;
                case PIDSPEEDSET:
                    pidSpeedParameters(loraData, false);
                    loraData.cmd = PIDSPEEDSET;
                    xQueueSend(udpOut, (void *)&loraData, 0); // Keep the watchdog in sub happy
                    mainData.ps = loraData.ps;
                    mainData.is = loraData.is;
                    mainData.ds = loraData.ds;
                    mainData.kis = 0;
                    break;
                default:
                    break;
                }
            }
        }
        //***************************************************************************************************
        //      New WiFi data
        //***************************************************************************************************
        if (xQueueReceive(udpIn, (void *)&udpDataIn, 1) == pdTRUE)
        {
            mainData.lastUdpIn = millis();
            switch (udpDataIn.cmd)
            {
            case DIRSPEED:
                mainData.dirMag = udpDataIn.dirMag;
                mainData.speedBb = udpDataIn.speedBb;
                mainData.speedSb = udpDataIn.speedSb;
                break;
            case SUBACCU:
                mainData.subAccuV = udpDataIn.subAccuV;
                mainData.subAccuP = udpDataIn.subAccuP;
                break;
            default:
                break;
            }
        }
    }
}