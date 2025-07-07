#include <Arduino.h>
#include "esp_log.h"
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
#include "sercom.h"
#include "calibrate.h"

// #define BUFLENMHRG 60 // one sampel each sec so 60 sec for stabilisation
static RoboStruct mainData;
// RoboStruct b0, b1, b2;
// RoboStruct *buoyPara[4] = {&b0, &b1, &b2};
RoboStruct buoyPara[3] = {};
RoboStruct *buoyParaPtrs[3] = {&buoyPara[0], &buoyPara[1], &buoyPara[2]};
static RoboStruct mainUdpIn;
static RoboWindStruct wind;
static LedData mainCollorStatus;
static LedData mainCollorUtil;
static LedData mainCollorGps;
static Buzz mainBuzzerData;
static int wifiConfig = 0;
static int msg;
static int presses = -1;
static int blink = BLINK_FAST;
static int loralstmsg = 0;
static unsigned long loraTimerOut = millis();
static unsigned long loraTimerIn = millis();
static unsigned long udpTimerOut = millis();
static unsigned long buttonBlinkTimer = millis();
static unsigned long logtimer = millis();
static unsigned long remotetimer = millis();
static unsigned long updateSubtimer = millis();

bool debounce = false;
bool buttonState = false;
bool lastButtonState = false;
bool longPressReported = false;
unsigned long lastPressTime = 0;
unsigned long debounceDelay = 0;
int pressCount = 0;

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
    initwifiqueue();
    initgpsqueue();
    initloraqueue();
    initserqueue();
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
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 4000, &wifiConfig, configMAX_PRIORITIES - 10, NULL, 0);
    xTaskCreatePinnedToCore(SercomTask, "SerialTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    // tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
    // mainData.tgLat = 52.29308075283747;
    // mainData.tgLng = 4.932570409845357;
    // memDockPos(&mainData, SET);
    // mainData.tgLat = 0;
    // mainData.tgLng = 0;
    // memDockPos(&mainData, GET);
    // Serial.println("Dock position set to: ");
    // Serial.printf("Lat: %.8lf, Lng: %.8lf\r\n", mainData.tgLat, mainData.tgLng);
    Serial.println("Main task running!");
    // defautls(mainData);
}

//***************************************************************************************************
//      keypress detection
//***************************************************************************************************
#define LONG_PRESS_DURATION 5000 // ms
#define PRESS_TIMEOUT 500        // ms
int countKeyPressesWithTimeoutAndFinalLongPress()
{
    unsigned long currentTime = millis();
    int buttonState = digitalRead(BUTTON_PIN);
    // Debounce
    if (currentTime < debounceDelay)
        return -1;

    // Start of a new press
    if (buttonState == HIGH && lastButtonState == LOW)
    {
        lastPressTime = currentTime;
        debounce = true;
        longPressReported = false;
        beep(2000, buzzer);
    }

    // Long press detection
    if (buttonState == HIGH && (currentTime - lastPressTime > 3000) && !longPressReported)
    {
        beep(2000, buzzer);
        int result = 101 + pressCount; // 100 + short press count
        pressCount = 0;
        longPressReported = true;
        return result;
    }

    // Count short presses on release
    if (buttonState == LOW && lastButtonState == HIGH && debounce)
    {
        if (!longPressReported)
        {
            pressCount++;
        }
        debounce = false;
        debounceDelay = currentTime + 50;
    }

    // Timeout for short press sequence (500 ms)
    if ((currentTime - lastPressTime) > 500 && pressCount > 0 && buttonState == LOW && !longPressReported)
    {
        int result = pressCount;
        pressCount = 0;
        return result;
    }
    lastButtonState = buttonState;
    return -1;
}

//***************************************************************************************************
//      Short, Short, Short	3
//      Short, Short, Long	103
//      Short, Long, Short	1 (resets after invalid long in middle, or could be ignored)
//      Long only	0
//      Short, Short, Short, Long	104
//      key press stuff
//      One press: lock/unlock
//      Two press: start line computation
//      Three press: compute track
//      Five press: sail to dock position
//      Four short presses and one long: store as docposition
//      nine short presses and onle long: start calibration of magnetic compass
//***************************************************************************************************
void handelKeyPress(RoboStruct *key)
{
    int presses = countKeyPressesWithTimeoutAndFinalLongPress();
    if (presses > 0)
    {
        switch (presses)
        {
        case 1: // lock / unlock
            if ((key->status != LOCKED) && (key->status != DOCKED))
            {
                key->status = LOCKING;
            }
            else
            {
                key->status = IDELING;
            }
            key->loralstmsg = 0;
            break;
        case 2:
            key->status = COMPUTESTART;
            break;
        case 3:
            key->status = COMPUTETRACK;
            break;
        case 5:
            key->status = DOCKING;
            break;
        case 105:
            key->status = STOREASDOC;
            break;
        case 10:
            key->status = SET_DECLINATION;
            break;
        case 110:
            key->status = START_CALIBRATE_MAGNETIC_COMPASS;
            break;
        default:
            beep(-1, buzzer);
            break;
        }
    }
}

//***************************************************************************************************
//      Light button control
//***************************************************************************************************
void buttonLight(RoboStruct sta)
{
    if (buttonBlinkTimer < millis())
    {
        buttonBlinkTimer = millis() + blink;
        if (sta.status == LOCKED || sta.status == DOCKED)
        {
            digitalWrite(BUTTON_LIGHT_PIN, HIGH);
            blink = 2000;
        }
        else if (sta.status == CALIBRATE_MAGNETIC_COMPASS)
        {
            digitalWrite(BUTTON_LIGHT_PIN, !digitalRead(BUTTON_LIGHT_PIN));
            blink = 50;
        }
        else
        {
            digitalWrite(BUTTON_LIGHT_PIN, !digitalRead(BUTTON_LIGHT_PIN));
            if (sta.gpsFix == true)
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
void handelStatus(RoboStruct *stat, RoboStruct buoyPara[3])
{
    RoboStruct LoraTx;
    stat->IDs = stat->buoyId;
    LoraTx = *stat;
    stat->IDr = BUOYIDALL;
    switch (stat->status)
    {
    case IDELING:
        beep(2, buzzer);
        stat->cmd = IDLE;
        stat->status = IDLE;
        xQueueSend(udpOut, (void *)stat, 0);  // update WiFi
        xQueueSend(loraOut, (void *)stat, 0); // update Lora
        stat->ack = LORAGETACK;
        xQueueSend(serOut, (void *)stat, 20); // update sub
        break;
    case LOCKING:
        if (stat->gpsFix == true)
        {
            beep(1, buzzer);
            stat->status = LOCKED;
            stat->cmd = LOCKPOS;
            stat->ack = LORASET;
            stat->tgLat = stat->lat;
            stat->tgLng = stat->lng;
            AddDataToBuoyBase(*stat, buoyParaPtrs); // store positon for later calculations Track positioning
            // IDr,IDs,ACK,MSG,LAT,LON
            xQueueSend(udpOut, (void *)stat, 0);   // update WiFi
            xQueueSend(loraOut, (void *)stat, 10); // send out trough Lora
            RouteToPoint(stat->lat, stat->lng, stat->tgLat, stat->tgLng, &stat->tgDist, &stat->tgDir);
            stat->cmd = DIRDIST;
            xQueueSend(serOut, (void *)stat, 0);  // send course and distance to sub
            xQueueSend(loraOut, (void *)stat, 0); // send course and distance to sub
        }
        else
        {
            beep(2, buzzer);
            stat->status = IDLE;
        }
        break;

    case DOCKING:
        beep(1, buzzer);
        memDockPos(stat, GET);
        stat->status = DOCKED;
        printf("Retreved data for docking tgLat:%.8f tgLng:%.8f\r\n", stat->tgLat, stat->tgLng);
        RouteToPoint(stat->lat, stat->lng, stat->tgLat, stat->tgLng, &stat->tgDist, &stat->tgDir);
        stat->cmd = DIRDIST;
        xQueueSend(serOut, (void *)stat, 0);  // send course and distance to sub
        xQueueSend(loraOut, (void *)stat, 0); // send course and distance to sub
        break;

    case COMPUTESTART:
        buoyPara[0].wDir = stat->wDir;
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
            stat->status = SENDTRACK;
            printf("#Send track info\r\n");
        }
        else
        {
            beep(-1, buzzer);
            stat->status = LOCKED;
        }
        break;
    case COMPUTETRACK:
        buoyPara[0].wDir = stat->wDir;
        for (int i = 0; i < 3; i++)
        {
            trackPosPrint(buoyPara[i].trackPos);
            printf(" = (%.12f,%.12f)\r\n", buoyPara[i].tgLat, buoyPara[i].tgLng);
        }
        buoyPara[3] = reCalcTrack(buoyPara);
        if (buoyPara[0].trackPos != -1 || buoyPara[1].trackPos != -1 || buoyPara[2].trackPos != -1)
        {
            beep(1, buzzer);
            stat->status = SENDTRACK;
        }
        else
        {
            beep(-1, buzzer);
            stat->status = LOCKED;
        }
        break;
    case SENDTRACK:
        for (int i = 0; i < 3; i++)
        {
            if (buoyPara[i].trackPos != 0 && buoyPara[i].IDs != 0 && buoyPara[i].IDs != stat->buoyId)
            {
                memcpy(&LoraTx, &buoyPara[i], sizeof(RoboStruct));
                trackPosPrint(buoyPara[i].trackPos);
                printf("n = (%.12f,%.12f)\r\n", buoyPara[i].tgLat, buoyPara[i].tgLng);
                LoraTx.IDr = buoyPara[i].IDs;
                LoraTx.IDs = stat->buoyId;
                LoraTx.cmd = SETLOCKPOS;
                LoraTx.ack = LORAGETACK;
                xQueueSend(loraOut, (void *)&LoraTx, 10); // send out trough Lora
                xQueueSend(udpOut, (void *)&LoraTx, 10);  // send out trough WiFi
            }
            if (buoyPara[i].IDs == stat->mac)
            {
                stat->tgLat = buoyPara[i].tgLat;
                stat->tgLng = buoyPara[i].tgLng;
            }
        }
        stat->status = LOCKED;
        break;
    case START_CALIBRATE_MAGNETIC_COMPASS:
        LoraTx.cmd = CALIBRATE_MAGNETIC_COMPASS;
        LoraTx.ack = LORAGETACK;
        xQueueSend(serOut, (void *)&LoraTx, 10); // send out trough Lora
        stat->status = CALIBRATE_MAGNETIC_COMPASS;
        break;
    case SET_DECLINATION:
        LoraTx.cmd = SET_DECLINATION;
        LoraTx.ack = LORAGETACK;
        xQueueSend(serOut, (void *)&LoraTx, 10); // send out trough Lora
        break;
    case STOREASDOC:
        if (stat->gpsFix == true)
        {
            printf("Storing docpositoin\r\n");
            memDockPos(stat, SET);
            beep(1000, buzzer);
        }
        else
        {
            beep(-1, buzzer);
        }
        stat->status = IDELING;
        break;
    default:
        break;
    }
}

//***************************************************************************************************
//  Timer routines
//***************************************************************************************************
void handleTimerRoutines(RoboStruct *timer)
{
    timer->IDs = timer->mac;
    timer->IDr = BUOYIDALL;
    timer->ack = LORAINF;
    //***************************************************************************************************
    // sub data out
    //***************************************************************************************************
    if (timer->lastSerOut < millis())
    {
        timer->lastSerOut = millis() + 5000;
        if (timer->status == LOCKED || timer->status == DOCKED)
        {
            timer->lastSerOut = millis() + 1000;
            RouteToPoint(timer->lat, timer->lng, timer->tgLat, timer->tgLng, &timer->tgDist, &timer->tgDir);
            timer->cmd = DIRDIST;
            xQueueSend(serOut, (void *)timer, 0);  // send course and distance to sub
            xQueueSend(loraOut, (void *)timer, 0); // send course and distance to sub
            timer->cmd = DIRMDIRTGDIRG;
            xQueueSend(loraOut, (void *)timer, 0); // send course and distance to sub
            xQueueSend(udpOut, (void *)timer, 0);  // send course and distance to sub
        }
        else if (timer->status == REMOTE) // Remote controlled
        {
            timer->lastSerOut = millis() + 750;
            timer->cmd = REMOTE;
            xQueueSend(serOut, (void *)timer, 0);
            timer->cmd = DIRSPEED;
            xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
            // xQueueSend(udpOut, (void *)timer, 10);  // send out trough wifi
        }
        else
        {
            timer->cmd = PING;
        }
    }
    //***************************************************************************************************
    // RF data out
    //***************************************************************************************************
    if (timer->loralstmsg < millis())
    {
        if (timer->status == IDLE)
        {
            timer->loralstmsg = millis() + 5000 + random(0, 150);
        }
        else
        {
            timer->loralstmsg = millis() + 1000 + random(0, 150);
        }
        timer->cmd = SUBPWR;
        xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
        xQueueSend(udpOut, (void *)timer, 10);  // send out trough wifi
        timer->cmd = DIRSPEED;
        xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
        xQueueSend(udpOut, (void *)timer, 10);  // send out trough wifi

        if ((timer->status == LOCKED || timer->status == DOCKED))
        {
            timer->cmd = LOCKPOS;
            xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
            xQueueSend(udpOut, (void *)timer, 10);  // send out trough wifi
            //  IDr,IDs,MSG,ACK,tgDir,tgDist
            timer->cmd = DIRDIST;
            xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
            xQueueSend(udpOut, (void *)timer, 10);
            timer->cmd = WINDDATA;
            xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
            xQueueSend(udpOut, (void *)timer, 10);
        }
        if (timer->status == REMOTE)
        {
            timer->cmd = REMOTE;
            xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
            xQueueSend(udpOut, (void *)timer, 10);
        }
    }

    if (logtimer < millis())
    {
        logtimer = millis() + 1000 + random(0, 150);
        battVoltage(timer->topAccuV, timer->topAccuP);
        addNewSampleInBuffer(&wind, timer->dirMag); // add sample to buffer for wind direction calculation.
        deviationWindRose(&wind);
        timer->wStd = wind.wStd;
        timer->wDir = wind.wDir; // averige wind dir

        // RouteToPoint(timer->lat, timer->lng, timer->tgLat, timer->tgLng, &timer->tgDist, &timer->tgDir);
        // printf("Status:%d Lat: %.0f Lon:%.0f tgLat: %.0f tgLon:%.0f tgDist:%.2f tgDir:%.0f mDir:%.0f wDir:%.0f wStd:%.2f ", timer->status, timer->lat, timer->lng, timer->tgLat, timer->tgLng, timer->tgDist, timer->tgDir, timer->dirMag, timer->wDir, timer->wStd);
        // printf("Vtop: %1.1fV %3d%% Vsub: %1.1fV %d%% BB:%02d SB:%02d\r\n", timer->topAccuV, timer->topAccuP, timer->subAccuV, timer->subAccuP, timer->speedBb, timer->speedSb);
    }
    if (udpTimerOut + 5000 < millis())
    {
        udpTimerOut = millis() + random(0, 150);
        timer->cmd = BUOYPOS;
        xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
        xQueueSend(udpOut, (void *)timer, 10);  // send out trough wifi
    }
}

// ***************************************************************************************************
// handle external data input
// ***************************************************************************************************
void handelRfData(RoboStruct *RfOut, RoboStruct *buoyPara[3])
{
    RoboStruct RfIn;
    RfIn.IDr = -1;
    if (xQueueReceive(loraIn, (void *)&RfIn, 1) == pdTRUE) // new lora data
    {
    }
    else if (xQueueReceive(udpIn, (void *)&RfIn, 1) == pdTRUE) // new lora data
    {
    }

    if (RfIn.IDr != -1)
    {
        RfOut->IDs = RfOut->mac;
        RfOut->IDr = RfIn.IDs;
        RfOut->ack = LORAINF;
        RfOut->cmd = RfIn.cmd;
        if (RfIn.IDr == RfOut->mac || RfIn.IDr == BUOYIDALL) // yes for me
        {
            switch (RfIn.cmd)
            {
            case DOCKING:
                if (RfOut->status != DOCKING || RfOut->status != DOCKED)
                {
                    printf("#Status set to DOCKING\r\n");
                    RfOut->status = DOCKING;
                }
                break;
            case LOCKPOS: // store new data into position database
                AddDataToBuoyBase(RfIn, &buoyPara[3]);
                break;
            case PIDRUDDER:
                if (RfIn.ack == LORAGET)
                {
                    xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                }
                else
                {
                    xQueueSend(loraOut, (void *)&RfIn, 0); // update sub
                    xQueueSend(udpOut, (void *)&RfIn, 0);  // update sub
                }
                break;
            case PIDRUDDERSET:
                pidRudderParameters(RfIn, SET);
                printf("#PIDRUDDERSET: %05.2f %05.2f %05.2f\r\n", RfIn.pr, RfIn.ir, RfIn.dr);
                RfIn.ack = LORAGETACK;
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                break;
            case PIDSPEED:
                if (RfIn.ack == LORAGET)
                {
                    xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                }
                else
                {
                    xQueueSend(loraOut, (void *)&RfIn, 0); // update sub
                    xQueueSend(udpOut, (void *)&RfIn, 0);  // update sub
                }
            case PIDSPEEDSET:
                pidSpeedParameters(RfIn, SET);
                printf("#PIDSPEEDSET: %05.2f %05.2f %05.2f\r\n", RfIn.ps, RfIn.is, RfIn.ds);
                RfOut->cmd = PIDSPEEDSET;
                RfIn.ack = LORAGETACK;
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                break;
            case DIRSPEED:
                RfOut->tgDir = RfIn.tgDir;
                RfOut->speedSet = RfIn.speedSet;
                RfOut->status = DIRSPEED;
                break;
            case REMOTE:
                RfOut->status = RfIn.cmd;
                RfOut->tgDir = RfIn.tgDir;
                RfOut->tgSpeed = RfIn.tgSpeed;
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                RfOut->status = RfIn.cmd;
                break;
            case DIRDIST:
                double lat, lon;
                printf("New dir: %f New distance %f\r\n", RfIn.tgDir, RfIn.tgDist);
                adjustPositionDirDist(RfIn.tgDir, RfIn.tgDist, RfOut->lat, RfOut->lng, &RfOut->tgLat, &RfOut->tgLng);
                printf("New cordinates: https://www.google.nl/maps/@%f,%f,14z\r\n", RfOut->tgLat, RfOut->tgLng);
                RouteToPoint(RfOut->lat, RfOut->lng, RfOut->tgLat, RfOut->tgLng, &RfOut->tgDist, &RfOut->tgDir);
                printf("New dir: %f New distance %f\r\n", RfOut->tgDir, RfOut->tgDist);
                if (RfOut->status != LOCKED)
                {
                    RfOut->status = LOCKING;
                }
                break;
            case LOCKING:
                if (RfOut->gpsFix == true && RfOut->status != LOCKING)
                {
                    RfOut->status = LOCKING;
                    beep(1, buzzer);
                }
                break;
            case DOCKPOS: // Get the positon to dock
                memDockPos(RfOut, GET);
            case STOREASDOC: // Store location as doc location
                if (RfOut->gpsFix == true)
                {
                    memDockPos(RfOut, SET);
                    beep(1000, buzzer);
                }
                else
                {
                    beep(-1, buzzer);
                }
                break;
            case SETLOCKPOS: // store new data into position database and sail to it
                RfOut->tgLat = RfIn.tgLat;
                RfOut->tgLng = RfIn.tgLng;
                RfIn.IDs = RfOut->mac; // Put this Id in field for positioning
                AddDataToBuoyBase(RfIn, &buoyPara[3]);
                RfOut->status = LOCKED;
                break;
            case IDELING:
            case IDLE:
                if (RfOut->status != IDELING || RfOut->status != IDLE)
                {
                    printf("#Status set to IDLE (by lora input)\r\n");
                    RfOut->status = IDELING;
                }
                break;
            case SUBACCU:
                RfOut->subAccuV = RfIn.subAccuV;
                RfOut->subAccuP = RfIn.subAccuP;
                break;
            case SUBPWR:
                RfOut->speedBb = RfIn.speedBb;   // set speed for bow
                RfOut->speedSb = RfIn.speedSb;   // set speed for stern
                RfOut->subAccuV = RfIn.subAccuV; // set sub accu voltage
                break;
            case STORE_DECLINATION:
                printf("Declinaton set to: %f\r\n", RfIn.declination);
                RfOut->declination = RfIn.declination; // set inclination'
                RfIn.ack = LORAGETACK;
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                break;
            case MAXMINPWR:
                if (RfIn.ack == LORAGET)
                {
                    xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                }
                else
                {
                    xQueueSend(loraOut, (void *)&RfIn, 0); // update
                    xQueueSend(udpOut, (void *)&RfIn, 0);  // update
                }
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                break;
            case MAXMINPWRSET:
                RfIn.ack = LORAGETACK;
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                break;
            default:
                break;
            }
        }
    }
}

// ***************************************************************************************************
// handle new gps datainput
// ***************************************************************************************************
void handelGpsData(RoboStruct *gps)
{
    RoboStruct gpsin, tx;
    if (xQueueReceive(gpsQue, (void *)&gpsin, 0) == pdTRUE) // New gps data
    {
        gps->lat = gpsin.lat;
        gps->lng = gpsin.lng;
        gps->gpsSat = gpsin.gpsSat;
        if (gpsin.gpsFix == true && gps->gpsFix == false)
        {
            beep(2000, buzzer);
            gps->gpsFix = true;
            tx.IDr = BUOYIDALL;
            tx.cmd = BUOYPOS;
            tx.ack = LORAINF;
            xQueueSend(loraOut, (void *)&tx, 10); // send out trough Lora
            xQueueSend(udpOut, (void *)&tx, 10);  // send out trough Lora
        }
        gps->gpsFix = gpsin.gpsFix;
    }
}

//***************************************************************************************************
//      New serial data
//***************************************************************************************************
void handelSerialData(RoboStruct *ser)
{
    RoboStruct serDataIn;
    if (xQueueReceive(serIn, (void *)&serDataIn, 1) == pdTRUE)
    {
        ser->lastSerIn = millis();
        if (mainCollorUtil.color != CRGB::DarkBlue)
        {
            mainCollorUtil.color = CRGB::DarkBlue;
            mainCollorUtil.blink = BLINK_SLOW;
            xQueueSend(ledUtil, (void *)&mainCollorUtil, 0); // update GPS led
        }
        switch (serDataIn.cmd)
        {
        case PONG:
            break;
        case DIRSPEED:
            ser->dirMag = serDataIn.dirMag;
            ser->speedBb = serDataIn.speedBb;
            ser->speedSb = serDataIn.speedSb;
            printf("M:%03.0f T:%03.0f D:%03.1f   BB:%d SB:%d\r\n", ser->dirMag, ser->tgDir, ser->tgDist, ser->speedBb, ser->speedSb);

            break;
        case SUBACCU:
            ser->subAccuV = serDataIn.subAccuV;
            ser->subAccuP = serDataIn.subAccuP;
            break;
        case IDELING:
            printf("#Status set to IDELING (by serial input)\r\n");
            ser->status = IDELING;
            break;
        case IDLE:
            ser->status = IDLE;
            break;
        case STOREASDOC:
            if (ser->gpsFix == true)
            {
                printf("Store Doc pos)\r\n");
                memDockPos(ser, SET);
            }
            ser->status = IDELING;
            break;

        default:
            break;
        }
    }
    if (mainData.lastSerIn + 2000 < millis())
    {
        mainData.lastSerIn = millis();
        if (mainCollorUtil.color != CRGB::Red)
        {
            mainCollorUtil.color = CRGB::Red;
            mainCollorUtil.blink = BLINK_SLOW;
            xQueueSend(ledUtil, (void *)&mainCollorUtil, 0); // update GPS led
        }
    }
}

//***************************************************************************************************
//      Main loop
//***************************************************************************************************
void loop(void)
{
    mainData.mac = espMac();
    mainData.IDs = espMac();
    Serial.println("Main loop running!");
    mainData.status = IDLE;
    mainCollorGps.color = CRGB::DarkRed;
    mainCollorGps.blink = BLINK_FAST;
    xQueueSend(ledGps, (void *)&mainCollorGps, 0); // update GPS led
    mainCollorUtil.blink = BLINK_SLOW;
    beep(1000, buzzer);
    mainData.lastLoraOut = millis();
    while (true)
    {
        //***************************************************************************************************
        //      Timer routines
        //***************************************************************************************************
        handleTimerRoutines(&mainData);
        //***************************************************************************************************
        //      Check front key
        //***************************************************************************************************
        handelKeyPress(&mainData);
        //***************************************************************************************************
        //      status actions
        //***************************************************************************************************
        handelStatus(&mainData, buoyPara);
        //***************************************************************************************************
        //      New data Rf data (UTP or Lora)
        //***************************************************************************************************
        handelRfData(&mainData, buoyParaPtrs);
        //****************************************************************************************************
        //      New GPS data
        //***************************************************************************************************
        handelGpsData(&mainData);
        //***************************************************************************************************
        //      New serial data
        //***************************************************************************************************
        // mainData = handelSerialData(mainData);
        handelSerialData(&mainData);
        //***************************************************************************************************
        //      Light button control
        //***************************************************************************************************
        buttonLight(mainData);
        //***************************************************************************************************
        //      Serial watchdog
        //***************************************************************************************************
    }
}