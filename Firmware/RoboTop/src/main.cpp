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
RoboStruct mainData;
// RoboStruct b0, b1, b2;
// RoboStruct *buoyPara[4] = {&b0, &b1, &b2};
RoboStruct buoyPara[3] = {};
RoboStruct *buoyParaPtrs[3] = {&buoyPara[0], &buoyPara[1], &buoyPara[2]};
static RoboStruct mainUdpIn;
static RoboWindStruct wind;
static LedData mainCollorStatus;
static LedData mainCollorUtil;
static LedData mainCollorGps;
static PwrData mainPwrData;
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
static unsigned int gpsErrorCnt = 0;
static unsigned int distErrorCnt = 0;

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
/**
 * @brief Initializes the system, peripherals, and tasks.
 * 
 * This function sets up serial communication, PID controller, pins, 
 * memory, and various queues. It also creates several FreeRTOS tasks 
 * for buzzer, LED, GPS, WiFi, Serial, and LoRa operations.
 */
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
    printf("Command ID SETUPDATA: %d\r\n", SETUPDATA);

    initMemory();
    initbuzzerqueue();
    initledqueue();
    initwifiqueue();
    initgpsqueue();
    initloraqueue();
    initserqueue();
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 2048, NULL, 1, NULL, 1);
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
    memDockPos(&mainData, GET);
    Serial.println("Dock position set to: ");
    Serial.printf("Lat: %.8lf, Lng: %.8lf\r\n", mainData.tgLat, mainData.tgLng);
    Serial.println("Main task running!");
    // defautls(&mainData);
}

//***************************************************************************************************
//      keypress detection
//***************************************************************************************************
#define LONG_PRESS_DURATION 5000 // ms
#define PRESS_TIMEOUT 500        // ms
/**
 * @brief Detects and counts button presses with timeout and long-press support.
 * 
 * @return int The number of short presses, or 100 + short press count for a sequence ending in a long press. 
 *         Returns -1 if no action is completed.
 */
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
/**
 * @brief Processes button press results and updates the buoy status.
 * 
 * @param key Pointer to the RoboStruct containing the buoy's state and data.
 */
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
            key->status = CALC_COMPASS_OFFSET;
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
/**
 * @brief Controls the button's LED behavior based on the buoy's status and GPS fix.
 * 
 * @param sta Pointer to the RoboStruct containing the buoy's status and GPS fix state.
 */
void buttonLight(RoboStruct *sta)
{
    if (buttonBlinkTimer < millis())
    {
        buttonBlinkTimer = millis() + blink;
        if (sta->status == LOCKED || sta->status == DOCKED)
        {
            digitalWrite(BUTTON_LIGHT_PIN, HIGH);
            blink = 2000;
        }
        else if (sta->status == CALIBRATE_MAGNETIC_COMPASS)
        {
            digitalWrite(BUTTON_LIGHT_PIN, !digitalRead(BUTTON_LIGHT_PIN));
            blink = 50;
        }
        else
        {
            digitalWrite(BUTTON_LIGHT_PIN, !digitalRead(BUTTON_LIGHT_PIN));
            if (sta->gpsFix == true)
            {
                if (blink != 1000)
                {
                    mainCollorGps.color = CRGB::Green;
                    mainCollorGps.blink = BLINK_SLOW;
                    xQueueSend(ledGps, (void *)&mainCollorGps, 0); //
                    blink = 1000;
                }
            }
            else
            {
                if (blink != 100)
                {
                    mainCollorGps.color = CRGB::DarkRed;
                    mainCollorGps.blink = BLINK_FAST;
                    xQueueSend(ledGps, (void *)&mainCollorGps, 0); //
                    blink = 100;
                }
            }
        }
    }
}

//***************************************************************************************************
//  status actions
//***************************************************************************************************
/**
 * @brief Handles state machine transitions and associated actions for the buoy.
 * 
 * @param stat Pointer to the current buoy's RoboStruct.
 * @param buoyPara Array of RoboStructs for all buoys in the system.
 */
void handelStatus(RoboStruct *stat, RoboStruct buoyPara[3])
{
    static int lastStatus = IDLE;
    if (lastStatus == IDLE && stat->status != IDLE && stat->status != IDELING)
    {
        RoboStruct wakeupMsg;
        wakeupMsg.cmd = WAKEUP;
        xQueueSend(serOut, (void *)&wakeupMsg, 0);
    }
    lastStatus = stat->status;

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
            stat->cmd = RESET_SPEED_RUD_PID;
            xQueueSend(serOut, (void *)stat, 0);  // send course and distance to sub
            xQueueSend(loraOut, (void *)stat, 0); // send course and distance to sub
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
        stat->cmd = RESET_SPEED_RUD_PID;
        xQueueSend(serOut, (void *)stat, 0);  // send course and distance to sub
        xQueueSend(loraOut, (void *)stat, 0); // send course and distance to sub
        stat->cmd = DIRDIST;
        xQueueSend(serOut, (void *)stat, 0);  // send course and distance to sub
        xQueueSend(loraOut, (void *)stat, 0); // send course and distance to sub
        break;

    case COMPUTESTART:
        buoyPara[0].wDir = stat->wDir;
        calcTrackPos(buoyPara);
        for (int i = 0; i < 3; i++)
        {
            trackPosPrint(buoyPara[i].trackPos);
            printf(" = (%.12f,%.12f)\r\n", buoyPara[i].tgLat, buoyPara[i].tgLng);
        }
        recalcStartLine(buoyPara);
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
        reCalcTrack(buoyPara);
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
    case CALC_COMPASS_OFFSET:
        LoraTx.cmd = CALC_COMPASS_OFFSET;
        LoraTx.ack = LORAGETACK;
        xQueueSend(serOut, (void *)&LoraTx, 10); // send out trough Lora
        stat->status = IDELING;
        break;
    case STOREASDOC:
        if (stat->gpsFix == true)
        {
            printf("Storing docpositoin\r\n");
            stat->tgLat = stat->lat;
            stat->tgLng = stat->lng;
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
//  In-Field Compass Calibration State Machine
//***************************************************************************************************
/**
 * @brief Manages the in-field compass calibration state machine.
 * 
 * @param timer Pointer to the RoboStruct containing calibration state and data.
 */
void handleInfieldCompassCalibration(RoboStruct *timer)
{
    static unsigned long calibStartTime = 0;
    static int calibPhase = 0;
    static double lat0, lon0;

    if (timer->status != INFIELD_CALIBRATE)
    {
        calibPhase = 0;
        return;
    }

    if (calibPhase == 0)
    {
        if (timer->gpsFix)
        {
            lat0 = timer->lat;
            lon0 = timer->lng;
            calibStartTime = millis();
            calibPhase = 1;
            printf("#INFIELD_COMPASS: Phase 0 (Home recorded: %f, %f)\r\n", lat0, lon0);
            
            // Trigger sub to do the spins
            RoboStruct cmdMsg;
            cmdMsg.cmd = INFIELD_CALIBRATE;
            cmdMsg.IDs = timer->mac;
            cmdMsg.ack = LORAINF;
            xQueueSend(serOut, (void *)&cmdMsg, 0);
        }
        else
        {
            printf("#INFIELD_COMPASS: Waiting for GPS fix...\r\n");
            delay(1000);
            return;
        }
    }
    else if (calibPhase == 1)
    {
        // Waiting for sub to finish (takes ~3 mins). Sub will send IDLE command when done.
        // We catch IDLE in handelRfData / handelSerialData which resets timer->status to IDELING/IDLE.
        // Wait, if timer->status becomes IDLE, this function exits!
        // To catch the end, we need to know it finished.
        // We'll just wait for the timeout (~190s) here as a backup if IDLE isn't caught,
        // or actually, if we let `handelSerialData` change `status` to `IDLE`, this state machine breaks before returning home!
        // We should intercept `IDLE` inside handelSerialData, or better, we just check if elapsed time > 160000ms
        
        unsigned long elapsed = millis() - calibStartTime;
        if (elapsed >= 165000) // 165 seconds = 2 mins 45 seconds (3 phases = 60+60+30 + 15 buffer)
        {
            printf("#INFIELD_COMPASS: Calibration time complete. Returning to Home (P0).\r\n");
            timer->tgLat = lat0;
            timer->tgLng = lon0;
            timer->status = LOCKED;
            calibPhase = 0;
            beep(1000, buzzer);
        }
    }
}

//***************************************************************************************************
//  In-Field Offset Calibration State Machine
//***************************************************************************************************
/**
 * @brief Manages the in-field compass offset calibration state machine.
 * 
 * @param timer Pointer to the RoboStruct containing calibration state and data.
 */
void handleInfieldOffsetCalibration(RoboStruct *timer)
{
    static unsigned long calibStartTime = 0;
    static int calibPhase = 0;
    static double lat0, lon0, lat1, lon1, lat2, lon2, lat2_stable, lon2_stable, lat3, lon3;
    static unsigned long lastUpdate = 0;

    if (timer->status != INFIELD_OFFSET_CALIBRATE)
    {
        calibPhase = 0;
        return;
    }

    if (calibPhase == 0)
    {
        if (timer->gpsFix)
        {
            lat0 = timer->lat;
            lon0 = timer->lng;
            calibStartTime = millis();
            calibPhase = 1;
            printf("#INFIELD_OFFSET: Phase 0 (Home recorded: %f, %f)\r\n", lat0, lon0);
        }
        else
        {
            printf("#INFIELD_OFFSET: Waiting for GPS fix...\r\n");
            delay(1000);
            return;
        }
    }

    unsigned long elapsed = millis() - calibStartTime;

    if (millis() - lastUpdate > 500)
    {
        lastUpdate = millis();
        RoboStruct cmdMsg;
        cmdMsg.cmd = TGDIRSPEED;
        cmdMsg.IDs = timer->mac;
        cmdMsg.ack = LORAINF;

        if (elapsed < 10000)
        {
            // Phase 0: Stabilization (Sail South for 10s before recording start point)
            cmdMsg.tgDir = 180.0;
            cmdMsg.speedSet = 50;
            xQueueSend(serOut, (void *)&cmdMsg, 0);
        }
        else if (calibPhase == 1 && elapsed >= 10000)
        {
            // Phase 1: Record Start Point P1 (after 10s of sailing South)
            lat1 = timer->lat;
            lon1 = timer->lng;
            calibPhase = 2;
            printf("#INFIELD_OFFSET: Phase 1 (P1 recorded: %f, %f)\r\n", lat1, lon1);
        }
        else if (calibPhase == 2 && elapsed < 130000)
        {
            // Phase 2: Calibration Leg
            cmdMsg.tgDir = 180.0;
            cmdMsg.speedSet = 50;
            xQueueSend(serOut, (void *)&cmdMsg, 0);
        }
        else if (calibPhase == 2 && elapsed >= 130000)
        {
            // Phase 3: Record End Point P2 and Calculate Offset
            lat2 = timer->lat;
            lon2 = timer->lng;
            
            double gpsCourse1 = calculateAngle(lat1, lon1, lat2, lon2);
            double newOffset = gpsCourse1 - 180.0;
            
            while (newOffset > 180.0) newOffset -= 360.0;
            while (newOffset < -180.0) newOffset += 360.0;

            timer->compassOffset += newOffset;
            
            printf("#INFIELD_OFFSET: Phase 3 (P2 recorded: %f, %f). GPS Course: %.2f. New Offset: %.2f\r\n", lat2, lon2, gpsCourse1, newOffset);
            
            // Send new offset to Sub
            RoboStruct offsetMsg;
            offsetMsg.cmd = STORE_COMPASS_OFFSET;
            offsetMsg.IDs = timer->mac;
            offsetMsg.ack = LORAINF;
            offsetMsg.compassOffset = timer->compassOffset;
            xQueueSend(serOut, (void *)&offsetMsg, 0);
            
            calibPhase = 4;
        }
        else if (calibPhase == 4 && elapsed < 140000)
        {
            // Phase 4: Return Leg Stabilization (wait for buoy to turn around)
            cmdMsg.tgDir = 0.0;
            cmdMsg.speedSet = 50;
            xQueueSend(serOut, (void *)&cmdMsg, 0);
        }
        else if (calibPhase == 4 && elapsed >= 140000)
        {
            // Phase 5: Record Stable Return Start Point
            lat2_stable = timer->lat;
            lon2_stable = timer->lng;
            calibPhase = 6;
            printf("#INFIELD_OFFSET: Phase 5 (P2_stable recorded for return leg: %f, %f)\r\n", lat2_stable, lon2_stable);
        }
        else if (calibPhase == 6 && elapsed < 260000)
        {
            // Phase 6: Return Leg Sailing
            cmdMsg.tgDir = 0.0;
            cmdMsg.speedSet = 50;
            xQueueSend(serOut, (void *)&cmdMsg, 0);
        }
        else if (calibPhase == 6 && elapsed >= 260000)
        {
            // Phase 7: Record Final Point P3 and Validate
            lat3 = timer->lat;
            lon3 = timer->lng;
            
            double gpsCourse2 = calculateAngle(lat2_stable, lon2_stable, lat3, lon3);
            double validationError = abs(gpsCourse2 - 0.0);
            while (validationError > 180.0) validationError -= 360.0;
            
            printf("#INFIELD_OFFSET: Phase 7 (P3 recorded). GPS Course: %.2f. Error: %.2f\r\n", gpsCourse2, abs(validationError));
            
            // Stop motors
            cmdMsg.tgDir = 0.0;
            cmdMsg.speedSet = 0;
            xQueueSend(serOut, (void *)&cmdMsg, 0);
            
            // Phase 8: Return Home
            timer->tgLat = lat0;
            timer->tgLng = lon0;
            timer->status = LOCKED;
            calibPhase = 0;
            printf("#INFIELD_OFFSET: Complete. Returning to Home (P0).\r\n");
            beep(1000, buzzer);
        }
    }
}

//***************************************************************************************************
//  Timer routines
//***************************************************************************************************
/**
 * @brief Executes periodic tasks and handles timed data transmissions.
 * 
 * @param timer Pointer to the RoboStruct containing the buoy's data and timers.
 */
void handleTimerRoutines(RoboStruct *timer)
{
    handleInfieldCompassCalibration(timer);
    handleInfieldOffsetCalibration(timer);

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
            timer->lastSerOut = millis() + 250;
            RouteToPoint(timer->lat, timer->lng, timer->tgLat, timer->tgLng, &timer->tgDist, &timer->tgDir);
            if (timer->tgDist > 10000)
            {
                distErrorCnt++;
                if (distErrorCnt > 100)
                {
                    distErrorCnt = 0;
                    timer->status = IDELING;
                }
                return;
            }
            distErrorCnt = 0;
            timer->cmd = DIRDIST;
            xQueueSend(serOut, (void *)timer, 0); // send course and distance to sub
            // timer->cmd = DIRMDIRTGDIRG;
            xQueueSend(udpOut, (void *)timer, 0); // send course and distance to sub
        }
        else if (timer->status == REMOTE) // Remote controlled
        {
            timer->lastSerOut = millis() + 500;
            timer->cmd = REMOTE;
            xQueueSend(serOut, (void *)timer, 0);
            timer->cmd = TGDIRSPEED;
            xQueueSend(udpOut, (void *)timer, 10); // send out trough wifi
        }
    }
    //***************************************************************************************************
    // RF data out
    //***************************************************************************************************
    if (timer->loralstmsg < millis())
    {
        if ((timer->status == LOCKED || timer->status == DOCKED))
        {
            timer->loralstmsg = millis() + 500;
            timer->cmd = TOPDATA;
            xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
            xQueueSend(udpOut, (void *)timer, 10);  // send out trough wifi
        }
        if (timer->status == REMOTE)
        {
            timer->loralstmsg = millis() + 1000 + random(0, 150);
            timer->cmd = REMOTE;
            xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
            xQueueSend(udpOut, (void *)timer, 10);
        }
    }

    if (logtimer < millis())
    {
        logtimer = millis() + 1000;
        battVoltage(timer->topAccuV, timer->topAccuP);
        addNewSampleInBuffer(&wind, timer->dirMag); // add sample to buffer for wind direction calculation.
        deviationWindRose(&wind);
        timer->wStd = wind.wStd;
        timer->wDir = wind.wDir; // averige wind dir
        timer->cmd = TOPDATA;
        // xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
        xQueueSend(udpOut, (void *)timer, 10); // send out trough wifi

        // RouteToPoint(timer->lat, timer->lng, timer->tgLat, timer->tgLng, &timer->tgDist, &timer->tgDir);
        printf("Status:%d Lat: %.8f Lon:%.8f tgDist:%.2f tgDir:%.0f mDir:%.0f wDir:%.0f wStd:%.2f ", timer->status, timer->lat, timer->lng, timer->tgDist, timer->tgDir, timer->dirMag, timer->wDir, timer->wStd);
        printf("Vtop: %1.1fV %3d%% Vsub: %1.1fV %d%% BB:%02d SB:%02d\r\n", timer->topAccuV, timer->topAccuP, timer->subAccuV, timer->subAccuP, timer->speedBb, timer->speedSb);
    }
    if (udpTimerOut + 10000 < millis() && !((timer->status == LOCKED || timer->status == DOCKED)))
    {
        udpTimerOut = millis() + random(0, 150);
        timer->cmd = BUOYPOS;
        xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
        xQueueSend(udpOut, (void *)timer, 10);  // send out trough wifi
        timer->cmd = TOPDATA;
        xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
        xQueueSend(udpOut, (void *)timer, 10);  // send out trough wifi
    }
}

// ***************************************************************************************************
// handle external data input
// ***************************************************************************************************
/**
 * @brief Processes incoming data from LoRa or UDP (WiFi).
 * 
 * @param RfOut Pointer to the RoboStruct to be updated with incoming data.
 * @param buoyPara Array of pointers to RoboStructs for all buoys.
 */
void handelRfData(RoboStruct *RfOut, RoboStruct *buoyPara[3])
{
    RoboStruct RfIn;
    RfIn.IDr = -1;
    bool from_udp = false;

    if (xQueueReceive(loraIn, (void *)&RfIn, 1) == pdTRUE) // new lora data
    {
        printf("handelRfData: Received from LoRa\r\n");
    }
    else if (xQueueReceive(udpIn, (void *)&RfIn, 1) == pdTRUE) // new udp data
    {
        from_udp = true;
        printf("handelRfData: Received from UDP\r\n");
    }

    if (RfIn.IDr != -1)
    {
        if(from_udp) {
            printf("handelRfData: Processing UDP message. IDr: %llX, IDs: %llX, cmd: %d, ack: %d, compassOffset: %f\r\n",
                   RfIn.IDr, RfIn.IDs, RfIn.cmd, RfIn.ack, RfIn.compassOffset);
        }
        
        // Deduplication Filter for Mode Commands
        static int lastInCmd = -1;
        static int lastInStatus = -1;
        static unsigned long lastInTime = 0;

        // Only filter mode-changing commands (DOCKING, LOCKING, IDLE, etc.)
        // Do NOT filter critical, single-shot action commands.
        if (RfIn.cmd != REMOTE && RfIn.cmd != TGDIRSPEED && RfIn.cmd != DIRDIST && RfIn.cmd != LOCKING && RfIn.cmd != DOCKING && RfIn.cmd != IDELING)
        {
            if (RfIn.cmd == lastInCmd && RfIn.status == lastInStatus && (millis() - lastInTime < 2000))
            {
                // Duplicate command received within 2 seconds, ignore it
                return;
            }
            lastInCmd = RfIn.cmd;
            lastInStatus = RfIn.status;
            lastInTime = millis();
        }

        RfOut->IDs = RfOut->mac;
        RfOut->IDr = RfIn.IDs;
        RfOut->ack = LORAINF;
        RfOut->cmd = RfIn.cmd;
        if (RfIn.IDr == RfOut->mac || RfIn.IDr == BUOYIDALL) // yes for me
        {
            switch (RfIn.cmd)
            {
            case DOCKING:
                if (RfOut->status != DOCKING && RfOut->status != DOCKED)
                {
                    printf("#Status set to DOCKING\r\n");
                    RfOut->status = DOCKING;
                }
                break;
            case LOCKPOS: // store new data into position database
                AddDataToBuoyBase(RfIn, buoyParaPtrs);
                break;
            case INFIELD_CALIBRATE:
            case INFIELD_OFFSET_CALIBRATE:
                if (RfIn.ack == LORAGET || RfIn.ack == LORAGETACK)
                {
                    xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                }
                else
                {
                    xQueueSend(udpOut, (void *)&RfIn, 0); // For status updates from other buoys
                    xQueueSend(loraOut, (void *)&RfIn, 0); // For status updates from other buoys
                }
                break;
            case PIDRUDDER:
                if (RfIn.ack == LORAGET || RfIn.ack == LORAGETACK)
                {
                    xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                }
                else
                {
                    // For status updates from other buoys, only forward to UDP for local display
                    xQueueSend(udpOut, (void *)&RfIn, 0);
                    xQueueSend(loraOut, (void *)&RfIn, 0); // For status updates from other buoys
                }
                break;
            case PIDRUDDERSET:
                pidRudderParameters(&RfIn, SET);
                printf("#PIDRUDDERSET: %05.2f %05.2f %05.2f\r\n", RfIn.Kpr, RfIn.Kir, RfIn.Kdr);
                RfIn.ack = LORAGETACK;
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                break;
            case PIDSPEED:
                if (RfIn.ack == LORAGET || RfIn.ack == LORAGETACK)
                {
                    xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                }
                else
                {
                    // For status updates from other buoys, only forward to UDP for local display
                    xQueueSend(udpOut, (void *)&RfIn, 0);
                    xQueueSend(loraOut, (void *)&RfIn, 0); // For status updates from other buoys
                }
                break;
            case SETUPDATA:
                if (RfIn.ack == LORAGET || RfIn.ack == LORAGETACK)
                {
                    xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                }
                else
                {
                    // Update buoyPara base so web interface shows correct remote setup
                    for (int i = 0; i < 3; i++) {
                        if (buoyPara[i]->IDs == RfIn.IDs || buoyPara[i]->IDs == 0) {
                            *buoyPara[i] = RfIn;
                            break;
                        }
                    }
                    // For status updates from other buoys, only forward to UDP for local display
                    xQueueSend(udpOut, (void *)&RfIn, 0);
                    xQueueSend(loraOut, (void *)&RfIn, 0); // For status updates from other buoys
                }
                break;
            case PIDSPEEDSET:
                pidSpeedParameters(&RfIn, SET);
                printf("#PIDSPEEDSET: %05.2f %05.2f %05.2f\r\n", RfIn.Kps, RfIn.Kis, RfIn.Kds);
                RfOut->cmd = PIDSPEEDSET;
                RfIn.ack = LORAGETACK;
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                RfOut->Kps = RfIn.Kps;
                RfOut->Kis = RfIn.Kis;
                RfOut->Kds = RfIn.Kds;
                break;
            case TGDIRSPEED:
                RfOut->tgDir = RfIn.tgDir;
                RfOut->speedSet = RfIn.speedSet;
                RfOut->status = TGDIRSPEED;
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
                // printf("New dir: %f New distance %f\r\n", RfIn.tgDir, RfIn.tgDist);
                adjustPositionDirDist(RfIn.tgDir, RfIn.tgDist, RfOut->lat, RfOut->lng, &RfOut->tgLat, &RfOut->tgLng);
                // printf("New cordinates: https://www.google.nl/maps/@%f,%f,14z\r\n", RfOut->tgLat, RfOut->tgLng);
                RouteToPoint(RfOut->lat, RfOut->lng, RfOut->tgLat, RfOut->tgLng, &RfOut->tgDist, &RfOut->tgDir);
                // printf("New dir: %f New distance %f\r\n", RfOut->tgDir, RfOut->tgDist);
                RfOut->status = LOCKED;
                RfIn.cmd = RESET_SPEED_RUD_PID;
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
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
                break;
            case STOREASDOC: // Store location as doc location
                if (RfOut->gpsFix == true)
                {
                    RfOut->tgLat = RfOut->lat;
                    RfOut->tgLng = RfOut->lng;
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
                AddDataToBuoyBase(RfIn, buoyParaPtrs);
                RfOut->status = LOCKED;
                break;
            case IDELING:
            case IDLE:
                if (RfOut->status != IDELING && RfOut->status != IDLE)
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
                if (RfIn.ack == LORAGET || RfIn.ack == LORAGETACK)
                {
                    xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                }
                else
                {
                    // For status updates from other buoys, only forward to UDP for local display
                    xQueueSend(udpOut, (void *)&RfIn, 0);
                    xQueueSend(loraOut, (void *)&RfIn, 0); // For status updates from other buoys
                }
                break;
            case MAXMINPWRSET:
                RfIn.ack = LORAGETACK;
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                break;
            case STORE_COMPASS_OFFSET:
                RfOut->compassOffset = RfIn.compassOffset; // Update Top Buoy's local data
                RfOut->icmCompassOffset = RfIn.icmCompassOffset; // Update Top Buoy's local data
                xQueueSend(serOut, (void *)&RfIn, 0); // Forward the command to the sub
                break;
            case RAWCOMPASSDATA:
                printf("Raw:0,0,0,0,0,0,%5.3f,%5.3f,%5.3f\r\n", RfIn.magHard[0], RfIn.magHard[1], RfIn.magHard[2]);
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
/**
 * @brief Processes incoming GPS data from the GPS queue and applies filtering.
 * 
 * @param gps Pointer to the RoboStruct where GPS coordinates and fix status will be updated.
 */
void handelGpsData(RoboStruct *gps)
{
    RoboStruct gpsin;

    if (xQueueReceive(gpsQue, (void *)&gpsin, 0) == pdTRUE) // New gps data
    {
        if (gpsin.gpsFix)
        {
            // --- First valid fix initialization ---
            if (!gps->gpsFix)
            {
                gpsErrorCnt = 0;
                gps->lat = gpsin.lat;
                gps->lng = gpsin.lng;
                gps->gpsSat = gpsin.gpsSat;
                gps->gpsFix = true;
                // First time fix event
                Serial.printf("First GPS fix acquired\r\n");
                beep(2000, buzzer);
                return;
            }

            // --- Accept small movements immediately ---
            double dist = distanceBetween(gpsin.lat, gpsin.lng, gps->lat, gps->lng);
            if (dist < 50)
            {
                gpsErrorCnt = 0;
                gps->lat = gpsin.lat;
                gps->lng = gpsin.lng;
                gps->gpsSat = gpsin.gpsSat;
                gps->gpsFix = true;
            }
            else
            {
                // Large jump: might be an outlier OR the buoy was moved
                gpsErrorCnt++;
                Serial.printf("Potential GPS outlier (dist: %.2f m), count: %u\r\n", dist, gpsErrorCnt);
                
                // If we get many consecutive outliers, accept the new position
                if (gpsErrorCnt > 30) 
                {
                    Serial.println("Accepting new GPS position after persistent outliers.");
                    gpsErrorCnt = 0;
                    gps->lat = gpsin.lat;
                    gps->lng = gpsin.lng;
                    gps->gpsSat = gpsin.gpsSat;
                    gps->gpsFix = true;
                }
            }
        }
        else
        {
            // Fix lost
            gps->gpsFix = false;
            gps->gpsSat = gpsin.gpsSat;
        }
    }
}
//***************************************************************************************************
//      New serial data
//***************************************************************************************************
/**
 * @brief Processes incoming serial data from the sub-controller.
 * 
 * @param ser Pointer to the RoboStruct to be updated with serial data.
 */
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
        case SUBDATA:
            ser->dirMag = serDataIn.dirMag;
            ser->speedBb = serDataIn.speedBb;
            ser->speedSb = serDataIn.speedSb;
            ser->ip = serDataIn.ip;
            ser->ir = serDataIn.ir;
            ser->subAccuV = serDataIn.subAccuV;
            ser->subAccuP = serDataIn.subAccuP;
            
            mainPwrData.ledBb = -ser->speedBb;
            mainPwrData.ledSb = -ser->speedSb;
            xQueueSend(ledPwr, (void *)&mainPwrData, 0);
            break;
        case SETUPDATA:
            serDataIn.IDs = mainData.IDs; // FIX: ensure IDs is set to Top's IDs for consistent buoy identification!
            serDataIn.mac = mainData.mac; // FIX: ensure mac is set to Top's mac before sending out!
            serDataIn.IDr = BUOYIDALL;    // FIX: ensure response is broadcasted to all (ID 1) just like BUOYPOS

            // Update local mainData so web interface sees the correct values
            mainData.Kpr = serDataIn.Kpr;
            mainData.Kir = serDataIn.Kir;
            mainData.Kdr = serDataIn.Kdr;
            mainData.Kps = serDataIn.Kps;
            mainData.Kis = serDataIn.Kis;
            mainData.Kds = serDataIn.Kds;
            mainData.maxSpeed = serDataIn.maxSpeed;
            mainData.minSpeed = serDataIn.minSpeed;
            mainData.pivotSpeed = serDataIn.pivotSpeed;
            mainData.compassOffset = serDataIn.compassOffset;

            xQueueSend(udpOut, (void *)&serDataIn, 0);
            xQueueSend(loraOut, (void *)&serDataIn, 0);
            printf("Setup data PID and Compass sent and updated locally\r\n");
            break;
        case PONG:
            break;
        case DIRSPEED:
            ser->dirMag = serDataIn.dirMag;
            ser->speedBb = serDataIn.speedBb;
            ser->speedSb = serDataIn.speedSb;
            printf("M:%03.0f T:%03.0f D:%03.1f   BB:%d SB:%d\r\n", ser->dirMag, ser->tgDir, ser->tgDist, ser->speedBb, ser->speedSb);
            
            mainPwrData.ledBb = -ser->speedBb;
            mainPwrData.ledSb = -ser->speedSb;
            xQueueSend(ledPwr, (void *)&mainPwrData, 0);
            break;
        case SUBACCU:
            ser->subAccuV = serDataIn.subAccuV;
            ser->subAccuP = serDataIn.subAccuP;
            break;
        case IDELING:
            if (ser->status != IDELING && ser->status != IDLE)
            {
                printf("#Status set to IDELING (by serial input)\r\n");
                ser->status = IDELING;
            }
            break;
        case IDLE:
            // Prevent delayed IDLE from Sub buoy from canceling an active state
            if (ser->status == IDELING || ser->status == IDLE) {
                ser->status = IDLE;
            }
            break;
        case LOCKED:
            if (ser->status != LOCKED && ser->status != LOCKING && ser->status != LOCKPOS) {
                ser->status = LOCKED;
            }
            break;
        case DOCKED:
            if (ser->status != DOCKED && ser->status != DOCKING) {
                ser->status = DOCKED;
            }
            break;
        case STOREASDOC:
            if (ser->gpsFix == true)
            {
                printf("Store Doc pos)\r\n");
                memDockPos(ser, SET);
            }
            ser->status = IDELING;
            break;
        case RAWCOMPASSDATA:
            printf("Raw:0,0,0,0,0,0,%.0f,%.0f,%.0f\r\n", serDataIn.magHard[0], serDataIn.magHard[1], serDataIn.magHard[2]);
            break;
        case PIDRUDDER:
        case PIDSPEED:
        case MAXMINPWR:
        case MAXMINPWRSET:
            if (serDataIn.ack == LORAGET || serDataIn.ack == LORAGETACK)
            {
                // This is a request from PC (Serial) -> Forward to Sub
                xQueueSend(serOut, (void *)&serDataIn, 0);
            }
            else
            {
                // This is a response from Sub -> Forward to PC/LoRa/UDP
                printf("Received PID/PWR data from sub. CMD: %d\r\n", serDataIn.cmd);
                serDataIn.IDs = mainData.IDs; // FIX: ensure IDs is set to Top's IDs for consistent buoy identification!
                serDataIn.mac = mainData.mac; // FIX: ensure mac is set to Top's mac before sending out!
                
                // IMPORTANT: Send the actual protocol string to the PC via Serial so RoboControl.py sees it!
                Serial.println(rfCode(&serDataIn));

                xQueueSend(udpOut, (void *)&serDataIn, 0);
                xQueueSend(loraOut, (void *)&serDataIn, 0);
            }
            break;

        default:
            break;
        }
    }
    if (ser->lastSerIn + 2000 < millis())
    {
        ser->lastSerIn = millis();
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
/**
 * @brief Main execution loop that continuously calls various handler functions.
 */
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
    beep(1, buzzer);
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
        buttonLight(&mainData);
        //***************************************************************************************************
        //      Serial watchdog
        //***************************************************************************************************
    }
}
