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
static RoboStruct buoyPara[3] = {};
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
    Serial.println("Main task running!");
    // defautls(mainData);
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
//      One press: lock/unlock
//      Two press: start computation
//      Three press: compute track
//      Five press: sail to dock position
//      Ten press: store as doc
//***************************************************************************************************
void handelKeyPress(RoboStruct *key)
{
    int presses = countKeyPressesWithTimeoutAndLongPressDetecton();
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
        case 10:
            key->status = START_CALIBRATE_MAGNETIC_COMPASS;
            break;
        case 0x0100:
            key->status = STOREASDOC;
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
            digitalWrite(BUTTON_LIGHT_PIN, LOW);
            blink = 2000;
            // mainCollorGps.color = CRGB::Green;
            // mainCollorGps.blink = BLINK_OFF;
            // xQueueSend(ledGps, (void *)&mainCollorGps, 0); // update led
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
void handelStatus(RoboStruct *stat, RoboStruct *buoyPara)
{
    RoboStruct LoraTx;
    RoboStruct doc;
    stat->IDs = stat->buoyId;
    LoraTx = *stat;
    stat->IDr = BUOYIDALL;
    switch (stat->status)
    {
    case IDELING:
        beep(2, buzzer);
        stat->cmd = IDLE;
        stat->status = IDLE;
        xQueueSend(udpOut, (void *)&stat, 0);  // update WiFi
        xQueueSend(loraOut, (void *)&stat, 0); // update Lora
        break;
    case LOCKING:
        if (stat->gpsFix == true)
        {
            beep(1, buzzer);
            PowerOnSub();
            stat->status = LOCKED;
            stat->IDr = BUOYIDALL; // Send to all
            stat->IDs = stat->mac; // msg is comming from me
            stat->cmd = LOCKPOS;
            stat->ack = LORASET;
            stat->tgLat = stat->lat;
            stat->tgLng = stat->lng;
            AddDataToBuoyBase(*stat, buoyPara); // store positon for later calculations Track positioning
            // IDr,IDs,ACK,MSG,LAT,LON
            xQueueSend(udpOut, (void *)&stat, 0);   // update WiFi
            xQueueSend(loraOut, (void *)&stat, 10); // send out trough Lora
        }
        else
        {
            beep(2, buzzer);
            stat->status = IDLE;
        }
        break;

    case DOCKING:
        beep(1, buzzer);
        PowerOnSub();
        doc = memDockPos(doc, GET);
        stat->tgLat = doc.tgLat;
        stat->tgLng = doc.tgLng;
        stat->status = DOCKED;
        printf("Retreved data for docking tgLat:%.8f tgLng:%.8f\r\n", stat->tgLat, stat->tgLng);
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
            if (buoyPara[i].IDs == stat->buoyId)
            {
                stat->tgLat = buoyPara[i].tgLat;
                stat->tgLng = buoyPara[i].tgLng;
            }
        }
        stat->status = LOCKED;
        break;
    case START_CALIBRATE_MAGNETIC_COMPASS:
        PowerOnSub();
        delay(500);
        LoraTx.cmd = CALIBRATE_MAGNETIC_COMPASS;
        xQueueSend(serOut, (void *)&LoraTx, 10); // send out trough Lora
        stat->status = CALIBRATE_MAGNETIC_COMPASS;
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
    if (timer->lastSerOut + 100 < millis())
    {
        timer->lastSerOut = millis();
        if (timer->status == LOCKED || timer->status == DOCKED)
        {
            RouteToPoint(timer->lat, timer->lng, timer->tgLat, timer->tgLng, &timer->tgDist, &timer->tgDir);
            timer->cmd = DIRDIST;
            xQueueSend(serOut, (void *)timer, 0); // Keep the watchdog in sub happy
        }
        else if (timer->status == REMOTE) // Send course info
        {
            timer->cmd = SPBBSPSB;
            xQueueSend(serOut, (void *)timer, 0); // Keep the watchdog in sub happy
        }
        else
        {
            timer->cmd = PING;
        }
    }
    //***************************************************************************************************
    // RF data out
    //***************************************************************************************************
    if (timer->loralstmsg + 5000 < millis())
    {
        timer->loralstmsg = millis() + random(0, 150);
        timer->cmd = BUOYPOS;
        xQueueSend(loraOut, (void *)timer, 10); // send out trough Lora
        xQueueSend(udpOut, (void *)timer, 10);  // send out trough wifi
        timer->cmd = SUBPWR;
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
            xQueueSend(serOut, (void *)timer, 10);
            xQueueSend(udpOut, (void *)timer, 10);
        }
    }

    if (logtimer < millis())
    {
        logtimer = millis() + 1000 + random(0, 150);
        battVoltage(timer->topAccuV, timer->topAccuP);
        wind = addNewSampleInBuffer(wind, timer->dirMag); // add sample to buffer for wind direction calculation.
        wind = deviationWindRose(wind);
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
        xQueueSend(udpOut, (void *)timer, 0); // Send to udp
    }
}

// ***************************************************************************************************
// handle external data input
// ***************************************************************************************************
void handelRfData(RoboStruct *RfOut)
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
        if (RfIn.IDr == BUOYIDALL)
        {
            switch (RfIn.cmd)
            {
            case DOCKING:
                printf("#Status set to DOCKING (by lora input)\r\n");
                RfOut->status = DOCKING;
                break;
            case LOCKPOS: // store new data into position database
                buoyPara[3] = AddDataToBuoyBase(RfIn, buoyPara);
                break;
            case PIDRUDDERSET:
                pidRudderParameters(RfIn, SET);
                printf("#PIDRUDDERSET: %05.2f %05.2f %05.2f\r\n", RfIn.pr, RfIn.ir, RfIn.dr);
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                break;
            case PIDSPEEDSET:
                pidSpeedParameters(RfIn, SET);
                printf("#PIDSPEEDSET: %05.2f %05.2f %05.2f\r\n", RfIn.ps, RfIn.is, RfIn.ds);
                RfOut->cmd = PIDSPEEDSET;
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                break;
            }
        }
        if (RfIn.IDr == RfOut->mac) // yes for me
        {
            switch (RfIn.cmd)
            {
            case DIRSPEED:
                RfOut->tgDir = RfIn.tgDir;
                RfOut->speedSet = RfIn.speedSet;
                if (RfOut->status != REMOTE)
                {
                    PowerOnSub();
                }
                RfOut->status = REMOTE;
                break;
            case DIRDIST:
                RfOut->tgDir = RfIn.tgDir;
                RfOut->tgDist = RfIn.tgDist;
                if (RfOut->status != LOCKED)
                {
                    PowerOnSub();
                }
                RfOut->status = LOCKED;
                break;
            case LOCKING:
                if (RfOut->gpsFix == true)
                {
                    RfOut->status = LOCKING;
                    beep(1, buzzer);
                }
                break;
            case LOCKPOS: // store new data into position database
                buoyPara[3] = AddDataToBuoyBase(*RfOut, buoyPara);
                break;
            case SETLOCKPOS: // store new data into position database
                RfOut->tgLat = RfIn.tgLat;
                RfOut->tgLng = RfIn.tgLng;
                buoyPara[3] = AddDataToBuoyBase(RfIn, buoyPara);
                RfOut->status = LOCKING;
                break;
            case DOCKING:
                printf("#Status set to DOCKING (by lora input)\r\n");
                RfOut->status = DOCKING;
                break;
            case IDELING:
                printf("#Status set to IDLE (by lora input)\r\n");
                RfOut->status = IDELING;
                break;
            case PIDRUDDERSET:
                pidRudderParameters(RfIn, SET);
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                break;
            case PIDSPEEDSET:
                pidSpeedParameters(RfIn, SET);
                xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                break;
            case SUBACCU:
                RfOut->subAccuV = RfIn.subAccuV;
                RfOut->subAccuP = RfIn.subAccuP;
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
        gps->gpsFix = gpsin.gpsFix;
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
    }
}
//***************************************************************************************************
//      New serial data
//***************************************************************************************************
// RoboStruct handelSerialData(RoboStruct ser)
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
            // printf("DirMag: %05.2f BB: %d SB: %d\r\n", ser->dirMag, ser->speedBb, ser->speedSb);

            break;
        case SUBACCU:
            ser->subAccuV = serDataIn.subAccuV;
            ser->subAccuP = serDataIn.subAccuP;
            break;
        case IDELING:
            printf("#Status set to IDELING (by serial input)\r\n");
            ser->status = IDELING;
            break;
        default:
            break;
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
        handelRfData(&mainData);
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
}