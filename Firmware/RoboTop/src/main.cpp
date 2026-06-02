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

RoboStruct mainData;
RoboStruct buoyPara[3] = {};
RoboStruct *buoyParaPtrs[3] = {&buoyPara[0], &buoyPara[1], &buoyPara[2]};

static RoboWindStruct wind;
static LedData mainCollorStatus;
static LedData mainCollorUtil;
static LedData mainCollorGps;
static PwrData mainPwrData;
static Buzz mainBuzzerData;

static int wifiConfig = 0;
static int blink = BLINK_FAST;
static unsigned long buttonBlinkTimer = millis();
static unsigned long logtimer = millis();

bool lastButtonState = false;
bool longPressReported = false;
unsigned long lastPressTime = 0;
unsigned long debounceDelay = 0;
int pressCount = 0;

double Setpoint, Input, Output;
double Kp = 20, Ki = 0.05, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/**
 * @brief Surgical update of a RoboStruct from an incoming packet.
 */
void surgicalUpdate(RoboStruct *target, RoboStruct *source) {
    target->IDs = source->IDs;
    target->cmd = source->cmd;
    target->ack = source->ack;
    target->status = source->status;

    switch (source->cmd) {
    case SUBDATA:
        target->dirMag = source->dirMag;
        target->speedBb = source->speedBb;
        target->speedSb = source->speedSb;
        target->ip = source->ip;
        target->ir = source->ir;
        target->subAccuV = source->subAccuV;
        target->subAccuP = source->subAccuP;
        target->subAccuI = source->subAccuI;
        break;
    case TOPDATA:
        target->dirMag = source->dirMag;
        target->gpsDir = source->gpsDir;
        target->tgDir = source->tgDir;
        target->tgDist = source->tgDist;
        target->wDir = source->wDir;
        target->wStd = source->wStd;
        target->speedBb = source->speedBb;
        target->speedSb = source->speedSb;
        target->ip = source->ip;
        target->ir = source->ir;
        target->subAccuV = source->subAccuV;
        target->subAccuP = source->subAccuP;
        target->lat = source->lat;
        target->lng = source->lng;
        target->gpsFix = source->gpsFix;
        target->gpsSat = source->gpsSat;
        break;
    case SETUPDATA:
        target->Kpr = source->Kpr; target->Kir = source->Kir; target->Kdr = source->Kdr;
        target->Kps = source->Kps; target->Kis = source->Kis; target->Kds = source->Kds;
        target->maxSpeed = source->maxSpeed; target->minSpeed = source->minSpeed;
        target->pivotSpeed = source->pivotSpeed;
        target->compassOffset = source->compassOffset;
        target->icmCompassOffset = source->icmCompassOffset;
        target->mechanicCorrection = source->mechanicCorrection;
        target->minOfsetDist = source->minOfsetDist;
        target->revBB = source->revBB; target->revSB = source->revSB;
        target->swap_BB_SB = source->swap_BB_SB;
        break;
    case PIDRUDDERSET:
        target->Kpr = source->Kpr; target->Kir = source->Kir; target->Kdr = source->Kdr;
        break;
    case PIDSPEEDSET:
        target->Kps = source->Kps; target->Kis = source->Kis; target->Kds = source->Kds;
        break;
    case MAXMINPWRSET:
        target->maxSpeed = source->maxSpeed; target->minSpeed = source->minSpeed; target->pivotSpeed = source->pivotSpeed;
        break;
    case STORE_COMPASS_OFFSET:
        target->compassOffset = source->compassOffset;
        target->icmCompassOffset = source->icmCompassOffset;
        target->mechanicCorrection = source->mechanicCorrection;
        break;
    case DIRSPEED:
        target->dirMag = source->dirMag;
        target->speedBb = source->speedBb;
        target->speedSb = source->speedSb;
        break;
    case SUBACCU:
        target->subAccuV = source->subAccuV;
        target->subAccuP = source->subAccuP;
        break;
    case LOCKPOS:
    case SETLOCKPOS:
        target->tgLat = source->tgLat;
        target->tgLng = source->tgLng;
        target->status = LOCKED;
        break;
    case MDIR:
        target->dirMag = source->dirMag;
        break;
    }
}

void setup()
{
    Serial.begin(115200);
    Setpoint = 0;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-100, 100);

    pinMode(BUTTON_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_LIGHT_PIN, OUTPUT);
    delay(100);
    printf("\r\nNicE-Buoy Top Booting...\r\n");

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

    wifiConfig = (digitalRead(BUTTON_PIN) == true) ? 1 : 0;
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8192, &wifiConfig, configMAX_PRIORITIES - 10, NULL, 0);
    xTaskCreatePinnedToCore(SercomTask, "SerialTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    
    // Load local params
    mainData.mac = espMac();
    mainData.IDs = mainData.mac;
    mainData.status = IDLE;
    memDockPos(&mainData, GET);
    thrusterInversion(&mainData, GET);
    thrusterSwap(&mainData, GET);
    pidRudderParameters(&mainData, GET);
    pidSpeedParameters(&mainData, GET);
    computeParameters(&mainData, GET);
    CompasOffset(&mainData, GET);
    CompasIcmOffset(&mainData, GET);
    MechanicalCorrection(&mainData.mechanicCorrection, GET);

    buoyPara[0] = mainData;
    Serial.printf("Top Buoy ID: %08X\r\n", (unsigned int)mainData.mac);
    Serial.println("Dock position set to: ");
    Serial.printf("Lat: %.8lf, Lng: %.8lf\r\n", mainData.tgLat, mainData.tgLng);
    Serial.println("Main task running!");
}

int countKeyPresses()
{
    unsigned long currentTime = millis();
    int buttonState = digitalRead(BUTTON_PIN);
    if (currentTime < debounceDelay) return -1;

    if (buttonState == HIGH && lastButtonState == LOW) {
        lastPressTime = currentTime;
        longPressReported = false;
        beep(2000, buzzer);
    }

    if (buttonState == HIGH && (currentTime - lastPressTime > 3000) && !longPressReported) {
        beep(2000, buzzer);
        longPressReported = true;
        return 100;
    }

    if (buttonState == LOW && lastButtonState == HIGH) {
        debounceDelay = currentTime + 50;
        if (!longPressReported) pressCount++;
        else pressCount = 0;
    }

    if ((currentTime - lastPressTime) > 500 && pressCount > 0 && buttonState == LOW) {
        int result = pressCount;
        pressCount = 0;
        return result;
    }
    lastButtonState = buttonState;
    return -1;
}

void handelKeyPress(RoboStruct *key)
{
    int presses = countKeyPresses();
    if (presses > 0) {
        switch (presses) {
        case 1: key->status = (key->status != LOCKED && key->status != DOCKED) ? LOCKING : IDELING; break;
        case 2: key->status = COMPUTESTART; break;
        case 3: key->status = COMPUTETRACK; break;
        case 5: key->status = DOCKING; break;
        case 100: key->status = STOREASDOC; break;
        case 10: key->status = CALC_COMPASS_OFFSET; break;
        case 110: key->status = START_CALIBRATE_MAGNETIC_COMPASS; break;
        default: beep(-1, buzzer); break;
        }
    }
}

void buttonLight(RoboStruct *sta)
{
    if (buttonBlinkTimer < millis()) {
        buttonBlinkTimer = millis() + blink;
        if (sta->status == LOCKED || sta->status == DOCKED) {
            digitalWrite(BUTTON_LIGHT_PIN, HIGH); blink = 2000;
        } else {
            digitalWrite(BUTTON_LIGHT_PIN, !digitalRead(BUTTON_LIGHT_PIN));
            if (sta->gpsFix) {
                if (blink != 1000) {
                    mainCollorGps.color = CRGB::Green; mainCollorGps.blink = BLINK_SLOW;
                    xQueueSend(ledGps, (void *)&mainCollorGps, 0); blink = 1000;
                }
            } else if (blink != 100) {
                mainCollorGps.color = CRGB::DarkRed; mainCollorGps.blink = BLINK_FAST;
                xQueueSend(ledGps, (void *)&mainCollorGps, 0); blink = 100;
            }
        }
    }
}

void handelStatus(RoboStruct *stat, RoboStruct para[3])
{
    static int lastStatus = IDLE;
    if (lastStatus == IDLE && stat->status != IDLE && stat->status != IDELING) {
        RoboStruct wakeupMsg = {}; wakeupMsg.cmd = WAKEUP;
        xQueueSend(serOut, (void *)&wakeupMsg, 0);
    }
    lastStatus = stat->status;

    stat->IDs = stat->mac;
    switch (stat->status) {
    case IDELING:
        beep(2, buzzer); stat->cmd = IDLE; stat->status = IDLE;
        xQueueSend(udpOut, (void *)stat, 0); xQueueSend(loraOut, (void *)stat, 0);
        stat->ack = LORAGETACK; xQueueSend(serOut, (void *)stat, 20);
        break;
    case LOCKING:
        if (stat->gpsFix) {
            beep(1, buzzer); stat->status = LOCKED; stat->cmd = LOCKPOS; stat->ack = LORASET;
            stat->tgLat = stat->lat; stat->tgLng = stat->lng;
            AddDataToBuoyBase(*stat, buoyParaPtrs);
            xQueueSend(udpOut, (void *)stat, 0); xQueueSend(loraOut, (void *)stat, 10);
            RouteToPoint(stat->lat, stat->lng, stat->tgLat, stat->tgLng, &stat->tgDist, &stat->tgDir);
            stat->cmd = RESET_SPEED_RUD_PID; xQueueSend(serOut, (void *)stat, 0);
            stat->cmd = DIRDIST; xQueueSend(serOut, (void *)stat, 0);
        } else { beep(2, buzzer); stat->status = IDLE; }
        break;
    case DOCKING:
        beep(1, buzzer); memDockPos(stat, GET); stat->status = DOCKED;
        RouteToPoint(stat->lat, stat->lng, stat->tgLat, stat->tgLng, &stat->tgDist, &stat->tgDir);
        stat->cmd = RESET_SPEED_RUD_PID; xQueueSend(serOut, (void *)stat, 0);
        stat->cmd = DIRDIST; xQueueSend(serOut, (void *)stat, 0);
        break;
    case STOREASDOC:
        if (stat->gpsFix) { stat->tgLat = stat->lat; stat->tgLng = stat->lng; memDockPos(stat, SET); beep(1000, buzzer); }
        stat->status = IDELING; break;
    }
}

void handleTimerRoutines(RoboStruct *timer)
{
    timer->IDs = timer->mac;
    if (logtimer < millis()) {
        logtimer = millis() + 1000;
        battVoltage(timer->topAccuV, timer->topAccuP);
        addNewSampleInBuffer(&wind, timer->dirMag); deviationWindRose(&wind);
        timer->wStd = wind.wStd; timer->wDir = wind.wDir;
        
        RoboStruct tx = *timer;
        tx.cmd = TOPDATA; tx.IDr = BUOYIDALL; tx.ack = LORAINF;
        xQueueSend(udpOut, (void *)&tx, 10);
        
        printf("ID:%08X ST:%d Lat:%.8f Lon:%.8f mDir:%.0f\r\n", (unsigned int)timer->mac, timer->status, timer->lat, timer->lng, timer->dirMag);
    }
}

void processIncomingCommand(RoboStruct *RfOut, RoboStruct *msgIn) {
    bool isForMe = (msgIn->IDr == RfOut->mac || msgIn->IDr == BUOYIDALL);
    bool isSet = (msgIn->ack == LORASET || msgIn->ack == LORAGETACK);

    if (isForMe && isSet) {
        surgicalUpdate(RfOut, msgIn);
        if (msgIn->cmd == SETUPDATA || msgIn->cmd == PIDRUDDERSET) pidRudderParameters(RfOut, SET);
        if (msgIn->cmd == SETUPDATA || msgIn->cmd == PIDSPEEDSET) pidSpeedParameters(RfOut, SET);
        if (msgIn->cmd == SETUPDATA || msgIn->cmd == MAXMINPWRSET) computeParameters(RfOut, SET);
        if (msgIn->cmd == SETUPDATA || msgIn->cmd == STORE_COMPASS_OFFSET) { CompasOffset(RfOut, SET); CompasIcmOffset(RfOut, SET); MechanicalCorrection(&RfOut->mechanicCorrection, SET); }
        if (msgIn->cmd == SETUPDATA) { thrusterInversion(RfOut, SET); thrusterSwap(RfOut, SET); }
        
        // Respond for Top unit
        RoboStruct res = *RfOut;
        res.IDs = RfOut->mac; res.IDr = msgIn->IDs; res.cmd = msgIn->cmd; res.ack = LORAINF;
        Serial.println(rfCode(&res));
        xQueueSend(udpOut, (void *)&res, 0);
    }
    
    // Forward to Sub
    if (msgIn->IDr != RfOut->mac) {
        msgIn->IDr = BUOYIDALL;
        xQueueSend(serOut, (void *)msgIn, 0);
    }
}

void handelRfData(RoboStruct *RfOut, RoboStruct *para[3])
{
    RoboStruct RfIn = {}; RfIn.IDr = -1;
    if (xQueueReceive(loraIn, (void *)&RfIn, 1) == pdTRUE) {}
    else if (xQueueReceive(udpIn, (void *)&RfIn, 1) == pdTRUE) {}

    if (RfIn.IDr != -1) {
        if (RfIn.ack == LORAGET || RfIn.ack == LORAGETACK || RfIn.ack == LORASET) {
            processIncomingCommand(RfOut, &RfIn);
        } else {
            // Response from other buoy
            int idx = -1;
            for (int i = 1; i < 3; i++) { if (buoyPara[i].IDs == RfIn.IDs || buoyPara[i].IDs == 0) { idx = i; break; } }
            if (idx != -1) {
                surgicalUpdate(&buoyPara[idx], &RfIn);
                Serial.println(rfCode(&RfIn));
                xQueueSend(udpOut, (void *)&RfIn, 0);
            }
        }
    }
}

void handelGpsData(RoboStruct *gps)
{
    RoboStruct gpsin = {};
    if (xQueueReceive(gpsQue, (void *)&gpsin, 0) == pdTRUE) {
        if (gpsin.gpsFix) { gps->lat = gpsin.lat; gps->lng = gpsin.lng; gps->gpsFix = true; }
        else { gps->gpsFix = false; }
        gps->gpsSat = gpsin.gpsSat;
        buoyPara[0].lat = gps->lat; buoyPara[0].lng = gps->lng; buoyPara[0].gpsFix = gps->gpsFix;
    }
}

void handelSerialData(RoboStruct *ser)
{
    RoboStruct serDataIn = {};
    if (xQueueReceive(serIn, (void *)&serDataIn, 1) == pdTRUE) {
        ser->lastSerIn = millis();
        if (serDataIn.ack == LORAGET || serDataIn.ack == LORAGETACK || serDataIn.ack == LORASET) {
            processIncomingCommand(ser, &serDataIn);
        } else {
            int idx = -1;
            for (int i = 1; i < 3; i++) { if (buoyPara[i].IDs == serDataIn.IDs || buoyPara[i].IDs == 0) { idx = i; break; } }
            if (idx != -1) {
                surgicalUpdate(&buoyPara[idx], &serDataIn);
                Serial.println(rfCode(&serDataIn));
                xQueueSend(udpOut, (void *)&serDataIn, 0);
                if (serDataIn.cmd == SUBDATA) {
                    mainPwrData.ledBb = serDataIn.speedBb; mainPwrData.ledSb = serDataIn.speedSb;
                    xQueueSend(ledPwr, (void *)&mainPwrData, 0);
                }
            }
        }
    }
}

void loop(void)
{
    while (true) {
        handleTimerRoutines(&mainData);
        handelKeyPress(&mainData);
        handelStatus(&mainData, buoyPara);
        handelRfData(&mainData, buoyParaPtrs);
        handelGpsData(&mainData);
        handelSerialData(&mainData);
        buttonLight(&mainData);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
