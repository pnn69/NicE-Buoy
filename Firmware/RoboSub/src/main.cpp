/**
 * @file main.cpp
 * @brief Primary Entry Point and Navigation Loop for NicE-Buoy Sub.
 */

#include <Arduino.h>
#include <Wire.h>
#include <RoboTone.h>
#include <RoboCompute.h>
#include "main.h"
#include "freertos/task.h"
#include "io_sub.h"
#include "subwifi.h"
#include "datastorage.h"
#include "leds.h"
#include "esc.h"
#include "compass.h"
#include "buzzer.h"
#include "adc.h"
#include "sercom.h"
#include "pidrudspeed.h"
#include "soc/rtc_cntl_reg.h"

#define HOST_NAME "RoboBuoySub"
TaskHandle_t compassTaskHandle = NULL;

// Global navigation data
RoboStruct mainData;
SemaphoreHandle_t mainDataMutex = NULL;
Message escOut;

static LedData mainLedStatus;
static unsigned long PwrOff = 0;
static unsigned long pidTimer = 0;
static unsigned long logtimer = millis();
static unsigned long nextSamp = millis();

// Forward declarations
void handleStatus(RoboStruct *stat);
void handleTimerRoutines(RoboStruct *in);
void handleSerandRfdata(RoboStruct *ser);
void handleSerialTimeOut(RoboStruct *ser);

void setup()
{
    mainDataMutex = xSemaphoreCreateMutex();
    Serial.begin(115200);
    delay(100);
    Serial.println("\r\nNicE-Buoy Sub Booting...\r\n");

    pinMode(PWRENABLE, OUTPUT);
    digitalWrite(PWRENABLE, 1);
    delay(500); 
    Wire.begin(21, 22);
    Wire.setClock(100000);

    initledqueue();
    initbuzzerqueue();
    initcompassQueue();
    initserqueue();
    initwifi();
    initMemory();
    
    // Load persistent data
    pidRudderParameters(&mainData, GET);
    pidSpeedParameters(&mainData, GET);
    speedMaxMin(&mainData, GET);
    CompasOffset(&mainData, GET);
    
    computeParameters(&mainData, GET);
    thrusterInversion(&mainData, GET);
    thrusterSwap(&mainData, GET);
    MechanicalCorrection(&mainData.mechanicCorrection, GET);

    mainData.mac = espMac();
    mainData.IDs = mainData.mac;
    Serial.printf("Robobuoy ID: %08x\r\n", (unsigned int)mainData.mac);

    InitCompass();
    initRudPid(&mainData);
    initSpeedPid(&mainData);
    initescqueue();

    int wifiConfig = 0;
    pinMode(BUTTON_PIN, INPUT);
    if (digitalRead(BUTTON_PIN) == LOW) { delay(100); if (digitalRead(BUTTON_PIN) == LOW) wifiConfig = 1; }

    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8192, &wifiConfig, 1, NULL, 0);
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(EscTask, "EscTask", 2400, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(CompassTask, "CompassTask", 8192, NULL, configMAX_PRIORITIES - 1, &compassTaskHandle, 1);
    xTaskCreatePinnedToCore(SercomTask, "SerialTask", 8192, NULL, configMAX_PRIORITIES - 3, NULL, 1);

    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    PwrOff = millis();
    mainData.status = IDLE;
}

void loop()
{
    while (true)
    {
        if (xSemaphoreTake(mainDataMutex, portMAX_DELAY)) 
        {
            rudderPid(&mainData);
            speedPid(&mainData);
            handleStatus(&mainData);
            handleTimerRoutines(&mainData);
            xQueueReceive(compass, (void *)&mainData.dirMag, 0);
            handleSerandRfdata(&mainData);
            handleSerialTimeOut(&mainData);
            
            if (logtimer < millis())
            {
                logtimer = millis() + 1000;
                battVoltage(mainData.subAccuV, mainData.subAccuP);
                battCurrent(mainData.subAccuI);
                printf("V31 ID:%08X ST:%d Lat:%.8f Lon:%.8f mDir:%.0f bb:%03d Sb:%03d\r\n", 
                        (unsigned int)mainData.mac, mainData.status, mainData.lat, mainData.lng, mainData.dirMag,
                        mainData.speedBb, mainData.speedSb);
            }
            xSemaphoreGive(mainDataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void handleStatus(RoboStruct *stat)
{
    if (stat->status == IDELING) {
        stat->speed = 0; stat->status = IDLE;
        xQueueSend(serOut, (void *)stat, 10);
    }
}

void handleTimerRoutines(RoboStruct *in)
{
    if (pidTimer < millis()) {
        pidTimer = millis() + 20;
        escOut.speedbb = in->speedBb; escOut.speedsb = in->speedSb;
        xQueueSend(escspeed, (void *)&escOut, 10);
    }
    if (nextSamp < millis()) {
        nextSamp = 250 + millis();
        in->cmd = SUBDATA; in->IDs = in->mac;
        xQueueSend(serOut, (void *)in, 10);
    }
}

void handleSerandRfdata(RoboStruct *ser)
{
    RoboStruct dataIn = {};
    if (xQueueReceive(serIn, (void *)&dataIn, 0) == pdTRUE) 
    {
        ser->lastSerIn = millis(); PwrOff = millis();
        switch (dataIn.cmd)
        {
        case IDLE: case IDELING:
            ser->status = IDELING; initRudPid(ser); initSpeedPid(ser); break;
        case DIRDIST:
            if (ser->status != LOCKED) { initRudPid(ser); initSpeedPid(ser); ser->status = LOCKED; }
            ser->tgDir = dataIn.tgDir; ser->tgDist = dataIn.tgDist; break;
        case PIDRUDDERSET:
            pidRudderParameters(&dataIn, SET); pidRudderParameters(ser, GET); initRudPid(ser);
            { RoboStruct res = *ser; res.IDr = dataIn.IDs; res.IDs = ser->mac; res.cmd = PIDRUDDERSET; res.ack = LORAINF; xQueueSend(serOut, (void *)&res, 10); }
            break;
        case PIDSPEEDSET:
            pidSpeedParameters(&dataIn, SET); pidSpeedParameters(ser, GET); initSpeedPid(ser);
            { RoboStruct res = *ser; res.IDr = dataIn.IDs; res.IDs = ser->mac; res.cmd = PIDSPEEDSET; res.ack = LORAINF; xQueueSend(serOut, (void *)&res, 10); }
            break;
        case STORE_COMPASS_OFFSET:
            CompasOffset(&dataIn, SET); MechanicalCorrection(&dataIn.mechanicCorrection, SET);
            ser->compassOffset = dataIn.compassOffset; ser->icmCompassOffset = dataIn.icmCompassOffset; ser->mechanicCorrection = dataIn.mechanicCorrection;
            { RoboStruct res = *ser; res.IDr = dataIn.IDs; res.IDs = ser->mac; res.cmd = STORE_COMPASS_OFFSET; res.ack = LORAINF; xQueueSend(serOut, (void *)&res, 10); }
            break;
        case SETUPDATA:
            if (dataIn.ack == LORAGET || dataIn.ack == LORAGETACK) {
                RoboStruct res = *ser; res.IDr = dataIn.IDs; res.IDs = ser->mac; res.cmd = SETUPDATA; res.ack = LORAINF;
                pidRudderParameters(&res, GET); pidSpeedParameters(&res, GET); speedMaxMin(&res, GET);
                CompasOffset(&res, GET); computeParameters(&res, GET);
                thrusterInversion(&res, GET); MechanicalCorrection(&res.mechanicCorrection, GET);
                xQueueSend(serOut, (void *)&res, 10);
            } else if (dataIn.ack == LORASET) {
                pidRudderParameters(&dataIn, SET); pidSpeedParameters(&dataIn, SET);
                speedMaxMin(&dataIn, SET); CompasOffset(&dataIn, SET); computeParameters(&dataIn, SET); thrusterInversion(&dataIn, SET); MechanicalCorrection(&dataIn.mechanicCorrection, SET);
                
                pidRudderParameters(ser, GET); pidSpeedParameters(ser, GET); speedMaxMin(ser, GET);
                CompasOffset(ser, GET); computeParameters(ser, GET);
                thrusterInversion(ser, GET); MechanicalCorrection(&ser->mechanicCorrection, GET);
                initRudPid(ser); initSpeedPid(ser);
                
                RoboStruct res = *ser; res.IDr = dataIn.IDs; res.IDs = ser->mac; res.cmd = SETUPDATA; res.ack = LORAINF;
                xQueueSend(serOut, (void *)&res, 10);
            }
            break;
        }
    }
}

void handleSerialTimeOut(RoboStruct *ser)
{
    if (ser->lastSerIn + 5000 < millis()) {
        if (mainLedStatus.color != CRGB::Red) {
            mainLedStatus.color = CRGB::Red; mainLedStatus.blink = BLINK_SLOW;
            xQueueSend(ledStatus, (void *)&mainLedStatus, 10);
            ser->status = IDELING;
        }
    }
    if (PwrOff + 1000 * 60 * 5 < millis()) digitalWrite(PWRENABLE, 0);
}
