/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
https://www.lilygo.cc/products/lora3
https://github.com/Xinyuan-LilyGO/TTGO-LoRa-Series
https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/T3_V1.6.1.pdf
433MHz is SX1278
*/
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "io.h"
#include "oled_ssd1306.h"
#include "LiLlora.h"
#include "sercom.h"
#include "controlwifi.h"
#include "adc.h"
#include "../../RoboDependency\RobobuoyVersion.h"

#define STRUCTLEN 5
RoboStruct IDs[STRUCTLEN];
RoboStruct mainData = {};
static adcDataType adcmain = {0, 0, 0, 0, false}; // adc data
static int lastPhysicalSwitchPos = -1;

// Button stuff
bool lastButtonState = false;
bool longPressReported = false;
unsigned long lastPressTime = 0;
unsigned long debounceDelay = 0;
int pressCount = 0;

static int wifiConfig = 0;

// Forward declarations
int countKeyPressesWithTimeoutAndLongPressDetecton();
void getNextValidID(RoboStruct *IDin);
void getBuoyArr(RoboStruct *IDin);
bool handelRfData(void);
void handelKeyPress(RoboStruct *key);
void dispatchCommand(RoboStruct *data, adcDataType *adc);

//***************************************************************************************************
//  Command Dispatcher
//***************************************************************************************************
void dispatchCommand(RoboStruct *data, adcDataType *adc)
{
    bool forceSend = false;

    // 1. Detect Switch Change (Edge-Triggered)
    if (adc->swPos != lastPhysicalSwitchPos)
    {
        lastPhysicalSwitchPos = adc->swPos;
        forceSend = true;
        
        data->ack = LORAGETACK; // 3
        data->status = IDLE;    // 7
        
        switch (adc->swPos)
        {
        case SW_LEFT:  data->cmd = REMOTE;  break; // 25
        case SW_MID:   data->cmd = IDELING; break; // 8
        case SW_RIGHT: data->cmd = LOCKING; break; // 12
        }
        printf("Switch moved! New Cmd: %d\n", data->cmd);

        int pos = posID(data);
        if (pos >= 0) IDs[pos].status = data->cmd; // locally save the real intent
    }

    // 2. Handle Continuous Remote Data (Direction/Speed)
    if (data->status == REMOTE || data->cmd == REMOTE)
    {
        data->tgDir = adc->heading;
        data->tgSpeed = adc->speed;
        data->speed = adc->speed;
        
        if (adc->newdata)
        {
            data->cmd = REMOTE; // 25
            forceSend = true;
            data->ack = LORAINF; // 6 (Continuous data update)
            adc->newdata = false;
        }
    }

    // 3. Dispatch to Queues ONLY if an event occurred (Physical move or Pot change)
    if (forceSend)
    {
        data->IDr = data->IDs; // Ensure target is set to the selected buoy
        
        static int lastSentCmd = -1;
        static int lastSentStatus = -1;
        static unsigned long lastSendTime = 0;

        // Filter duplicates if it's NOT a continuous REMOTE stream
        if (data->ack == LORAGETACK) 
        {
            if (data->cmd == lastSentCmd && data->status == lastSentStatus && (millis() - lastSendTime < 3000))
            {
                // Already sent this exact mode command recently, skip re-queueing to avoid UDP/LoRa loop bounces
                data->cmd = NOCMD;
                data->ack = 0;
                return;
            }
            lastSentCmd = data->cmd;
            lastSentStatus = data->status;
            lastSendTime = millis();
        }

        printf("dispatchCommand: target IDr = %08x, CMD = %d, STATUS = %d\n", data->IDr, data->cmd, data->status);

        if (loraOut != NULL) xQueueSend(loraOut, (void *)data, 10);
        if (udpOut != NULL) xQueueSend(udpOut, (void *)data, 10);
        
        // Clear command to avoid re-sending same packet
        data->cmd = NOCMD;
        data->ack = 0;
    }
}

//***************************************************************************************************
//  Setup
//***************************************************************************************************
void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, false);
    pinMode(SW_P_1, INPUT_PULLUP);
    pinMode(SW_P_2, OUTPUT);
    digitalWrite(SW_P_2, false);
    Wire.begin();
    initSSD1306();
    initloraqueue();
    initserqueue();
    initwifiqueue();
    printf("Robobuoy Lora Version: %0.1f Sub Build: %s %s\r\n", SUBVERSION, __DATE__, __TIME__);

    printf("Robobuoy ID: %08x\r\n", espMac());
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    xTaskCreatePinnedToCore(SercomTask, "SerialTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 4000, (void *)&wifiConfig, configMAX_PRIORITIES - 2, NULL, 0);
    mainData.IDs = 0;
    mainData.IDr = 0;
    mainData.status = IDLE;
    mainData.cmd = NOCMD;
    readAdc(&adcmain);
    lastPhysicalSwitchPos = adcmain.swPos;
}

//***************************************************************************************************
//      keypress detection
//***************************************************************************************************
int countKeyPressesWithTimeoutAndLongPressDetecton()
{
    unsigned long currentTime = millis();
    bool pressed = !digitalRead(SW_P_1);
    int result = -1;

    if (pressed && !lastButtonState) {
        lastPressTime = currentTime;
        longPressReported = false;
    } 
    else if (!pressed && lastButtonState) {
        if (!longPressReported) {
            pressCount++;
            debounceDelay = currentTime + 500;
        }
    }

    if (pressed && !longPressReported && (currentTime - lastPressTime > 3000)) {
        result = 101 + pressCount;
        pressCount = 0;
        longPressReported = true;
    }

    if (!pressed && pressCount > 0 && (currentTime > debounceDelay)) {
        result = pressCount;
        pressCount = 0;
    }

    lastButtonState = pressed;
    return result;
}

//***************************************************************************************************
//      key press handler
//***************************************************************************************************
void handelKeyPress(RoboStruct *key)
{
    int presses = countKeyPressesWithTimeoutAndLongPressDetecton();
    if (presses > 0)
    {
        Serial.printf("Key pressed %d times status %d\r\n", presses, key->status);
        
        if (presses == 1) // 1 Click: Cycle Buoy
        {
            getNextValidID(key);
            lastPhysicalSwitchPos = adcmain.swPos; // Latch
            Serial.printf("Switched to Buoy: %08x\n", key->IDs);
            return;
        }

        switch (presses)
        {
        case 5: 
            if (key->status != REMOTE) {
                key->cmd = DOCKING;
                key->ack = LORAGETACK;
            }
            break;
        case 101: 
            key->cmd = STOREASDOC; 
            key->ack = LORAGETACK;
            break; // Long press
        case 105: 
            key->cmd = STOREASDOC; 
            key->ack = LORAGETACK;
            break; // 4 clicks + Long
        case 102: 
            if (key->status == LOCKED) {
                key->cmd = COMPUTESTART;
                key->ack = LORAGETACK;
            }
            break; // 1 click + Long
        case 10: 
            key->cmd = STORE_DECLINATION; 
            key->ack = LORAGETACK;
            break;
        case 110: 
            key->cmd = CALIBRATE_MAGNETIC_COMPASS; 
            key->ack = LORAGETACK;
            break;
        }
        
        // Signal to dispatchCommand that a button event occurred
        if (loraOut != NULL) xQueueSend(loraOut, (void *)key, 10);
        if (udpOut != NULL) xQueueSend(udpOut, (void *)key, 10);
        key->cmd = NOCMD;
    }
}

// ***************************************************************************************************
// Buoy Management Functions
// ***************************************************************************************************
void fillBuoyArr(RoboStruct *IDin)
{
    for (int i = 0; i < STRUCTLEN; i++)
    {
        if (IDin->IDs == IDs[i].IDs) return;
        if (IDs[i].IDs == 0) { IDs[i] = *IDin; return; }
    }
}

void getBuoyArr(RoboStruct *IDin)
{
    for (int i = 0; i < STRUCTLEN; i++)
    {
        if (IDin->IDs == IDs[i].IDs) { *IDin = IDs[i]; return; }
    }
}

void getNextValidID(RoboStruct *IDin)
{
    int start = -1;
    for (int i = 0; i < STRUCTLEN; i++) { if (IDs[i].IDs == IDin->IDs) { start = i; break; } }
    if (start == -1 && IDin->IDs != 0) return;
    for (int i = 1; i <= STRUCTLEN; i++) {
        int idx = (start + i) % STRUCTLEN;
        if (IDs[idx].IDs != 0) { *IDin = IDs[idx]; return; }
    }
}

int posID(RoboStruct *IDin)
{
    for (int i = 0; i < STRUCTLEN; i++) { if (IDs[i].IDs == IDin->IDs) return i; }
    return -1;
}

void processData(RoboStruct *RfIn)
{
    int pos = posID(RfIn);
    if (pos < 0) { fillBuoyArr(RfIn); pos = posID(RfIn); }
    if (pos < 0) return;

    // 1. Update status and handle IDLE state forcing
    if (RfIn->cmd == SUBDATA || RfIn->cmd == TOPDATA || RfIn->cmd == BUOYPOS || 
        RfIn->cmd == LOCKED || RfIn->cmd == DOCKED || RfIn->cmd == IDELING || 
        RfIn->cmd == REMOTE || RfIn->cmd == NOCMD)
    {
        IDs[pos].status = RfIn->status;
        
        // If buoy is IDLE, force bars to zero immediately
        if (RfIn->status == IDLE || RfIn->status == IDELING) {
            IDs[pos].speedBb = 0;
            IDs[pos].speedSb = 0;
        }
    }

    // 2. Motor Power: ONLY update from commands that actually contain Bb/Sb fields in RoboDecode
    if (RfIn->cmd == SUBDATA || RfIn->cmd == TOPDATA || RfIn->cmd == SPBBSPSB)
    {
        IDs[pos].speedBb = RfIn->speedBb;
        IDs[pos].speedSb = RfIn->speedSb;
    }

    // 3. Heading
    if (RfIn->cmd == SUBDATA || RfIn->cmd == TOPDATA || RfIn->cmd == BUOYPOS || RfIn->cmd == DIRSPEED || RfIn->cmd == MDIR)
    {
        IDs[pos].dirMag = RfIn->dirMag;
    }

    // 4. GPS info: ONLY update from commands that carry GPS fields
    if (RfIn->cmd == TOPDATA || RfIn->cmd == BUOYPOS)
    {
        IDs[pos].gpsFix = RfIn->gpsFix;
        IDs[pos].gpsSat = RfIn->gpsSat;
        IDs[pos].lat = RfIn->lat;
        IDs[pos].lng = RfIn->lng;
    }

    // 5. Targets
    if (RfIn->cmd == TOPDATA || RfIn->cmd == LOCKED || RfIn->cmd == DOCKED || 
        RfIn->cmd == DIRDIST || RfIn->cmd == CALCRUDDER || RfIn->cmd == TGDIRSPEED)
    {
        IDs[pos].tgDir = RfIn->tgDir;
        IDs[pos].tgDist = RfIn->tgDist;
    }

    // 6. Battery: Update only if command carries power data
    if (RfIn->cmd == SUBDATA || RfIn->cmd == TOPDATA || RfIn->cmd == SUBACCU || RfIn->cmd == SUBPWR)
    {
        IDs[pos].subAccuV = RfIn->subAccuV;
        IDs[pos].subAccuP = RfIn->subAccuP;
    }
}

bool handelRfData(void)
{
    RoboStruct RfIn;
    bool updd = false;
    if (xQueueReceive(loraToMain, (void *)&RfIn, 0) == pdTRUE) { processData(&RfIn); updd = true; }
    if (xQueueReceive(udpIn, (void *)&RfIn, 0) == pdTRUE) { processData(&RfIn); updd = true; }
    return updd;
}

//***************************************************************************************************
//  Main loop
//***************************************************************************************************
void loop()
{
    if (mainData.IDs == 0) getNextValidID(&mainData);

    readAdc(&adcmain);
    handelKeyPress(&mainData);
    if (handelRfData()) getBuoyArr(&mainData);
    dispatchCommand(&mainData, &adcmain);
    updateOled(&mainData, &adcmain);

    vTaskDelay(pdMS_TO_TICKS(50));
}
