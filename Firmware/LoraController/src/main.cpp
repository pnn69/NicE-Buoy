/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
https://www.lilygo.cc/products/lora3
https://github.com/Xinyuan-LilyGO/TTGO-LoRa-Series
https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/T3_V1.6.1.pdf
433MHz is SX1278
*/
#include <Arduino.h>
// #include <WiFi.h>
// #include "freertos/task.h"
#include <math.h>
#include <Wire.h>
#include "io.h"
#include "oled_ssd1306.h"
#include "LiLlora.h"
#include "sercom.h"
#include "controlwifi.h"
#include "adc.h"
#include "../../RobobuoyDependency\RobobuoyVersion.h"

RoboStruct mainData = {};
RoboStruct displayData = {};
static adcDataType adcmain = {0, 0, 0, 0, false}; // adc data
bool newLoraDataOut = false;
int lastWsPos = 0;
// Button stuff
bool debounce = false;
bool buttonState = false;
bool lastButtonState = false;
bool longPressReported = false;
unsigned long lastPressTime = 0;
unsigned long debounceDelay = 0;
int pressCount = 0;

//***************************************************************************************************
//  Setup
//***************************************************************************************************
void setup()
{
    // SerialBT.begin("NicE_Buoy_Control"); // Bluetooth device name
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, false);
    pinMode(SW_P_1, INPUT_PULLUP);
    pinMode(SW_P_2, OUTPUT);
    digitalWrite(SW_P_2, false);
    initSSD1306();
    Wire.begin();
    initloraqueue();
    initserqueue();
    printf("Robobuoy Sub Version: %0.1f Sub Build: %s %s\r\n", SUBVERSION, __DATE__, __TIME__);
    printf("Robobuoy ID: %08x\r\n", espMac());
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    xTaskCreatePinnedToCore(SercomTask, "SerialTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    mainData.IDs = espMac(); // set my id
    mainData.status = IDLE;
    mainData.cmd = IDLE;
    mainData.cmd = -1;
    readAdc(&adcmain); // get adc data
    lastWsPos = adcmain.swPos;
}

//***************************************************************************************************
//      keypress detection
//***************************************************************************************************
int countKeyPressesWithTimeoutAndLongPressDetecton()
{
    unsigned long currentTime = millis();
    buttonState = !digitalRead(SW_P_1); // Active HIGH

    // Debounce
    if (currentTime < debounceDelay)
        return -1;

    // Start of a new press
    if (buttonState == HIGH && lastButtonState == LOW) {
        lastPressTime = currentTime;
        debounce = true;
        longPressReported = false;
    }

    // Long press detection
    if (buttonState == HIGH && (currentTime - lastPressTime > 3000) && !longPressReported) {
        int result = 101 + pressCount;  // 100 + short press count
        pressCount = 0;
        longPressReported = true;
        return result;
    }

    // Count short presses on release
    if (buttonState == LOW && lastButtonState == HIGH && debounce) {
        if (!longPressReported) {
            pressCount++;
        }
        debounce = false;
        debounceDelay = currentTime + 50;
    }

    // Timeout for short press sequence (500 ms)
    if ((currentTime - lastPressTime) > 500 && pressCount > 0 && buttonState == LOW && !longPressReported) {
        int result = pressCount;
        pressCount = 0;
        return result;
    }

    lastButtonState = buttonState;
    return -1;
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
        Serial.printf("Key pressed %d times\r\n", presses);
        switch (presses)
        {
        case 1: // lock / unlock
            if ((key->status == LOCKED) || (key->status == DOCKED))
            {
                key->cmd = IDELING;
            }
            else
            {
                key->cmd = LOCKING;
            }
            break;
        case 5:
            key->cmd = DOCKING;
            break;
        case 105:
            key->cmd = STOREASDOC;
            break;
        case 110:
            key->cmd = CALIBRATE_MAGNETIC_COMPASS;
            break;
        // case 0x0100:
        //     key->cmd = STOREASDOC;
        //     break;
        default:
            break;
        }
    }
}

// ***************************************************************************************************
// handle external data input
// ***************************************************************************************************
bool handelRfData(RoboStruct *RfOut)
{
    bool updd = false; // update data
    RoboStruct RfIn;
    if (xQueueReceive(loraIn, (void *)&RfIn, 1) == pdTRUE) // new lora data
    {
        if (RfIn.IDr == BUOYIDALL || RfIn.IDr == RfOut->mac) // check if data is for me
        {
            RfOut->status = RfIn.status; // set status
            switch (RfIn.cmd)
            {
            case DIRDIST:
                RfOut->tgDir = RfIn.tgDir;   // set target direction
                RfOut->tgDist = RfIn.tgDist; // set target distance
                updd = true;                 // data is updated
                break;
            case SUBPWR:
                RfOut->speedBb = RfIn.speedBb;   // set speed for bow
                RfOut->speedSb = RfIn.speedSb;   // set speed for stern
                RfOut->subAccuV = RfIn.subAccuV; // set sub accu voltage
                updd = true;                     // data is updated
                break;
            }
        }
    }
    return updd; // return true if data is updated
}

//***************************************************************************************************
//  Main loop
//***************************************************************************************************
void loop()
{

    readAdc(&adcmain); // get adc data
    switch (adcmain.swPos)
    {
    case SW_LEFT:
        if (mainData.dirSet != adcmain.rudder || mainData.speedSet != adcmain.speed)
        {
            if (adcmain.swPos != lastWsPos)        //
            {                                      // Prevent extreem data changes
                mainData.dirSet = adcmain.rudder;  // set rudder
                mainData.speedSet = adcmain.speed; // set speed
                mainData.speedBb = 0;              //
                mainData.speedSb = 0;              //
                lastWsPos = adcmain.swPos;         //
            }
            mainData.dirSet = adcmain.rudder;                  // set rudder
            mainData.speedSet = adcmain.speed;                 // set speed
            mainData.status = REMOTE;                          // set status to remote control
            mainData.cmd = REMOTE;                             //
            mainData.IDr = BUOYIDALL;                          // send to all CHANGE if more buoys are used
            mainData.speedBb = adcmain.speed + adcmain.rudder; //
            mainData.speedSb = adcmain.speed - adcmain.rudder; //
            mainData.ack = LORAINF;                            // no ack needed
            newLoraDataOut = true;                             // set flag to send data out
        }
        break;
    case SW_MID:                        // Remote control mode
        if (adcmain.swPos != lastWsPos) //
        {
            lastWsPos = adcmain.swPos; //
            mainData.status = IDLE;    // set status to remote control
            mainData.cmd = IDELING;    //
            mainData.IDr = BUOYIDALL;  // send to all CHANGE if more buoys are used
            mainData.ack = LORAGETACK; // ack needed
            newLoraDataOut = true;     // set flag to send data out
        }
        break;
    case SW_RIGHT:                      // Select mode
        if (adcmain.swPos != lastWsPos) //
        {
            lastWsPos = adcmain.swPos; //                           //
            mainData.status = IDLE;    // set status to remote control
            mainData.cmd = IDELING;    //
            mainData.IDr = BUOYIDALL;  // send to all CHANGE if more buoys are used
            mainData.ack = LORAGETACK; // ack needed
            newLoraDataOut = true;     // set flag to send data out
        }
        break;
        default:
        break;
    }
    handelKeyPress(&mainData);
    //***************************************************************************************************
    //      Check front key
    //***************************************************************************************************
    switch (mainData.cmd)
    {
    case LOCKING:
        mainData.IDr = BUOYIDALL;  // send to all CHANGE if more buoys are used
        mainData.status = LOCKED;  // set status to locked
        mainData.ack = LORAGETACK; // ack needed
        newLoraDataOut = true;     // set flag to send data out
        break;
    case DOCKING:
        mainData.IDr = BUOYIDALL;  // send to all CHANGE if more buoys are used
        mainData.status = DOCKED;  // set status to locked
        mainData.ack = LORAGETACK; // ack needed
        newLoraDataOut = true;     // set flag to send data out
        break;
    case STOREASDOC:
        mainData.IDr = BUOYIDALL;  // send to all CHANGE if more buoys are used
        mainData.status = STOREASDOC;  // set status to locked
        mainData.ack = LORAGETACK; // ack needed
        newLoraDataOut = true;     // set flag to send data out
        break;
    case CALIBRATE_MAGNETIC_COMPASS:
        mainData.IDr = BUOYIDALL;  // send to all CHANGE if more buoys are used
        mainData.status = STOREASDOC;  // set status to locked
        mainData.ack = LORAGETACK; // ack needed
        newLoraDataOut = true;     // set flag to send data out
        break;
    case IDELING:
        mainData.cmd = IDELING;
        mainData.IDr = BUOYIDALL;  // send to all CHANGE if more buoys are used
        mainData.status = IDLE;    // set status to locked
        mainData.ack = LORAGETACK; // ack needed
        newLoraDataOut = true;     // set flag to send data out
        break;
    case REMOTE:
        newLoraDataOut == false;
        break;
    }
    if (newLoraDataOut == true && mainData.cmd != -1)
    {
        xQueueSend(loraOut, (void *)&mainData, 10); // send to main
        newLoraDataOut == false;
    }
    if (handelRfData(&displayData))
    {
        updateOled(&displayData);
    }
    mainData.cmd = -1;
    delay(1);
}