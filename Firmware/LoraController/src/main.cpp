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

#define STRUCTLEN 5
RoboStruct IDs[STRUCTLEN];
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
    mainData.IDs = 0;
    mainData.IDr = 0;
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
    if (buttonState == HIGH && lastButtonState == LOW)
    {
        lastPressTime = currentTime;
        debounce = true;
        longPressReported = false;
    }

    // Long press detection
    if (buttonState == HIGH && (currentTime - lastPressTime > 3000) && !longPressReported)
    {
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
        Serial.printf("Key pressed %d times status %d\r\n", presses,key->status);
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
        case 10:
            key->cmd = STORE_DECLINATION;
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
// Add buoyID to list
// ***************************************************************************************************
void fillBuoyArr(RoboStruct *IDin)
{
    for (int i = 0; i < STRUCTLEN; i++)
    {
        if (IDin->IDs == IDs[i].IDs)
        {
            return;
        }
        if (IDs[i].IDs == 0)
        {
            Serial.print("added:");
            Serial.print(IDin->IDs, HEX);
            Serial.printf("on pos: %d\r\n", i);
            IDs[i] = *IDin;
            return;
        }
    }
}

// ***************************************************************************************************
// get data
// ***************************************************************************************************
void getBuoyArr(RoboStruct *IDin)
{
    for (int i = 0; i < STRUCTLEN; i++)
    {
        if (IDin->IDs == IDs[i].IDs)
        {
            *IDin = IDs[i];
            return;
        }
    }
    Serial.print("No data!");
}

// ***************************************************************************************************
// Select Next IDs
// ***************************************************************************************************
void getNextValidID(RoboStruct *IDin)
{
    if (IDin->IDs == 1 || IDin->IDs == 0 || IDin->IDs == -1)
    {
        for (int i = 0; i < STRUCTLEN; i++)
        {
            if (IDs[i].IDs != 0)
            {
                *IDin = IDs[i];
                return;
            }
            return;
        }
    }

    // Normal case: find current ID's index
    int start = -1;
    for (int i = 0; i < STRUCTLEN; i++)
    {
        if (IDs[i].IDs == IDin->IDs)
        {
            start = i;
            break;
        }
    }

    if (start == -1)
    {
        return; // Value not found
    }

    // Circular search for next non-zero ID
    for (int i = 1; i < STRUCTLEN; i++)
    {
        int idx = (start + i) % 5;
        if (IDs[idx].IDs != 0)
        {
            *IDin = IDs[idx];
            return; // ✅ Ensure to return immediately after finding the next valid one
        }
    }
}

// ***************************************************************************************************
// Return the position of te data in the struct arry
// ***************************************************************************************************
int posID(RoboStruct *IDin)
{
    if (IDin->IDs == 1 || IDin->IDs == 0 || IDin->IDs == -1)
    {
        return -1;
    }
    // Find the first occurrence of the input value
    for (int i = 0; i < STRUCTLEN; i++)
    {
        if (IDs[i].IDs == IDin->IDs)
        {
            return i;
        }
    }
    Serial.println("Pos not Found");
    return -1;
}

// ***************************************************************************************************
// handle external data input
// ***************************************************************************************************
bool handelRfData(void)
{
    RoboStruct RfIn, RfStore;
    bool updd = false; // update data
    if (xQueueReceive(loraToMain, (void *)&RfIn, 0) == pdTRUE)
    {
        int pos = posID(&RfIn);
        if (pos < 0)
        {
            pos = 0;
        }
        IDs[pos].IDs = RfIn.IDs;
        IDs[pos].status = RfIn.status;
        switch (RfIn.cmd)
        {
        case DIRDIST:
            IDs[pos].tgDir = RfIn.tgDir;   // set target direction
            IDs[pos].tgDir = RfIn.tgDir;   // set target direction
            IDs[pos].tgDist = RfIn.tgDist; // set target distance
            updd = true;                   // data is updated
            break;
        case SUBPWR:
            IDs[pos].speedBb = RfIn.speedBb;   // set speed for bow
            IDs[pos].speedSb = RfIn.speedSb;   // set speed for stern
            IDs[pos].subAccuV = RfIn.subAccuV; // set sub accu voltage
            updd = true;                       // data is updated
            break;
        case DIRSPEED:
            IDs[pos].dirMag = RfIn.dirMag;   // set speed for bow
            IDs[pos].speedBb = RfIn.speedBb; // set speed for bow
            IDs[pos].speedSb = RfIn.speedSb; // set speed for stern
            IDs[pos].speed = RfIn.speed;     // set sub accu voltage
            updd = true;                     // data is updated
            break;
        case DIRMDIRTGDIRG:
            IDs[pos].tgDir = RfIn.dirMag;  // set target direction
            IDs[pos].gpsDir = RfIn.gpsDir; // set target distance
            updd = true;                   // data is updated
            break;
        }
    }
    return updd; // return true if data is updated
}

//***************************************************************************************************
//  Main loop
//***************************************************************************************************
unsigned long remoteTimer = millis();
void loop()
{

    readAdc(&adcmain); // get adc data
    if (mainData.IDs == -1 || mainData.IDs == 0 || mainData.IDs == 1)
    {
        getNextValidID(&mainData);
    }
    switch (adcmain.swPos)
    {
    case SW_LEFT:
        if (mainData.tgDir != adcmain.heading || mainData.tgSpeed != adcmain.speed)
        {
            if (remoteTimer < millis())
            {
                remoteTimer = 1000 + millis();
                mainData.tgDir = adcmain.heading;
                mainData.tgSpeed = adcmain.speed;
                mainData.cmd = REMOTE;
                newLoraDataOut = true; // set flag to send data out
            }
        }
        if (adcmain.swPos != lastWsPos) //
        {                               // Prevent extreem data changes
            lastWsPos = adcmain.swPos;  //
            mainData.tgSpeed = 0;       // set speed
            mainData.cmd = REMOTE;
            newLoraDataOut = true; // set flag to send data out
        }
        break;
    case SW_MID: // Remote control mode

        if (adcmain.swPos != lastWsPos) //
        {
            lastWsPos = adcmain.swPos; //
            mainData.cmd = IDELING;    //
            mainData.ack = LORAGETACK; // ack needed
            newLoraDataOut = true;     // set flag to send data out
        }
        else
        {
            mainData.cmd = -1; //
            int presses = countKeyPressesWithTimeoutAndLongPressDetecton();
            if (presses == 1)
            {
                getNextValidID(&mainData);
            }
        }
        break;
    case SW_RIGHT:                      // Select mode
        if (adcmain.swPos != lastWsPos) //
        {
            lastWsPos = adcmain.swPos; //                           //
            mainData.cmd = IDELING;    //
            mainData.ack = LORAGETACK; // ack needed
            newLoraDataOut = true;     // set flag to send data out
        }
        //***************************************************************************************************
        //      Check front key
        //***************************************************************************************************
        handelKeyPress(&mainData);
        break;
    default:
        newLoraDataOut = false; // set flag to send data out
        break;
    }

    //***************************************************************************************************
    //      Take care of last command
    //***************************************************************************************************
    switch (mainData.cmd)
    {
    case LOCKING:
        mainData.ack = LORAGETACK; // ack needed
        newLoraDataOut = true;     // set flag to send data out
        break;
    case IDELING:
        mainData.ack = LORAGETACK; // ack needed
        newLoraDataOut = true;     // set flag to send data out
        break;
    case DOCKING:
        mainData.ack = LORAGETACK; // ack needed
        newLoraDataOut = true;     // set flag to send data out
        break;
    case STOREASDOC:
        mainData.ack = LORAGETACK; // ack needed
        newLoraDataOut = true;     // set flag to send data out
        break;
    case CALIBRATE_MAGNETIC_COMPASS:
        mainData.ack = LORAGETACK; // ack needed
        newLoraDataOut = true;     // set flag to send data out
        break;
    default:
        break;
    }
    if (newLoraDataOut == true)
    {
        mainData.IDr = mainData.IDs;
        xQueueSend(loraOut, (void *)&mainData, 10); // send to lora
        newLoraDataOut = false;
        mainData.cmd = -1;
    }
    if (handelRfData())
    {
        getBuoyArr(&mainData);
        updateOled(&mainData);
    }
    delay(1);
}