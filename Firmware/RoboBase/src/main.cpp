#include <Arduino.h>
#include <RoboCompute.h>
#include "main.h"
#include "io_base.h"
#include "lorabase.h"
#include "oled_ssd1306.h"
#include "basewifi.h"
#include "leds.h"

// #define BUFLENMHRG 60 // one sampel each sec so 60 sec for stabilisation

unsigned long buoyId = ROBOBASE;
static RoboStruct wind;
static RoboStruct loraTx;
static RoboStruct loraRx;
static RoboStruct mainData;
static RoboStruct loraData;
RoboStruct buoyPara[3] = {};
static unsigned int status = IDLE;
static int cmd;
unsigned long loraTimer = millis();

int buttonState = 0;             // Current state of the button
int lastButtonState = 0;         // Previous state of the button
unsigned long lastPressTime = 0; // Time of the last press
unsigned long debounceDelay = 0; // Debounce time in milliseconds
int pressCount = 0;              // Count the number of button presses
int longPressCount = 0;
bool debounce = false; // Debouncing flag
bool longPressDetected = false;

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
    buttonState = !digitalRead(BUTTON_PIN_SW);
    // Check if the button is pressed and it's a new press (debounce)
    if (buttonState == HIGH && lastButtonState == LOW && !debounce)
    {
        pressCount++;    // Increment the button press count
        debounce = true; // Set debounce flag
        // delay(debounceDelay);        // Simple debouncing by adding a delay
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
    else if ((currentTime - lastPressTime) > 3000 && pressCount == 1 && buttonState == HIGH) // Long press is 3 seconds
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
        printf("Key pressed %d\r\n", presses);
        switch (presses)
        {
        case 1:
            printf("Lock positions\r\n");
            stat = LOCKING;
            break;
        case 2:
            printf("Compute new startline\r\n");
            stat = COMPUTESTART;
            break;
        case 3:
            printf("Compute new track\r\n");
            stat = COMPUTETRACK;
            break;
        case 5:
            printf("IDLE\r\n");
            stat = IDELING;
            break;

        case 10:
            printf("Store pos as DOCKPOS\r\n");
            // stat = DOCK_STORING;
            break;
        case 0x0100: // long press (More than 3 sec)
            printf("Sail to DOCKPOS\r\n");
            stat = DOCKING;
            break;

        default:
            break;
        }
    }
    return stat;
}

//***************************************************************************************************
//      status actions
//***************************************************************************************************
int handelStatus(int status)
{
    String loraString = "";
    switch (status)
    {
    case COMPUTESTART:
        printf("# Computing startline now!!!\r\n");
        buoyPara[3] = calcTrackPos(buoyPara);
        for (int i = 0; i < 3; i++)
        {
            trackPosPrint(buoyPara[i].trackPos);
            printf(" = (%.12f,%.12f) # %lx\r\n", buoyPara[i].tgLat, buoyPara[i].tgLng, buoyPara[i].IDs);
        }
        buoyPara[3] = recalcStarLine(buoyPara);
        if ((buoyPara[0].trackPos != -1 && buoyPara[1].trackPos != -1) || (buoyPara[0].trackPos != -1 && buoyPara[2].trackPos != -1) || (buoyPara[1].trackPos != -1 && buoyPara[2].trackPos != -1))
        {
            status = SENDTRACK;
        }
        else
        {
            status = IDLE;
        }
        break;
    case COMPUTETRACK:
        printf("# Computing new track now!!!\r\n");
        buoyPara[0].wDir = mainData.wDir;
        for (int i = 0; i < 3; i++)
        {
            trackPosPrint(buoyPara[i].trackPos);
            printf(" = (%.12f,%.12f)\r\n", buoyPara[i].tgLat, buoyPara[i].tgLng);
        }
        buoyPara[3] = reCalcTrack(buoyPara);
        if (buoyPara[0].trackPos == -1 || buoyPara[1].trackPos == -1 || buoyPara[2].trackPos == -1)
        {
            status = IDLE;
        }
        else
        {
            status = SENDTRACK;
        }
        break;
    case SENDTRACK:
        printf("# Sending data out now!!!\r\n");
        for (int i = 0; i < 3; i++)
        {
            if (buoyPara[i].trackPos != 0 && buoyPara[i].IDs != 0)
            {
                memcpy(&loraTx, &buoyPara[i], sizeof(RoboStruct));
                trackPosPrint(buoyPara[i].trackPos);
                printf("n = (%.12f,%.12f)# IDr:%lx \r\n", buoyPara[i].tgLat, buoyPara[i].tgLng, buoyPara[i].IDs);
                loraTx.IDr = buoyPara[i].IDs;
                loraTx.IDs = buoyId;
                loraTx.cmd = SETLOCKPOS;
                loraTx.ack = LORAGETACK;
                delay(25);
                xQueueSend(loraOut, (void *)&loraTx, 10); // send out trough Lora
            }
        }
        status = LOCKED;
        break;
    case DOCKING:
        loraTx.IDr = BUOYIDALL;
        loraTx.IDs = buoyId;
        loraTx.cmd = DOCKING;
        loraTx.ack = LORAINF;
        xQueueSend(loraOut, (void *)&loraTx, 10); // send out trough Lora
        for (int i = 0; i < 3; i++)
        {
            if (buoyPara[i].mac != 0)
            {
                loraTx.IDr = buoyPara[i].mac;
                loraTx.IDs = buoyId;
                loraTx.cmd = DOCKING;
                loraTx.ack = LORAGETACK;
                xQueueSend(loraOut, (void *)&loraTx, 10); // send out trough Lora
            }
        }
        status = DOCKED;
        break;
    case IDELING:
        loraTx.IDs = buoyId;
        loraTx.cmd = IDELING;
        loraTx.ack = LORAGETACK;
        for (int i = 0; i < 3; i++)
        {
            if (buoyPara[i].IDs != 0)
            {
                loraTx.IDr = buoyPara[i].IDs;
                xQueueSend(loraOut, (void *)&loraTx, 10); // send out trough Lora
            }
        }
        loraTx.IDr = BUOYIDALL;
        loraTx.IDs = buoyId;
        loraTx.cmd = IDELING;
        loraTx.ack = LORASET;
        for (int i = 0; i < 3; i++)
        {
            xQueueSend(loraOut, (void *)&loraTx, 10); // send out trough Lora
            delay(150);
        }
        status = IDLE;
        break;
    case LOCKING:
        for (int i = 0; i < 3; i++)
        {
            if (buoyPara[i].IDs != 0)
            {
                loraTx.IDr = buoyPara[i].IDs;
                loraTx.IDs = buoyId;
                loraTx.cmd = LOCKING;
                loraTx.ack = LORAGETACK;
                xQueueSend(loraOut, (void *)&loraTx, 10); // send out trough Lora
            }
        }
        loraTx.IDr = BUOYIDALL;
        loraTx.IDs = buoyId;
        loraTx.cmd = LOCKING;
        loraTx.ack = LORASET;
        for (int i = 0; i < 3; i++)
        {
            xQueueSend(loraOut, (void *)&loraTx, 10); // send out trough Lora
            delay(150);
        }
        status = IDLE;
        break;
    default:
        break;
    }
    return status;
}
//***************************************************************************************************
//  Setup
//***************************************************************************************************
void setup()
{
    Serial.begin(115200);
    pinMode(BUTTON_PIN_SW, INPUT_PULLUP);
    pinMode(BUTTON_PIN_GND, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BULETINLED, OUTPUT);
    digitalWrite(BUTTON_PIN_GND, LOW);
    digitalWrite(BULETINLED, LOW);
    digitalWrite(LED_PIN, HIGH);
    initSSD1306();
    initloraqueue();
    initwifiqueue();
    initledqueue();
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8000, NULL, configMAX_PRIORITIES - 5, NULL, 0);
    xTaskCreatePinnedToCore(LedTask, "WiFiTask", 2000, NULL, 10, NULL, 1);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    Serial.println("Main task running!");
}

//***************************************************************************************************
//  Main loop
//***************************************************************************************************
void loop(void)
{
    int presses = -1;
    int stat = 0;
    while (true)
    {
        //***************************************************************************************************
        //      Check front key
        //***************************************************************************************************
        status = handelKeyPress(status);
        //***************************************************************************************************
        //      status actions
        //***************************************************************************************************
        status = handelStatus(status);
        //***************************************************************************************************
        //      New Lora data
        //***************************************************************************************************
        if (xQueueReceive(loraIn, (void *)&loraRx, 10) == pdTRUE) // New lora data
        {
            if (loraRx.cmd == LOCKPOS)
            {
                buoyPara[3] = AddDataToBuoyBase(loraRx, buoyPara);
            }
        }
        vTaskDelay(1);
    }
}
