#include <Arduino.h>
#include <RoboCompute.h>
#include "main.h"
#include "io_base.h"
#include "lorabase.h"
#include "roboRaceTrack.h"
#include "oled_ssd1306.h"
#include "basewifi.h"

// #define BUFLENMHRG 60 // one sampel each sec so 60 sec for stabilisation

unsigned long buoyId = -2;
static RoboWindStruct wind;
static lorabuf loraTx;
static lorabuf loraRx;
static char loraTransmitt[MAXSTRINGLENG];
static RoboStruct mainData;
static RoboStruct loraData;
RoboStruct buoyPara[3] = {};
static unsigned int status = IDLE;
static int msg;
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
    buttonState = digitalRead(BUTTON_PIN);
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
        calcTrackPos(buoyPara);
        for (int i = 0; i < 3; i++)
        {
            trackPosPrint(buoyPara[i].trackPos);
            printf(" = (%.12f,%.12f)\r\n", buoyPara[i].tgLat, buoyPara[i].tgLng);
        }
        buoyPara[3] = recalcStarLine(buoyPara);
        if ((buoyPara[0].trackPos != -1 && buoyPara[1].trackPos != -1) || (buoyPara[0].trackPos != -1 && buoyPara[2].trackPos != -1) || (buoyPara[1].trackPos != -1 && buoyPara[2].trackPos != -1))
        {
            status = LORASENDTRACK;
        }
        else
        {
            status = IDLE;
        }
        break;
    case COMPUTETRACK:
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
            status = LORASENDTRACK;
        }
        break;
    case LORASENDTRACK:
        for (int i = 0; i < 3; i++)
        {
            if (buoyPara[i].trackPos != 0)
            {
                trackPosPrint(buoyPara[i].trackPos);
                printf("n = (%.12f,%.12f)\r\n", buoyPara[i].tgLat, buoyPara[i].tgLng);
                loraTx.macIDr = buoyPara[i].mac;
                loraTx.macIDs = buoyId;
                loraTx.msg = LORALOCKPOS;
                loraTx.ack = LORAGETACK;
                loraString = RoboCode(buoyPara[i], LORALOCKPOS);
                loraString.toCharArray(loraTx.data, loraString.length() + 1);
                xQueueSend(loraOut, (void *)&loraTx, 10); // send out trough Lora
            }
        }
        status = LOCKED;
        break;
    case DOCKING:
        loraTx.macIDr = BUOYIDALL;
        loraTx.macIDs = buoyId;
        loraTx.msg = DOCKING;
        loraTx.ack = LORAINF;
        loraString = RoboCode(buoyPara[0], DOCKING);
        loraString.toCharArray(loraTx.data, loraString.length() + 1);
        xQueueSend(loraOut, (void *)&loraTx, 10); // send out trough Lora
        for (int i = 0; i < 3; i++)
        {
            if (buoyPara[i].mac != 0)
            {
                loraTx.macIDr = buoyPara[i].mac;
                loraTx.macIDs = buoyId;
                loraTx.msg = DOCKING;
                loraTx.ack = LORAGETACK;
                loraString = RoboCode(buoyPara[i], DOCKING);
                loraString.toCharArray(loraTx.data, loraString.length() + 1);
                xQueueSend(loraOut, (void *)&loraTx, 10); // send out trough Lora
            }
        }
        status = DOCKED;
        break;
    case IDELING:
        loraTx.macIDr = BUOYIDALL;
        loraTx.macIDs = buoyId;
        loraTx.msg = LORAIDELING;
        loraTx.ack = LORAGETACK;
        loraString = RoboCode(buoyPara[0], LORAIDELING);
        loraString.toCharArray(loraTx.data, loraString.length() + 1);
        xQueueSend(loraOut, (void *)&loraTx, 10); // send out trough Lora
        status = IDLE;
        break;
    case LOCKING:
        for (int i = 0; i < 3; i++)
        {
            if (buoyPara[i].mac != 0)
            {
                loraTx.macIDr = buoyPara[i].mac;
                loraTx.macIDs = buoyId;
                loraTx.msg = LOCKING;
                loraTx.ack = LORAGETACK;
                loraString = RoboCode(buoyPara[i], LOCKING);
                loraString.toCharArray(loraTx.data, loraString.length() + 1);
                xQueueSend(loraOut, (void *)&loraTx, 10); // send out trough Lora
            }
        }
        loraTx.macIDr = BUOYIDALL;
        loraTx.macIDs = buoyId;
        loraTx.msg = LOCKING;
        loraTx.ack = LORASET;
        loraString = RoboCode(buoyPara[0], LOCKING);
        loraString.toCharArray(loraTx.data, loraString.length() + 1);
        xQueueSend(loraOut, (void *)&loraTx, 10); // send out trough Lora
        status = IDLE;
        break;
    default:
        break;
    }
    return status;
}
void setup()
{
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT_PULLDOWN);
    pinMode(BUTTON_PIN_pwr, OUTPUT);
    digitalWrite(BUTTON_PIN_pwr, HIGH);
    initSSD1306();
    initloraqueue();
    initwifiqueue();
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8000, NULL, configMAX_PRIORITIES - 5, NULL, 0);
    Serial.println("Main task running!");
    // buoyPara[0].wDir = 275;
    // buoyPara[0].mac = 000010;
    // buoyPara[0].trackPos = -1;
    // buoyPara[0].tgLat = 52.289071585251;
    // buoyPara[0].tgLng = 4.926363927671;
    // buoyPara[1].mac = 000020;
    // buoyPara[1].trackPos = -1;
    // buoyPara[1].tgLat = 52.289749207653;
    // buoyPara[1].tgLng = 4.933875490798;
    // buoyPara[2].mac = 000030;
    // buoyPara[2].trackPos = -1;
    // buoyPara[2].tgLat = 52.291505969351;
    // buoyPara[2].tgLng = 4.932830118095;
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
            String loraStringIn = String(loraRx.data);
            loraData = RoboDecode(loraStringIn, loraData);
            loraData.mac = loraRx.macIDs; // store ID
            if (loraRx.macIDr == buoyId)  // yes i got a update
            {
            }
            if (loraRx.msg == LORALOCKPOS)
            {
                loraData.loralstmsg = millis();
                AddDataToBuoyBase(loraData, buoyPara);
            }
        }
        vTaskDelay(1);
    }
}
