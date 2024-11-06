#include <Arduino.h>
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
#define HOST_NAME "RoboBuoySub"

// pid subparameter;
static RoboStruct mainData;
static char udpInMain[MAXSTRINGLENG];
static Message esc;
static unsigned long buoyId;
static int subStatus = IDLE;
static LedData mainLedStatus;
static PwrData mainPwrData;
static Buzz mainBuzzerData;
static int wifiConfig = 0;

unsigned long nextSamp = millis();
unsigned long accuSamp = millis();
unsigned long logtimer = millis();
unsigned long udptimer = millis();

int buttonState = 0;              // Current state of the button
int lastButtonState = 0;          // Previous state of the button
unsigned long lastPressTime = 0;  // Time of the last press
unsigned long debounceDelay = 50; // Debounce time in milliseconds
int pressCount = 0;               // Count the number of button presses
bool debounce = false;            // Debouncing flag

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
// Setup
//***************************************************************************************************
void handleTimerRoutines(RoboStruct in)
{
    udptimer = millis();
    if (subStatus == UDPERROR)
    {
        mainLedStatus.color = CRGB::Blue;
        mainLedStatus.blink = BLINK_SLOW;
        xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
        subStatus = IDLE;
    }
    if (udptimer + 2000 < millis() + random(0, 100))
    {
        if (subStatus != UDPERROR)
        {
            subStatus = UDPERROR;
            mainLedStatus.color = CRGB::LightGoldenrodYellow;
            mainLedStatus.blink = BLINK_FAST;
            xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
        }
        udptimer = millis();
    }

    if (nextSamp < millis())
    {
        
        mainData.cmd = SUBDIRSPEED;
        xQueueSend(udpOut, (void *)&mainData, 10); // update WiFi
    }
    if (accuSamp < millis())
    {
        accuSamp = 5000 + millis() + random(0, 100);
        mainData.cmd = SUBACCU;
        battVoltage(mainData.subAccuV, mainData.subAccuP);
        xQueueSend(udpOut, (void *)&mainData, 10); // update WiFi
    }
    if (logtimer < millis())
    {
        logtimer = millis() + 1000;
        if (subStatus == TOPCALCRUDDER)
        {
            printf("hdg:%6.2f dir%6.2f dist%5.2f speed:%4d ", mainData.dirMag, mainData.tgDir, mainData.tgDist, mainData.speed);
            // printf(" buoy.iintergrater:%f pr%f ir%f err%f", mainData.iintergrater, mainData.pr, mainData.ir,mainData.lastErrr);
            printf("Dir:%6.1f Speed:%4d BB:%2d SB:%2d is:%5.1f ir:%5.1f\r\n", smallestAngle(mainData.dirMag, mainData.tgDir), mainData.speed, mainData.speedBb, mainData.speedSb, mainData.is, mainData.ir);
        }
        else
        {
            printf("Hdg:%5.1f Vbat:%3.1f %3d%%\r\n", mainData.dirMag, mainData.subAccuV, mainData.subAccuP);
            logtimer = millis() + 250;
        }
    }
}

//***************************************************************************************************
// Setup
//***************************************************************************************************
void setup()
{
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT);
    delay(10);
    printf("\r\nSetup running!\r\n");
    printf("Robobuoy Sub Version: %0.1f\r\n", SUBVERSION);
    buoyId = initwifiqueue();
    Serial.print("Buoy ID: ");
    Serial.println(buoyId);
    initbuzzerqueue();
    initledqueue();
    initescqueue();
    initMemory();
    initcompassQueue();
    InitCompass();
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 1000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(EscTask, "EscTask", 2400, NULL, configMAX_PRIORITIES - 10, NULL, 1);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(CompassTask, "CompasaTask", 2000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    if (digitalRead(BUTTON_PIN) == true)
    {
        delay(100);
        if (digitalRead(BUTTON_PIN) == true)
        {
            wifiConfig = 1;
        }
    }
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8000, &wifiConfig, configMAX_PRIORITIES - 5, NULL, 0);
    /*
    Get sored settings
    */

    // mainData.ps = 20;
    // mainData.is = 0.1;
    // mainData.ds = 0;
    // pidSpeedParameters(mainData, SET);
    // mainData.pr = 0.5;
    // mainData.ir = 0.02;
    // mainData.dr = 0;
    // pidRudderParameters(mainData, SET);
    // mainData.minOfsetDist = 2;
    // mainData.maxOfsetDist = 8;
    // mainData.minSpeed = 0;
    // mainData.maxSpeed = 80;
    // computeParameters(mainData, SET);
    // mainData.minOfsetDist = 2;
    // mainData.maxOfsetDist = 8;
    // mainData.minSpeed = 0;
    // mainData.maxSpeed = 80;
    // computeParameters(mainData, SET);
    mainData = pidSpeedParameters(mainData, GET);
    mainData = pidRudderParameters(mainData, GET);
    mainData = computeParameters(mainData, GET);
    Serial.println("Compute Parameters Speed P:" + String(mainData.kps) +
                   " I:" + String(mainData.kis) +
                   " Rudder p:" + String(mainData.kpr) +
                   " I:" + String(mainData.kir) +
                   " MAxspeed:" + String(mainData.maxSpeed) +
                   " MINspeed:" + String(mainData.minSpeed) +
                   " Maxoffsetdistance:" + String(mainData.maxOfsetDist) +
                   " MINoffsetdistance:" + String(mainData.minOfsetDist));
    Serial.println("Setup done!");
    delay(1000);
}

//***************************************************************************************************
//      key press stuff
//***************************************************************************************************
void handelKeyPress(void)
{
    int presses = countKeyPressesWithTimeoutAndLongPressDetecton();
    if (presses > 0)
    {
        Serial.print("Number of key presses: ");
        Serial.println(presses);
        if (presses == 5) // Calibrate compas north
        {
            beep(1, buzzer);
            calibrateNorthCompas();
            presses = -1;
        }
        if (presses == 10) // Calibrate compas
        {
            beep(1, buzzer);
            calibrateParametersCompas();
            presses = -1;
        }
        if (presses != -1)
        {
            beep(-1, buzzer);
        }
    }
}

//***************************************************************************************************
//      status actions
//***************************************************************************************************
int handelStatus(int stat)
{
    return stat;
}

//***************************************************************************************************
//  Main loop
//***************************************************************************************************
void loop(void)
{
    String dataIn = "";
    String dataOut = "";
    int msg = -1;
    float vbat = 0;
    int speedbb = 50, speedsb = 0;
    int presses = 0;
    beep(1000, buzzer);
    delay(500);
    mainLedStatus.color = CRGB::Blue;
    mainLedStatus.blink = BLINK_SLOW;
    xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
    mainPwrData.bb = CRGB::Black;
    mainPwrData.sb = CRGB::Black;
    mainPwrData.blinkBb = BLINK_OFF;
    mainPwrData.blinkSb = BLINK_OFF;
    xQueueSend(ledPwr, (void *)&mainPwrData, 10); // update util led

    /*
        Main loop
    */
    while (true)
    {
        //***************************************************************************************************
        //      Timer routines
        //***************************************************************************************************
        handleTimerRoutines(mainData);
        //***************************************************************************************************
        //      New compass data
        //***************************************************************************************************
        if (xQueueReceive(compass, (void *)&mainData.dirMag, 2) == pdTRUE)
        {
            if (subStatus == TOPCALCRUDDER)
            {
                mainData = CalcRudderBuoy(mainData); // calculate power to thrusters
                if (esc.speedbb != mainData.speedBb || esc.speedsb != mainData.speedSb)
                {
                    esc.speedbb = mainData.speedBb;
                    esc.speedsb = mainData.speedSb;
                    xQueueSend(escspeed, (void *)&esc, 10);
                }
            }
        }

        //***************************************************************************************************
        //      Check front key
        //***************************************************************************************************
        handelKeyPress();
        //***************************************************************************************************
        //      new udp message
        //***************************************************************************************************
        if (xQueueReceive(udpIn, (void *)&udpInMain, 0) == pdTRUE)
        {
            mainData = RoboDecode(udpInMain, mainData);
            switch (mainData.cmd)
            {
            case IDLE:
                subStatus = IDELING;
                break;
            case LOCKED:
                subStatus = LOCKED;
                break;
            case PING:
                mainData.cmd = PONG;
                break;
            }
        }
    }
    vTaskDelay(1);
}
