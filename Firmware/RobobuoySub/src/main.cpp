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
#include "sercom.h"
#include "pidrudspeed.h"

#define HOST_NAME "RoboBuoySub"

// pid subparameter;
static RoboStruct mainData;
static RoboStruct udpInMain;
static RoboStruct serDataIn;
static RoboStruct dataIn;
static RoboStruct compassInData;
static Message escOut;
static unsigned long buoyId;
static unsigned long PwrOff;
static int subStatus = IDLE;
static LedData mainLedStatus;
static PwrData mainPwrData;
static Buzz mainBuzzerData;
static int wifiConfig = 0;

unsigned long nextSamp = millis();
unsigned long accuSamp = millis();
unsigned long logtimer = millis();
unsigned long udptimer = millis();
unsigned long pidtimer = millis();
unsigned long esctimer = millis();
unsigned long serialtimer = millis();

int buttonState = 0;              // Current state of the button
int lastButtonState = 0;          // Previous state of the button
unsigned long lastPressTime = 0;  // Time of the last press
unsigned long debounceDelay = 50; // Debounce time in milliseconds
int pressCount = 0;               // Count the number of button presses
bool debounce = false;            // Debouncing flag
int presses = 0;

//***************************************************************************************************
// Setup
//***************************************************************************************************
void setup()
{
    pinMode(PWRENABLE, OUTPUT);
    digitalWrite(PWRENABLE, 1); // enable powersupply
    pinMode(ESC_SB_PWR_PIN, OUTPUT);
    pinMode(ESC_BB_PWR_PIN, OUTPUT);
    digitalWrite(ESC_SB_PWR_PIN, LOW);
    digitalWrite(ESC_BB_PWR_PIN, LOW);
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(BATTENABLE, OUTPUT);
    digitalWrite(BATTENABLE, 1); // enable batery sample signal
    digitalWrite(PWRENABLE, true);
    printf("\r\n\r\n\r\nSetup running!\r\n");
    printf("Robobuoy Sub Version: %0.1f Sub Build: %s %s\r\n", SUBVERSION, __DATE__, __TIME__);
    mainData.mac = espMac();
    printf("Robobuoy ID: %08x\r\n", mainData.mac);
    initwifi(); // buoyID is mac adress esp32
    initMemory();
    InitCompass();
    initledqueue();
    initbuzzerqueue();
    initcompassQueue();
    initserqueue();
    initRudPid();
    initSpeedPid();
    initescqueue();
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8000, &wifiConfig, configMAX_PRIORITIES - 5, NULL, 0);
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 1000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(EscTask, "EscTask", 2400, NULL, configMAX_PRIORITIES - 10, NULL, 1);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(CompassTask, "CompasaTask", 2000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    xTaskCreatePinnedToCore(SercomTask, "SerialTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    if (digitalRead(BUTTON_PIN) == true)
    {
        delay(100);
        if (digitalRead(BUTTON_PIN) == true)
        {
            wifiConfig = 1;
        }
    }
    mainData = computeParameters(mainData, GET);
    Serial.println("Setup done!");
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
    if (buttonState == LOW && lastButtonState == HIGH && !debounce)
    {
        pressCount++;                     // Increment the button press count
        debounce = true;                  // Set debounce flag
        beep(10, buzzer);                 // short high pitch beep
        debounceDelay = currentTime + 50; // Simple debouncing by adding a delay
        lastPressTime = currentTime;      // Record the time of the last press
    }
    // Reset debounce flag if the button is released
    if (buttonState == HIGH)
    {
        debounce = false;
    }
    // If more than 2 seconds have passed without a press, return the count
    if ((currentTime - lastPressTime) > 500 && pressCount > 0 && buttonState == HIGH)
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
    else if ((currentTime - lastPressTime) > 3000 && pressCount == 1 && buttonState == LOW)
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
            presses = CALIBRATE_MAGNETIC_COMPASS;
            xQueueSend(compassIn, (void *)&presses, 10); // Start compass calibration
            presses = -1;
        }
        if (presses != -1)
        {
            beep(-1, buzzer);
        }
    }
}
//***************************************************************************************************
// Timer routines
//***************************************************************************************************
void handleTimerRoutines(RoboStruct *in)
{
    switch (in->status)
    {
    case LOCKED:
    case DOCKED:
    case DIRDIST:
        if (in->tgDist > 1.5)
        {
            speedPid(in);
            if (in->tgSpeed < 0) // do not go backwards
            {
                in->tgSpeed = 0;
            }
            rudderPid(in);
        }
        else if (in->tgDist > .5)
        {
            RoboStruct *t = in;
            t->maxSpeed = 20;
            t->minSpeed = -20;
            t->tgSpeed = 0;
            rudderPid(t);
            in->tgSpeed = t->tgSpeed;
            in->speedSb = t->speedSb;
            in->speedBb = t->speedBb;
        }
        else
        {
            escOut.speedbb = 0;
            escOut.speedsb = 0;
            xQueueSend(escspeed, (void *)&escOut, 10);
        }
        break;
    case REMOTE:
        rudderPid(in);
        break;
    case SPBBSPSB: // SpeedBb,SpeedSb
        escOut.speedbb = in->speedBb;
        escOut.speedsb = in->speedSb;
        xQueueSend(escspeed, (void *)&escOut, 10);
        break;
    case IDLE:
    case IDELING:
        escOut.speedbb = 0;
        escOut.speedsb = 0;
        xQueueSend(escspeed, (void *)&escOut, 10);
        break;
    default:
        escOut.speedbb = 0;
        escOut.speedsb = 0;
        xQueueSend(escspeed, (void *)&escOut, 10);
        break;
    }

    if (nextSamp + 1000 < millis())
    {
        nextSamp = millis();
        xQueueSend(serOut, (void *)in, 10);
        printf("COG:%03.0f TG:%03.0f TD:%05.2f Error: %03.0f output: %07.2f tgSpeed: %05.2f bb: %3d Sb: %3d sTATUS:%d\r\n", in->dirMag, in->tgDir, in->tgDist, smallestAngle(in->tgDir, in->dirMag), rudderOutput, in->tgSpeed, in->speedBb, in->speedSb,in->status);
    }

    if (accuSamp < millis())
    {
        accuSamp = 60 * 1010 + millis();
        battVoltage(in->subAccuV, in->subAccuP);
    }

    if (logtimer < millis())
    {
        logtimer = millis() + 1000;
        in->cmd = SUBACCU;
        xQueueSend(serOut, (void *)in, 10); // Keep the watchdog in sub happy
    }
}
//***************************************************************************************************
//      status actions
//***************************************************************************************************
void handelStatus(RoboStruct *stat)
{
    if (stat->status == IDELING)
    {
        stat->speed = 0;
        stat->speedBb = 0;
        stat->speedSb = 0;
        escOut.speedbb = 0;
        escOut.speedsb = 0;
        xQueueSend(escspeed, (void *)&escOut, 10);
        printf("Idle\r\n");
        delay(100);
        stat->status = IDLE;
    }
}

//***************************************************************************************************
//  Main loop
//***************************************************************************************************
void loop(void)
{

    mainPwrData.sb = CRGB::Yellow;
    mainPwrData.bb = CRGB::Yellow;
    xQueueSend(ledStatus, (void *)&mainPwrData, 0); // update GPS led
    xQueueSend(ledPwr, (void *)&mainPwrData, 0);    // update util led
    PwrOff = millis();
    while (true)
    {
        //***************************************************************************************************
        //      Status change handeling
        //***************************************************************************************************
        handelStatus(&mainData);
        //***************************************************************************************************
        //      Timer routines
        //***************************************************************************************************
        handleTimerRoutines(&mainData);
        //***************************************************************************************************
        //      Check front key
        //***************************************************************************************************
        handelKeyPress();
        //***************************************************************************************************
        //      New compass data
        //***************************************************************************************************
        xQueueReceive(compass, (void *)&mainData.dirMag, 0);
        //***************************************************************************************************
        //      new serial message
        //***************************************************************************************************
        dataIn.IDr = -1;
        if (xQueueReceive(serIn, (void *)&dataIn, 0) == pdTRUE) // New serial data
        {
            if (mainLedStatus.color != CRGB::DarkBlue)
            {
                mainLedStatus.color = CRGB::DarkBlue;
                mainLedStatus.blink = BLINK_SLOW;
                xQueueSend(ledStatus, (void *)&mainLedStatus, 0); // update util led
            }
        }
        //***************************************************************************************************
        //      new udp message (handle only if no new serial data)
        //***************************************************************************************************
        // else if (xQueueReceive(udpIn, (void *)&dataIn, 0)) // New UDP data
        // {
        // }
        if (dataIn.IDr != -1)
        {
            switch (dataIn.cmd)
            {
            case IDLE:
            case IDELING:
                escOut.speedbb = 0;
                escOut.speedsb = 0;
                xQueueSend(escspeed, (void *)&escOut, 10);
                mainData.status = IDELING;
                printf("\"IDLE\" command recieved!\r\n");
                break;
            case DIRDIST:
                if (mainData.status != LOCKED)
                {
                    computeParameters(mainData, GET);
                    printf("DIRDIST\r\n");
                    startESC();
                    mainData.tgDist = 0;
                    initRudPid();
                    initSpeedPid();
                    mainData.status = LOCKED;
                }
                mainData.tgDir = dataIn.tgDir;
                mainData.tgDist = dataIn.tgDist;
                break;
            case DIRSPEED:
                if (mainData.status != DIRSPEED)
                {
                    computeParameters(mainData, GET);
                    printf("DIRSPEED\r\n");
                    mainData.tgDist = 0;
                    startESC();
                    initRudPid();
                    initSpeedPid();
                    mainData.status = DIRSPEED;
                }
                mainData.tgDir = dataIn.tgDir;
                mainData.speedSet = dataIn.speedSet;
                break;
            case REMOTE:
                if (mainData.status != REMOTE)
                {
                    computeParameters(mainData, GET);
                    mainData.status = REMOTE;
                }
                mainData.tgDir = dataIn.tgDir;
                mainData.tgSpeed = dataIn.tgSpeed;
                break;
            case LOCKED:
                if (mainData.status != LOCKED)
                {
                    printf("Locked\r\n");
                    mainData.tgDist = 0;
                    startESC();
                    initRudPid();
                    initSpeedPid();
                    mainData.tgLat = dataIn.tgLat;
                    mainData.tgLng = dataIn.tgLng;
                    mainData.status = LOCKED;
                }
                break;
            case DOCKED:
                if (mainData.status != DOCKED)
                {
                    printf("Docked\r\n");
                    startESC();
                    mainData.tgDist = 0;
                    initRudPid();
                    initSpeedPid();
                    memDockPos(&dataIn, GET);
                    mainData.status = DOCKED;
                }
                break;
            case PIDRUDDERSET:
                printf("Rudder PID pr:%0.2f ir:%0.2f dr:%0.2f\r\n", dataIn.pr, dataIn.ir, dataIn.dr);
                pidRudderParameters(dataIn, SET);
                rudderPID.SetTunings(dataIn.pr, dataIn.ir, dataIn.dr, DIRECT);
                break;
            case PIDSPEEDSET:
                printf("Rudder PID ps:%0.2f is:%0.2f ds:%0.2f\r\n", dataIn.ps, dataIn.is, dataIn.ds);
                pidSpeedParameters(dataIn, SET);
                speedPID.SetTunings(dataIn.ps, dataIn.is, dataIn.ds, DIRECT);
                break;
            case STORE_DECLINATION:
                Declination(&dataIn.declination, SET);
                break;
            case SET_DECLINATION:
                calibrateMagneticNorth();
                break;
            case CALIBRATE_MAGNETIC_COMPASS:
                xQueueSend(compassIn, (void *)&dataIn.cmd, 10); // Keep the watchdog in sub happy
                delay(1000);
                break;
                printf("Declination: %0.2f\r\n", dataIn.declination);
                break;
            case MAXMINPWR:
                speedMaxMin(&dataIn, SET);
                speedMaxMin(&mainData, GET);
                break;
            case PING:
                mainData.cmd = DIRSPEED;
                xQueueSend(serOut, (void *)&mainData, 10);
                break;
            }
            mainData.lastSerIn = millis();
            PwrOff = millis();
        }
        //***************************************************************************************************
        //      Serial watchdog
        //***************************************************************************************************
        // if (mainData.lastSerIn + 1000 * 5 < millis())
        if (mainData.lastSerIn + 1000 * 5 < millis())
        {
            if (mainLedStatus.color != CRGB::Red)
            {
                Serial.println("Set light to Red");
                mainData.lastSerIn = millis();
                mainLedStatus.color = CRGB::Red;
                mainLedStatus.blink = BLINK_SLOW;
                xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
                digitalWrite(ESC_SB_PWR_PIN, LOW);
                digitalWrite(ESC_BB_PWR_PIN, LOW);
                mainData.status = IDELING;
            }
        }
        //***************************************************************************************************
        //      Shutdown the system
        //***************************************************************************************************
        // if (PwrOff + 1000 * 60 * 3 < millis())
        if (PwrOff + 1000 * 60 * 5 < millis())
        {
            PwrOff = millis();
            triggerESC();
            beep(-1, buzzer);
            printf("Power off now!\r\n");
            digitalWrite(ESC_SB_PWR_PIN, LOW);
            digitalWrite(ESC_BB_PWR_PIN, LOW);
            delay(1000);
            digitalWrite(PWRENABLE, 0); // disable powersupply
            delay(1000);
            PwrOff = millis();
        }
        vTaskDelay(1);
    }
}
