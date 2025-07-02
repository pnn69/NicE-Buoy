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
    pinMode(PWRENABLE, OUTPUT);
    digitalWrite(PWRENABLE, 1); // enable batery sample signal
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
    initRudPid(&mainData);
    initSpeedPid(&mainData);
    speedMaxMin(&mainData, GET);
    initescqueue();
    if (digitalRead(BUTTON_PIN) == true)
    {
        delay(100);
        if (digitalRead(BUTTON_PIN) == true)
        {
            wifiConfig = 1;
        }
    }
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8000, &wifiConfig, configMAX_PRIORITIES - 5, NULL, 0);
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 1000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(EscTask, "EscTask", 2400, NULL, configMAX_PRIORITIES - 10, NULL, 1);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(CompassTask, "CompasaTask", 2000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    xTaskCreatePinnedToCore(SercomTask, "SerialTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
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
            delay(1000);
            calibrateMagneticNorth();
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
        else if (in->tgDist > 0.5)
        {
            in->tgSpeed = 0;
            rudderPid(in);
        }
        else
        {
            in->tgSpeed = 0;
            in->speedBb = 0;
            in->speedSb = 0;
            escOut.speedbb = 0;
            escOut.speedsb = 0;
            xQueueSend(escspeed, (void *)&escOut, 10);
        }
        // if (udptimer < millis())
        // {
        //     udptimer = millis() + 250;
        //     xQueueSend(udpOut, (void *)in, 10);
        // }
        // break;
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
        break;
    }

    if (nextSamp < millis())
    {
        nextSamp = 250 + millis();
        printf("TD:%05.2f TgSpeed: %05.2f C:%03.0f T:%03.0f A:%03.0f Rud:%02.2f  bb:%03d Sb:%03d S:%d", in->tgDist, in->tgSpeed, in->dirMag, in->tgDir, smallestAngle(in->tgDir, in->dirMag), rudderOutput, in->speedBb, in->speedSb, in->status);
        printf("     RudderITerm: %03.0f  SpeedITerm: %03.0f\r\n", in->ir, in->is);
        in->cmd = DIRSPEED;
        xQueueSend(serOut, (void *)in, 10);
    }

    if (accuSamp < millis())
    {
        accuSamp = 1010 + millis();
        battVoltage(in->subAccuV, in->subAccuP);
    }

    if (logtimer < millis())
    {
        logtimer = millis() + 5000;
        in->cmd = SUBACCU;
        xQueueSend(serOut, (void *)in, 10);
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
        stat->status = IDLE;
        escOut.speedbb = 0;
        escOut.speedsb = 0;
        xQueueSend(escspeed, (void *)&escOut, 10);
        xQueueSend(serOut, (void *)stat, 10);
        printf("Idle\r\n");
    }
}
//***************************************************************************************************
//  Handle incomming data
//***************************************************************************************************
/**
 * @brief Handles incoming serial and RF data for the RoboBuoy system.
 *
 * This function processes new data received from either the serial interface or RF (UDP) communication.
 * It updates the system state, executes commands, and manages the control logic for the RoboBuoy.
 * The function checks for new serial data, updates LED status, and processes commands such as IDLE, DIRDIST,
 * DIRSPEED, REMOTE, LOCKED, DOCKED, PID tuning, declination settings, and power settings.
 * It also handles sending acknowledgments and updating PID parameters for rudder and speed controllers.
 *
 * @param ser Pointer to a RoboStruct structure containing the current state and parameters of the RoboBuoy.
 */
void handelSerandRfdata(RoboStruct *ser)
{

    dataIn.IDr = -1;
    if (xQueueReceive(serIn, (void *)&dataIn, 0) == pdTRUE) // New serial data
    {
        mainData.lastSerIn = millis();
        PwrOff = millis();
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
            if (ser->status != IDELING)
                if (ser->status != IDELING)
                {
                    escOut.speedbb = 0;
                    escOut.speedsb = 0;
                    xQueueSend(escspeed, (void *)&escOut, 10);
                    ser->tgDist = 0;
                    ser->tgSpeed = 0;
                    ser->tgDir = 0;
                    ser->status = IDELING;
                    initRudPid(ser);
                    initSpeedPid(ser);
                    printf("IDLE command recieved!\r\n");
                }
            break;
        case DIRDIST:
            if (ser->status != LOCKED)
            {
                printf("DIRDIST command recieved!");
                initRudPid(ser);
                initSpeedPid(ser);
                ser->status = LOCKED;
            }
            ser->tgDir = dataIn.tgDir;
            ser->tgDist = dataIn.tgDist;
            break;
        case DIRSPEED:
            if (ser->status != DIRSPEED)
            {
                printf("DIRSPEED command recieved!");
                ser->tgDist = 0;
                initRudPid(ser);
                initSpeedPid(ser);
                ser->status = DIRSPEED;
            }
            ser->tgDir = dataIn.tgDir;
            ser->speedSet = dataIn.speedSet;
            break;
        case REMOTE:
            if (ser->status != REMOTE)
            {
                ser->status = REMOTE;
            }
            ser->tgDir = dataIn.tgDir;
            ser->tgSpeed = dataIn.tgSpeed;
            break;
        case LOCKED:
            if (ser->status != LOCKED)
            {
                printf("Locked command recieved!");
                ser->tgDist = 0;
                initRudPid(ser);
                initSpeedPid(ser);
                ser->tgLat = dataIn.tgLat;
                ser->tgLng = dataIn.tgLng;
                ser->status = LOCKED;
            }
            break;
        case DOCKED:
            if (ser->status != DOCKED)
            {
                printf("Docked command recieved!");
                ser->tgDist = 0;
                initRudPid(ser);
                initSpeedPid(ser);
                memDockPos(&dataIn, GET);
                ser->status = DOCKED;
            }
            break;
        case PIDRUDDER:
            if (dataIn.ack == LORAGET)
            {
                ser->ack = LORAINF;
                xQueueSend(serOut, (void *)ser, 10);
            }
            break;
        case PIDRUDDERSET:
            printf("Rudder PID pr:%0.2f ir:%0.2f dr:%0.2f\r\n", dataIn.pr, dataIn.ir, dataIn.dr);
            pidRudderParameters(&dataIn, SET);
            pidRudderParameters(ser, GET);
            rudderPID.SetTunings(dataIn.pr, dataIn.ir, dataIn.dr, DIRECT);
            break;
        case PIDSPEED:
            if (dataIn.ack == LORAGET)
            {
                ser->ack = LORAINF;
                xQueueSend(serOut, (void *)ser, 10);
            }
            break;
        case PIDSPEEDSET:
            printf("Rudder PID ps:%0.2f is:%0.2f ds:%0.2f\r\n", dataIn.ps, dataIn.is, dataIn.ds);
            pidSpeedParameters(&dataIn, SET);
            pidSpeedParameters(ser, GET);
            speedPID.SetTunings(dataIn.ps, dataIn.is, dataIn.ds, DIRECT);
            break;
        case STORE_DECLINATION:
            printf("Declinaton set to: %f\r\n", dataIn.declination);
            Declination(&dataIn.declination, SET);
            Declination(&ser->declination, GET);
            break;
        case SET_DECLINATION:
            calibrateMagneticNorth();
            ser->status = IDLE;
            xQueueSend(serOut, (void *)ser, 10);
            break;
        case CALIBRATE_MAGNETIC_COMPASS:
            calibrateParametersCompas();
            ser->status = IDLE;
            xQueueSend(serOut, (void *)ser, 10);
            break;
        case MAXMINPWR:
            if (dataIn.ack == LORAGET)
            {
                ser->ack = LORAINF;
                xQueueSend(serOut, (void *)ser, 10);
            }
            break;
        case MAXMINPWRSET:
            printf("Max speed %d Min speed %d\r\n", dataIn.maxSpeed, dataIn.minSpeed);
            speedMaxMin(&dataIn, SET);
            speedMaxMin(ser, GET);
            initRudPid(ser);
            initSpeedPid(ser);
            printf("Max speed %d Min speed %d\r\n", ser->maxSpeed, ser->minSpeed);
            break;
        case PING:
            ser->cmd = DIRSPEED;
            xQueueSend(serOut, (void *)ser, 10);
            break;
        }
        ser->lastSerIn = millis();
        PwrOff = millis();
    }
}

//***************************************************************************************************
//  Serial
//***************************************************************************************************
void handelSerialTimeOut(RoboStruct *ser)
{
    // if (mainData.lastSerIn + 1000 * 5 < millis())
    if (ser->lastSerIn + 1000 * 5 < millis())
    {
        if (mainLedStatus.color != CRGB::Red)
        {
            Serial.println("Set light to Red");
            ser->lastSerIn = millis();
            mainLedStatus.color = CRGB::Red;
            mainLedStatus.blink = BLINK_SLOW;
            xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
            digitalWrite(ESC_SB_PWR_PIN, LOW);
            digitalWrite(ESC_BB_PWR_PIN, LOW);
            ser->status = IDELING;
        }
    }
    //***************************************************************************************************
    //      Shutdown the system
    //***************************************************************************************************
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
        //      new serial message and udp
        //***************************************************************************************************
        handelSerandRfdata(&mainData);
        //***************************************************************************************************
        //      Serial watchdog
        //***************************************************************************************************
        handelSerialTimeOut(&mainData);
        vTaskDelay(1);
    }
}
