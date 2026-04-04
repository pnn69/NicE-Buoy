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
// #include "soc/soc.h"
#include "soc/rtc_cntl_reg.h" // needed to disable brownout detector

#define HOST_NAME "RoboBuoySub"
TaskHandle_t compassTaskHandle = NULL; // Task handle for compass task

// pid subparameter;
static RoboStruct mainData;
static RoboStruct udpInMain;
static RoboStruct serDataIn;
static RoboStruct dataIn;
static RoboStruct compassInData;
static Message escOut;
static unsigned long buoyId;
static unsigned long PwrOff;
static unsigned long pidTimer = 0;
static int subStatus = IDLE;
static LedData mainLedStatus;
static PwrData mainPwrData;
static Buzz mainBuzzerData;
static int wifiConfig = 0;
// timer variables
unsigned long nextSamp = millis();
unsigned long logtimer = millis();
// Button configuration
int buttonState = 0;              // Current state of the button
int lastButtonState = 0;          // Previous state of the button
unsigned long lastPressTime = 0;  // Time of the last press
unsigned long debounceDelay = 50; // Debounce time in milliseconds
int pressCount = 0;               // Count the number of button presses
bool debounce = false;            // Debouncing flag
int presses = 0;

/**
 * @brief Main setup function for the RoboBuoy Sub module.
 * 
 * This function initializes all the necessary hardware pins, enables the power supply,
 * and sets up serial communication. It also initializes memory, WiFi, the compass,
 * inter-task communication queues, and the PID controllers for rudder and speed.
 * Furthermore, it starts various FreeRTOS tasks assigned to specific CPU cores 
 * to handle background operations like WiFi, buzzer, ESCs, LEDs, compass, and serial communication.
 * Finally, it disables the brownout detector to prevent reset on voltage drops.
 * 
 * @param None
 * @return void
 */
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
    printf("Setup running!");
    printf("Robobuoy Sub Version: %0.1f Sub Build: %s %s", SUBVERSION, __DATE__, __TIME__);
    mainData.mac = espMac();
    printf("Robobuoy ID: %08x", mainData.mac);
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

    if (digitalRead(BUTTON_PIN) == LOW)
    {
        delay(100);
        if (digitalRead(BUTTON_PIN) == LOW)
        {
            wifiConfig = 1;
        }
    }
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8000, &wifiConfig, configMAX_PRIORITIES - 5, NULL, 0);
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(EscTask, "EscTask", 2400, NULL, configMAX_PRIORITIES - 5, NULL, 1);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(CompassTask, "CompassTask", 4000, NULL, configMAX_PRIORITIES - 1, &compassTaskHandle, 0); //&compassTaskHandle is used to suspend/resume the task
    xTaskCreatePinnedToCore(SercomTask, "SerialTask", 4000, NULL, configMAX_PRIORITIES - 3, NULL, 0);
    Serial.println("Setup done!");
    // Disable brownout detector
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
}

/**
 * @brief Detects button presses, implementing debounce and long-press detection.
 * 
 * This function reads the state of a physical button and applies a simple debounce mechanism.
 * It counts the number of consecutive presses within a specific time window.
 * If the button is held for an extended period, it registers it as a long press.
 * If a sequence of presses completes without further input, the total count is returned.
 * 
 * @param None
 * @return int The number of presses detected, 0x100 for a long press, or -1 if no complete sequence is detected yet.
 */
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

/**
 * @brief Processes the detected key presses to trigger specific actions.
 * Examples: 5 presses to calibrate magnetic north, 1 or 10 presses to start compass calibration.
 */
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
        if (presses == 1) // Calibrate compas
        {
            beep(1, buzzer);
            presses = CALIBRATE_MAGNETIC_COMPASS;
            xQueueSend(compassIn, (void *)&presses, 10); // Start compass calibration
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

/**
 * @brief Handles changes in the system's operational status.
 * 
 * This function monitors the main status of the RoboBuoy system. Specifically, 
 * it detects if the system transitions into the IDELING state. If so, it ensures 
 * that the ESC (Electronic Speed Controller) speeds are set to 0, stopping the motors, 
 * and then transitions the system to the IDLE state. It also queues the updated 
 * status for serial transmission.
 * 
 * @param stat Pointer to the `RoboStruct` containing the current system state, speed values, and operational status.
 * @return void This function does not return a value.
 */
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
        printf("Idle");
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
                printf("IDLE command recieved!");
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
        case TGDIRSPEED:
            if (ser->status != TGDIRSPEED)
            {
                printf("TGDIRSPEED command recieved!");
                ser->tgDist = 0;
                initRudPid(ser);
                initSpeedPid(ser);
                ser->status = TGDIRSPEED;
            }
            ser->tgDir = dataIn.tgDir;
            ser->speedSet = dataIn.speedSet;
            ser->tgSpeed = dataIn.speedSet; // Important: update tgSpeed so PID uses it
            break;
        case REMOTE:
            if (ser->status != REMOTE)
            {
                printf("REMOTE command recieved!");
                ser->status = REMOTE;
            }
            ser->tgDir = dataIn.tgDir;
            ser->tgSpeed = dataIn.tgSpeed;
            break;
        case LOCKED:
            if (ser->status != LOCKED)
            {
                printf("LOCKED command recieved!");
                ser->tgDist = 0;
                initRudPid(ser);
                initSpeedPid(ser);
                ser->status = LOCKED;
                ser->locked = false;
            }
            break;
        case DOCKED:
            if (ser->status != DOCKED)
            {
                printf("DOCKED command recieved!");
                ser->tgDist = 0;
                initRudPid(ser);
                initSpeedPid(ser);
                memDockPos(&dataIn, GET);
                ser->status = DOCKED;
                ser->locked = false;
            }
            break;
        case PIDRUDDER:
            if (dataIn.ack == LORAGET || dataIn.ack == LORAGETACK)
            {
                ser->IDr = dataIn.IDs;
                ser->cmd = PIDRUDDER;
                ser->ack = LORAINF;
                xQueueSend(serOut, (void *)ser, 10);
            }
            break;
        case PIDRUDDERSET:
            printf("New rudder PID settings pr:%0.2f ir:%0.2f dr:%0.2f\r\n", dataIn.Kpr, dataIn.Kir, dataIn.Kdr);
            pidRudderParameters(&dataIn, SET);
            pidRudderParameters(ser, GET);
            initRudPid(ser);
            printf("Rudder PID stored pr:%0.2f ir:%0.2f dr:%0.2f\r\n", ser->Kpr, ser->Kir, ser->Kdr);
            break;
        case PIDSPEED:
            if (dataIn.ack == LORAGET || dataIn.ack == LORAGETACK)
            {
                ser->IDr = dataIn.IDs;
                ser->cmd = PIDSPEED;
                ser->ack = LORAINF;
                xQueueSend(serOut, (void *)ser, 10);
            }
            break;
        case PIDSPEEDSET:
            printf("New speed PID settings ps:%0.2f is:%0.2f ds:%0.2f\r\n", dataIn.Kps, dataIn.Kis, dataIn.Kds);
            pidSpeedParameters(&dataIn, SET);
            pidSpeedParameters(ser, GET);
            initSpeedPid(ser);
            printf("Speed PID stored ps:%0.2f is:%0.2f ds:%0.2f\r\n", ser->Kps, ser->Kis, ser->Kds);
            break;
        case STORE_DECLINATION:
            printf("Declinaton set to: %f", dataIn.declination);
            Declination(&dataIn, SET);
            Declination(ser, GET);
            InitCompass();
            break;
        case STORE_COMPASS_OFFSET:
            printf("New compass offset: %f", dataIn.compassOffset);
            CompasOffset(&dataIn, SET);
            ser->compassOffset = dataIn.compassOffset; // Update running config
            InitCompass();
            printf(" (Stored)\r\n");
            break;
        case CALC_COMPASS_OFFSET:
            vTaskSuspend(compassTaskHandle);
            calibrateMagneticNorth();
            vTaskResume(compassTaskHandle);
            ser->status = IDELING;
            xQueueSend(serOut, (void *)ser, 10);
            ser->status = IDLE;
            break;
        case CALIBRATE_MAGNETIC_COMPASS:
            printf("Starting Desk Compass Calibration...");
            ser->status = CALIBRATE_MAGNETIC_COMPASS;
            {
                int cmd = CALIBRATE_MAGNETIC_COMPASS;
                xQueueSend(compassIn, (void *)&cmd, 10);
            }
            break;
        case INFIELD_CALIBRATE:
            printf("Starting In-Field Compass Calibration...");
            ser->status = INFIELD_CALIBRATE;
            {
                int cmd = INFIELD_CALIBRATE;
                xQueueSend(compassIn, (void *)&cmd, 10);
            }
            break;
        case MAXMINPWR:
                if (dataIn.ack == LORAGET || dataIn.ack == LORAGETACK)
                {
                    ser->IDr = dataIn.IDs;
                    ser->cmd = MAXMINPWR;
                    ser->ack = LORAINF;
                    speedMaxMin(ser, GET);
                    xQueueSend(serOut, (void *)ser, 10);
                }
                break;
            case MAXMINPWRSET:
            printf("New Speed settings Max:%d Min:%d Pivot:%0.2f\r\n", dataIn.maxSpeed, dataIn.minSpeed, dataIn.pivotSpeed);
            speedMaxMin(&dataIn, SET);
            speedMaxMin(ser, GET);
            initRudPid(ser);
            initSpeedPid(ser);
            printf("Max speed %d Min speed %d\r\n", ser->maxSpeed, ser->minSpeed);
            break;
        case SETUPDATA:
            if (dataIn.ack == LORAGET || dataIn.ack == LORAGETACK)
            {
                ser->IDr = dataIn.IDs;
                ser->cmd = SETUPDATA;
                ser->ack = LORAINF;
                // Fetch all data before sending
                pidRudderParameters(ser, GET);
                pidSpeedParameters(ser, GET);
                speedMaxMin(ser, GET);
                CompasOffset(ser, GET);
                xQueueSend(serOut, (void *)ser, 10);
                printf("Sent SETUPDATA back\r\n");
            }
            break;
        case HARDIRONFACTORS:
            dataIn.mac = espMac();
            hardIron(&dataIn, SET);
            InitCompass();
            break;
        case SOFTIRONFACTORS:
            dataIn.mac = espMac();
            softIron(&dataIn, SET);
            InitCompass();
            break;
        case RESET_RUDDER_PID:
            resetRudPid();
            printf("Resetting PID RUDDER!!\r\n");
            break;
        case RESET_SPEED_PID:
            resetSpeedPid();
            printf("Resetting PID SPEED!!\r\n");
            break;
        case RESET_SPEED_RUD_PID:
            resetSpeedPid();
            printf("Resetting PID SPEED!!\r\n");
            resetRudPid();
            printf("Resetting PID RUDDER!!\r\n");
            ser->locked = false; // Reset the locked flag so speed target initializes properly
            break;
        case PING:
            ser->cmd = DIRSPEED;
            xQueueSend(serOut, (void *)ser, 10);
            break;
        }
        dataIn.cmd = -1; // reset command
        ser->lastSerIn = millis();
        PwrOff = millis();
    }
}

/**
 * @brief Checks for serial communication timeouts and triggers safety shutdowns.
 * 
 * This function monitors the time elapsed since the last serial communication. 
 * If no data is received for 5 seconds, it transitions the system to a safe 
 * IDELING state, halts the motors, and turns the status LED red to indicate a 
 * timeout. If communication is lost for an extended period (5 minutes), it 
 * initiates a complete power-down sequence to preserve battery life, disabling 
 * the power supply and ESCs.
 * 
 * @param ser Pointer to the main `RoboStruct` containing system state and timestamps for last communication.
 * @return void This function does not return a value.
 */
void handelSerialTimeOut(RoboStruct *ser)
{
    if (ser->lastSerIn + 1000 * 5 < millis())
    {
        if (mainLedStatus.color != CRGB::Red)
        {
            Serial.println("Set light to Red");
            printf("Serial timeout!\r\n");
            ser->lastSerIn = millis();
            mainLedStatus.color = CRGB::Red;
            mainLedStatus.blink = BLINK_SLOW;
            xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
            delay(500);
            digitalWrite(ESC_SB_PWR_PIN, LOW);
            digitalWrite(ESC_BB_PWR_PIN, LOW);
            ser->status = IDELING;
        }
    }
    //***************************************************************************************************
    //      Shutdown the system after 5 minutes of no serial communication
    //***************************************************************************************************
    if (PwrOff + 1000 * 60 * 5 < millis())
    // if (PwrOff + 1000 * 10 < millis())
    {
        WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector
        PwrOff = millis();
        triggerESC();
        beep(-1, buzzer);
        printf("Power off now!\r\n");
        delay(5000);
        digitalWrite(ESC_SB_PWR_PIN, LOW);
        digitalWrite(ESC_BB_PWR_PIN, LOW);
        digitalWrite(PWRENABLE, 0); // disable powersupply
        delay(1000);
        PwrOff = millis();
    }
}

/**
 * @brief Handles periodic background tasks and control loop calculations.
 * Depending on the current status (LOCKED, DOCKED, DIRDIST, REMOTE, etc.), 
 * this calls the PID routines for speed and rudder, limits speeds, and 
 * queues ESC commands. Also handles telemetry reporting and logging.
 * @param in Pointer to the main RoboStruct containing system state.
 */
void handleTimerRoutines(RoboStruct *in)
{
    switch (in->status)
    {
    case LOCKED:
    case DOCKED:
    case DIRDIST:
        if (pidTimer < millis())
        {
            pidTimer = millis() + 50; // 50ms
            speedPid(in);
            if (in->tgDist > 1.0 && in->tgDist < 5000)
            {
                if (in->locked == false)
                {
                    in->locked = true;
                    resetSpeedPid();
                    in->tgSpeed = 0;
                }
                // Serial.printf("TD:%05.2f TgSpeed: %05.2f C:%03.0f T:%03.0f A:%03.0f", in->tgDist, in->tgSpeed, in->dirMag, in->tgDir, smallestAngle(in->tgDir, in->dirMag));
                if (in->tgSpeed < 0) // do not go backwards
                {
                    in->tgSpeed = 0;
                }
                if (in->ip > in->maxSpeed)
                {
                    in->ip = in->maxSpeed;
                }
                rudderPid(in);
            }
            else
            {
                in->locked = false;
                in->ir = 0;
                in->tgSpeed = 0;
                in->speedBb = 0;
                in->speedSb = 0;
            }
            escOut.speedbb = in->speedBb;
            escOut.speedsb = in->speedSb;
            xQueueSend(escspeed, (void *)&escOut, 10);
        }
        break;
    case REMOTE:
    case TGDIRSPEED:
        rudderPid(in);
        escOut.speedbb = in->speedBb;
        escOut.speedsb = in->speedSb;
        xQueueSend(escspeed, (void *)&escOut, 10);
        break;
    case SPBBSPSB: // SpeedBb,SpeedSb
        escOut.speedbb = in->speedBb;
        escOut.speedsb = in->speedSb;
        xQueueSend(escspeed, (void *)&escOut, 10);
        break;
    case IDLE:
    case IDELING:
        if (escOut.speedbb != 0 || escOut.speedsb != 0)
        {

            escOut.speedbb = 0;
            escOut.speedsb = 0;
            xQueueSend(escspeed, (void *)&escOut, 10);
        }
        in->speedBb = 0;
        in->speedSb = 0;
        in->ip = 0;
        in->ir = 0;
        break;
    default:
        break;
    }

    if (nextSamp < millis())
    {
        nextSamp = 250 + millis();
        in->cmd = SUBDATA;
        xQueueSend(serOut, (void *)in, 10);
    }

    if (logtimer < millis())
    {
        logtimer = millis() + 1000;
        battVoltage(in->subAccuV, in->subAccuP);
        battCurrent(in->subAccuI);
        printf("TD:%05.2f TgSpeed: %05.2f C:%03.0f T:%03.0f A:%03.0f Rud:%02.2f  bb:%03d Sb:%03d ", in->tgDist - 2, in->tgSpeed, in->dirMag, in->tgDir, smallestAngle(in->tgDir, in->dirMag), rudderOutput, in->speedBb, in->speedSb);
        printf("  ip: %05.3f ir: %05.3f\r\n", in->ip, in->ir);
    }
}

/**
 * @brief The main execution loop of the RoboBuoy Sub module.
 * 
 * This function serves as the infinite loop for the main thread. After initial 
 * setup, it continuously polls and handles various sub-systems: status changes, 
 * timer-based control routines, physical key presses, new data from the compass 
 * queue, incoming serial/RF messages, and communication watchdog timeouts. 
 * It manages the primary execution flow and ensures the system remains responsive.
 * 
 * @param void No parameters are taken by this function.
 * @return void This function does not return a value as it runs indefinitely.
 */
void loop(void)
{

    mainLedStatus.color = CRGB::Yellow;
    mainLedStatus.blink = BLINK_OFF;
    mainPwrData.sb = CRGB::Yellow;
    mainPwrData.bb = CRGB::Yellow;
    xQueueSend(ledStatus, (void *)&mainLedStatus, 0); // update status led
    xQueueSend(ledPwr, (void *)&mainPwrData, 0);    // update power led
    PwrOff = millis();
    mainData.status = IDLE;
    beep(1, buzzer);
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
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
