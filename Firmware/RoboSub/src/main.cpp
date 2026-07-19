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
// #include "soc/soc.h"
#include "soc/rtc_cntl_reg.h" // needed to disable brownout detector

#define HOST_NAME "RoboBuoySub"
TaskHandle_t compassTaskHandle = NULL; // Task handle for compass task

#ifndef SET_AS_NORTH
#define SET_AS_NORTH 125 // Custom command number for Set as North coming from Top
#endif

#define POWEROFFTIME 60000 * 60 // 60 minutes

// pid subparameter;
RoboStruct mainData;
SemaphoreHandle_t mainDataMutex = NULL;
static RoboStruct udpInMain;
static RoboStruct serDataIn;
static RoboStruct dataIn;
static RoboStruct compassInData;
Message escOut;
static unsigned long buoyId;
static unsigned long PwrOff;
static unsigned long pidTimer = 0;
int subStatus = IDLE;
static LedData mainLedStatus;
static PwrData mainPwrData;
static Buzz mainBuzzerData;
static int wifiConfig = 0;
extern uint32_t global_params_rev;

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
    Serial.begin(115200);
    pinMode(PWRENABLE, OUTPUT);
    digitalWrite(PWRENABLE, 1); // enable powersupply
    delay(1000); // Give ICM-20948 and other sensors time to power up completely!
    Wire.begin(21, 22);
    Wire.setClock(400000);
    
    // I2C Scanner
    Serial.println("\n--- I2C Scanner ---");
    // mainData.IDr = BUOYIDALL;
    byte error, address;
    int nDevices = 0;
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("I2C device found at address 0x%02X\n", address);
            nDevices++;
        }
        else if (error==4) {
            Serial.printf("Unknown error at address 0x%02X\n", address);
        }    
    }
    if (nDevices == 0) Serial.println("No I2C devices found\n");
    else Serial.println("done\n");
    Serial.println("-------------------");

    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(PWRENABLE, OUTPUT);
    digitalWrite(PWRENABLE, 1); // enable batery sample signal
    digitalWrite(PWRENABLE, true);
    mainDataMutex = xSemaphoreCreateMutex();
    printf("Setup running!");
    printf("Robobuoy Sub Version: %0.1f Sub Build: %s %s", SUBVERSION, __DATE__, __TIME__);
    mainData.mac = espMac();
    mainData.IDs = mainData.mac;
    mainData.IDr = BUOYIDALL;
    printf("Robobuoy ID: %08x", mainData.mac);
    initwifi(); // buoyID is mac adress esp32
    initMemory();
    pidRudderParameters(&mainData, GET);
    pidSpeedParameters(&mainData, GET);
    speedMaxMin(&mainData, GET);
    CompasOffset(&mainData, GET);
    computeParameters(&mainData, GET);
    thrusterInversion(&mainData, GET);
    thrusterSwap(&mainData, GET);
    
    InitCompass();
    initledqueue();
    initbuzzerqueue();
    initcompassQueue();
    initserqueue();
    initRudPid(&mainData);
    initSpeedPid(&mainData);
    initescqueue();

    if (digitalRead(BUTTON_PIN) == LOW)
    {
        delay(100);
        if (digitalRead(BUTTON_PIN) == LOW)
        {
            wifiConfig = 1;
        }
    }
    // CORE 0: Network and Telemetry
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 16384, &wifiConfig, configMAX_PRIORITIES - 10, NULL, 0);
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 0);
    
    // CORE 1: Real-time Control and Sensors
    xTaskCreatePinnedToCore(EscTask, "EscTask", 2400, NULL, configMAX_PRIORITIES - 5, NULL, 1);
    xTaskCreatePinnedToCore(CompassTask, "CompassTask", 8192, NULL, configMAX_PRIORITIES - 1, &compassTaskHandle, 1);
    xTaskCreatePinnedToCore(SercomTask, "SerialTask", 8192, NULL, configMAX_PRIORITIES - 3, NULL, 1);
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
            CalibrateCompass();
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

static int pendingStatus = -1;

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
        // Wait for motor ramps to reach 0 before fully transitioning
        if (stat->speedBb == 0 && stat->speedSb == 0)
        {
            int nextStatus = (pendingStatus != -1) ? pendingStatus : IDLE;
            stat->speed = 0;
            stat->status = nextStatus;
            initRudPid(stat);
            initSpeedPid(stat);
            xQueueSend(serOut, (void *)stat, 10);
            printf("Ramp down complete. Transitioning to status: %d\r\n", nextStatus);
            pendingStatus = -1;
        }
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
        printf("Serial command recieved %d\r\n", dataIn.cmd);
        mainData.lastSerIn = millis();
        PwrOff = millis();
        if (mainLedStatus.color != CRGB::DarkBlue)
        {
            mainLedStatus.color = CRGB::DarkBlue;
            mainLedStatus.blink = BLINK_SLOW;
            xQueueSend(ledStatus, (void *)&mainLedStatus, 0); // update util led
        }
    }
    
    if (dataIn.IDr != -1)
    {
        // Only process packets addressed to us, or broadcast, or special SETUPDATA cases
        if (dataIn.IDr == mainData.mac || dataIn.IDr == BUOYIDALL || dataIn.cmd == SETUPDATA)
        {
            switch (dataIn.cmd)
            {
            case IDLE:
            case IDELING:
                if (ser->status != IDELING && ser->status != IDLE)
                {
                    ser->tgDist = 0;
                    ser->tgSpeed = 0;
                    ser->tgDir = 0;
                    ser->status = IDELING;
                    printf("IDLE command received. Initiating smooth motor ramp down.\r\n");
                }
                break;
            case DIRDIST:
                {
                    int targetStatus = (dataIn.status == DOCKED) ? DOCKED : LOCKED;
                    if (ser->status != targetStatus)
                    {
                        if (ser->status == LOCKED || ser->status == DOCKED)
                        {
                            printf("DIRDIST command received! Transitioning from status %d to %d via smooth ramp down.\r\n", ser->status, targetStatus);
                            pendingStatus = targetStatus;
                            ser->status = IDELING;
                        }
                        else
                        {
                            printf("DIRDIST command received! Instantly transitioning to status: %d\r\n", targetStatus);
                            initRudPid(ser);
                            initSpeedPid(ser);
                            ser->status = targetStatus;
                        }
                    }
                    ser->tgDir = dataIn.tgDir;
                    ser->tgDist = dataIn.tgDist;
                }
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
                    if (ser->status == DOCKED)
                    {
                        printf("LOCKED command received! Transitioning from DOCKED via smooth ramp down.\r\n");
                        ser->tgDist = 0;
                        pendingStatus = LOCKED;
                        ser->status = IDELING;
                        ser->locked = false;
                    }
                    else
                    {
                        printf("LOCKED command received! Instantly transitioning to LOCKED.\r\n");
                        ser->tgDist = 0;
                        initRudPid(ser);
                        initSpeedPid(ser);
                        ser->status = LOCKED;
                        ser->locked = false;
                    }
                }
                break;
            case DOCKED:
                if (ser->status != DOCKED)
                {
                    if (ser->status == LOCKED)
                    {
                        printf("DOCKED command received! Transitioning from LOCKED via smooth ramp down.\r\n");
                        ser->tgDist = 0;
                        pendingStatus = DOCKED;
                        ser->status = IDELING;
                        memDockPos(&dataIn, GET);
                        ser->locked = false;
                    }
                    else
                    {
                        printf("DOCKED command received! Instantly transitioning to DOCKED.\r\n");
                        ser->tgDist = 0;
                        initRudPid(ser);
                        initSpeedPid(ser);
                        memDockPos(&dataIn, GET);
                        ser->status = DOCKED;
                        ser->locked = false;
                    }
                }
                break;
            case PIDRUDDER:
                if (dataIn.ack == GET || dataIn.ack == GETACK)
                {
                    RoboStruct response = mainData;
                    response.IDr = dataIn.IDs;
                    response.cmd = PIDRUDDER;
                    response.ack = INF;
                    pidRudderParameters(&response, GET);
                    xQueueSend(serOut, (void *)&response, 10);
                }
                break;
            case PIDRUDDERSET:
                global_params_rev++;
// printf("New rudder PID settings pr:%0.2f ir:%0.2f dr:%0.2f\r\n", dataIn.Kpr, dataIn.Kir, dataIn.Kdr);
                pidRudderParameters(&dataIn, SET);
                pidRudderParameters(ser, GET);
                initRudPid(ser);
// printf("Rudder PID stored pr:%0.2f ir:%0.2f dr:%0.2f\r\n", ser->Kpr, ser->Kir, ser->Kdr);
                
                {
                    RoboStruct response = *ser;
                    response.IDr = dataIn.IDs;
                    response.cmd = PIDRUDDERSET;
                    response.ack = INF;
                    xQueueSend(serOut, (void *)&response, 10);
                }
                break;
            case PIDSPEED:
                if (dataIn.ack == GET || dataIn.ack == GETACK)
                {
                    RoboStruct response = mainData;
                    response.IDr = dataIn.IDs;
                    response.cmd = PIDSPEED;
                    response.ack = INF;
                    pidSpeedParameters(&response, GET);
                    xQueueSend(serOut, (void *)&response, 10);
                }
                break;
            case PIDSPEEDSET:
                global_params_rev++;
// printf("New speed PID settings ps:%0.2f is:%0.2f ds:%0.2f\r\n", dataIn.Kps, dataIn.Kis, dataIn.Kds);
                pidSpeedParameters(&dataIn, SET);
                pidSpeedParameters(ser, GET);
                initSpeedPid(ser);
// printf("Speed PID stored ps:%0.2f is:%0.2f ds:%0.2f\r\n", ser->Kps, ser->Kis, ser->Kds);
                
                {
                    RoboStruct response = *ser;
                    response.IDr = dataIn.IDs;
                    response.cmd = PIDSPEEDSET;
                    response.ack = INF;
                    xQueueSend(serOut, (void *)&response, 10);
                }
                break;
            case ADAPTIVE_TRIM:
                if (dataIn.ack == GET || dataIn.ack == GETACK)
                {
                    RoboStruct response = mainData;
                    response.IDr = dataIn.IDs;
                    response.cmd = ADAPTIVE_TRIM;
                    response.ack = INF;
                    response.compass_trim = mainData.compass_trim;
                    response.compass_trim_enabled = mainData.compass_trim_enabled;
                    xQueueSend(serOut, (void *)&response, 10);
                }
                else if (dataIn.ack == SET)
                {
                    mainData.compass_trim = dataIn.compass_trim;
                    mainData.compass_trim_enabled = dataIn.compass_trim_enabled;
                    
                    if (mainData.compass_trim < -15.0f) mainData.compass_trim = -15.0f;
                    if (mainData.compass_trim > 15.0f) mainData.compass_trim = 15.0f;
                    
                    float trim_val = (float)mainData.compass_trim;
                    bool trim_en = mainData.compass_trim_enabled;
                    memCompassTrim(&trim_val, &trim_en, SET);
                    global_params_rev++;

                    RoboStruct response = mainData;
                    response.IDr = dataIn.IDs;
                    response.cmd = ADAPTIVE_TRIM;
                    response.ack = INF;
                    response.compass_trim = mainData.compass_trim;
                    response.compass_trim_enabled = mainData.compass_trim_enabled;
                    xQueueSend(serOut, (void *)&response, 10);
                }
                break;
            case STORE_DECLINATION:
                printf("Declinaton set to: %f", dataIn.declination);
                Declination(&dataIn, SET);
                Declination(ser, GET);
                InitCompass();
                break;
            case STORE_COMPASS_OFFSET:
                CompassOffsetCorrection(&dataIn.compassOffset, false);
                ser->compassOffset = dataIn.compassOffset; // Update running config
                mainData.compassOffset = dataIn.compassOffset; // Ensure compassTask uses the new offset immediately!
                printf(" (Stored)\r\n");
                // Send an ACK back to the Top buoy so it stops retransmitting (if GETACK is used)
                if (dataIn.ack == GETACK || dataIn.ack == SET) {
                    RoboStruct response = mainData;
                    response.IDr = dataIn.IDs;
                    response.cmd = STORE_COMPASS_OFFSET;
                    response.ack = INF;
                    response.status = IDELING; // Force a status sync back to Top
                    xQueueSend(serOut, (void *)&response, 10);
                }
                break;
            case SET_AS_NORTH:
                {
                    double newOffset = 0;
                    bool success = false;
                    if (mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(500))) {
                        newOffset = mainData.compassOffset - mainData.dirMag;
                        while (newOffset < -180.0) newOffset += 360.0;
                        while (newOffset > 180.0) newOffset -= 360.0;
                        mainData.compassOffset = newOffset;
                        ser->compassOffset = newOffset;
                        success = true;
                        xSemaphoreGive(mainDataMutex);
                    }
                    if (success) {
                        CompasOffset(&mainData, SET);
                        global_params_rev++;
                        printf("SET_AS_NORTH command received via serial! Offset adjusted to %0.2f and saved to NVS.\r\n", newOffset);
                        extern QueueHandle_t buzzer;
                        if (buzzer != NULL) {
                            beep(-1, buzzer);
                        }
                        RoboStruct response = mainData;
                        response.IDr = dataIn.IDs;
                        response.cmd = SET_AS_NORTH;
                        response.ack = INF;
                        response.status = IDELING;
                        xQueueSend(serOut, (void *)&response, 10);
                    }
                }
                break;
            case CALC_COMPASS_OFFSET:
                vTaskSuspend(compassTaskHandle);
                CalibrateCompass();
                vTaskResume(compassTaskHandle);
                {
                    RoboStruct response = mainData;
                    response.status = IDELING;
                    xQueueSend(serOut, (void *)&response, 10);
                }
                ser->status = IDLE;
                break;
            case CALIBRATE_MAGNETIC_COMPASS:
                printf("Starting Desk Compass Calibration...");
                ser->status = CALIBRATE_MAGNETIC_COMPASS;
                {
                    // Immediate feedback
                    mainLedStatus.color = CRGB::Purple;
                    mainLedStatus.blink = BLINK_FAST;
                    xQueueSend(ledStatus, (void *)&mainLedStatus, 0);
                    beep(1, buzzer);

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
                    if (dataIn.ack == GET || dataIn.ack == GETACK)
                    {
                        RoboStruct response = mainData;
                        response.IDr = dataIn.IDs;
                        response.cmd = MAXMINPWR;
                        response.ack = INF;
                        speedMaxMin(&response, GET);
                        xQueueSend(serOut, (void *)&response, 10);
                    }
                    break;
                case MAXMINPWRSET:
                global_params_rev++;
                printf("New Speed settings Max:%d Min:%d Pivot:%0.2f\r\n", dataIn.maxSpeed, dataIn.minSpeed, dataIn.pivotSpeed);
                speedMaxMin(&dataIn, SET);
                speedMaxMin(ser, GET);
                initRudPid(ser);
                initSpeedPid(ser);
                printf("Max speed %d Min speed %d\r\n", ser->maxSpeed, ser->minSpeed);
                
                // Send confirmation back so Top and UI get the updated values
                {
                    RoboStruct response = *ser;
                    response.IDr = dataIn.IDs;
                    response.cmd = MAXMINPWRSET;
                    response.ack = INF;
                    xQueueSend(serOut, (void *)&response, 10);
                }
                break;
            case SETUPDATA:
                global_params_rev++;
                if (dataIn.ack == GET || dataIn.ack == GETACK)
                {
                    RoboStruct response = mainData;
                    response.IDs = mainData.mac;
                    response.IDr = dataIn.IDs;
                    response.cmd = SETUPDATA;
                    response.ack = INF;
                    // Fetch all data before sending
                    pidRudderParameters(&response, GET);
                    pidSpeedParameters(&response, GET);
                    speedMaxMin(&response, GET);
                    CompasOffset(&response, GET);
                    thrusterSwap(&response, GET);
                    thrusterInversion(&response, GET);
                    computeParameters(&response, GET); 
                    xQueueSend(serOut, (void *)&response, 10);
// printf("Sent SETUPDATA back to %X\r\n", response.IDr);
                }
                else if (dataIn.ack == SET)
                {
// printf("New setup received. Updating PID and Inversion flags.\r\n");
                    pidRudderParameters(&dataIn, SET);
                    pidSpeedParameters(&dataIn, SET);
                    speedMaxMin(&dataIn, SET);
                    CompasOffset(&dataIn, SET);
                    thrusterSwap(&dataIn, SET);
                    thrusterInversion(&dataIn, SET);
                    computeParameters(&dataIn, SET);
                    
                    // Reload into running config
                    pidRudderParameters(ser, GET);
                    pidSpeedParameters(ser, GET);
                    speedMaxMin(ser, GET);
                    CompasOffset(ser, GET);
                    thrusterSwap(ser, GET);
                    thrusterInversion(ser, GET);
                    computeParameters(ser, GET);
                    mainData.compassOffset = ser->compassOffset; // Ensure compassTask uses the new offset immediately!
                    
                    initRudPid(ser);
                    initSpeedPid(ser);

                    // Send confirmation back so Top and UI get the updated values
                    RoboStruct response = *ser;
                    response.IDr = dataIn.IDs;
                    response.cmd = SETUPDATA;
                    response.ack = INF;
                    xQueueSend(serOut, (void *)&response, 10);
                    printf("Sent updated SETUPDATA back\r\n");
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
// printf("Resetting PID RUDDER!!\r\n");
                break;
            case RESET_SPEED_PID:
                resetSpeedPid();
// printf("Resetting PID SPEED!!\r\n");
                break;
            case RESET_SPEED_RUD_PID:
                resetSpeedPid();
// printf("Resetting PID SPEED!!\r\n");
                resetRudPid();
// printf("Resetting PID RUDDER!!\r\n");
                ser->locked = false; // Reset the locked flag so speed target initializes properly
                break;
            case PING:
                ser->cmd = DIRSPEED;
                xQueueSend(serOut, (void *)ser, 10);
                break;
            }
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
            ser->status = IDELING;
        }
    }
    //***************************************************************************************************
    //      Shutdown the system after 5 minutes of no serial communication
    //***************************************************************************************************
    if (PwrOff + POWEROFFTIME < millis())
    // if (PwrOff + 1000 * 10 < millis())
    {
        WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector
        PwrOff = millis();
        // beep(-1, buzzer);
        printf("Power off now!\r\n");
        delay(1000);
        digitalWrite(PWRENABLE, 0); // disable powersupply
        delay(5000);
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
            pidTimer = millis() + 20; // 20ms
            speedPid(in);
            
            // Use filtered distance for the safety check to prevent jitter-induced drops
            double filtered_dist = GetFilteredDist(); 
            double safety_threshold = (in->status == DOCKED) ? 0.2 : 1.0;

            if (filtered_dist > safety_threshold && filtered_dist < 5000)
            {
                if (in->locked == false)
                {
                    in->locked = true;
                    resetSpeedPid();
                    in->tgSpeed = 0;
                }
                if (in->tgSpeed < 0) in->tgSpeed = 0;
                if (in->ip > in->maxSpeed) in->ip = in->maxSpeed;
                
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
    case IDELING:
        if (pidTimer < millis())
        {
            pidTimer = millis() + 20; // 20ms
            rudderPid(in);
            
            escOut.speedbb = in->speedBb;
            escOut.speedsb = in->speedSb;
            xQueueSend(escspeed, (void *)&escOut, 10);
        }
        break;
    case IDLE:
        escOut.speedbb = 0;
        escOut.speedsb = 0;
        in->speedBb = 0;
        in->speedSb = 0;
        /*
         * CRITICAL FIX: We must NOT clear or overwrite in->ip (pitch) or in->ir (roll) to 0 here.
         * The pitch and roll represent real physical state values of the buoy that are calculated 
         * asynchronously by CompassTask() running on Core 1. If we overwrite them to 0 on Core 0 
         * during IDLE status, it creates a state-overwriting race condition, causing the telemetry 
         * and web dashboards to intermittently drop to 0.0 depending on scheduling order.
         */
        xQueueSend(escspeed, (void *)&escOut, 10);
        break;
    default:
        break;
    }

    if (nextSamp < millis())
    {
        nextSamp = 100 + millis();
        RoboStruct telemetry = mainData;
        telemetry.cmd = SUBDATA;
        telemetry.ack = 6; // Full Status packet
        xQueueSend(serOut, (void *)&telemetry, 0);
        
        static unsigned long nextUdpSamp = 0;
        if (udpOut && nextUdpSamp < millis()) {
            nextUdpSamp = 1000 + millis();
            xQueueSend(udpOut, (void *)&telemetry, 0);
        }
    }

    static unsigned long nextTrimSendTime = 0;
    if (nextTrimSendTime < millis()) {
        nextTrimSendTime = 10000 + millis(); // Every 10 seconds
        
        RoboStruct trimMsg = {};
        trimMsg.IDs = mainData.mac;
        trimMsg.IDr = BUOYIDALL;
        trimMsg.cmd = ADAPTIVE_TRIM;
        trimMsg.ack = INF;
        trimMsg.compass_trim = mainData.compass_trim;
        trimMsg.compass_trim_enabled = mainData.compass_trim_enabled;
        
        xQueueSend(serOut, (void *)&trimMsg, 0);
    }

    static unsigned long nextTrimSaveTime = 0;
    if (nextTrimSaveTime < millis()) {
        nextTrimSaveTime = 15000 + millis(); // Check every 15 seconds
        
        static double last_saved_trim = -999.0;
        static int last_saved_en = -1;
        
        int current_en = mainData.compass_trim_enabled ? 1 : 0;
        if (abs(mainData.compass_trim - last_saved_trim) > 0.02 || current_en != last_saved_en) {
            last_saved_trim = mainData.compass_trim;
            last_saved_en = current_en;
            
            float trim_val = (float)mainData.compass_trim;
            bool trim_en = mainData.compass_trim_enabled;
            memCompassTrim(&trim_val, &trim_en, SET);
            printf("Background Saved Compass Trim -> Val: %.3f, En: %d\r\n", trim_val, trim_en);
        }
    }

    /* 
     * Telemetry and Logging Interval Update:
     * Reduced logtimer interval from 1000ms to 500ms to increase logging resolution.
     * This provides more immediate physical sensor output, battery metrics, 
     * and ESC/thruster speeds for diagnostic interfaces, ensuring that rapid 
     * status transitions are captured clearly on the monitor without causing
     * significant serial bus contention.
     */
    if (logtimer < millis())
    {
        logtimer = millis() + 500;
        battVoltage(mainData.subAccuV, mainData.subAccuP);
        battCurrent(mainData.subAccuI);
        printf("C:%03.0f Rud:%02.2f  bb:%03d Sb:%03d ", mainData.dirMag, rudderOutput, mainData.speedBb, mainData.speedSb);
        printf("  Is: %05.3f Ir: %05.3f P: %05.1f R: %05.1f %0.2fV %0.2fA\r\n", mainData.ip, mainData.ir, mainData.pitch, mainData.roll, mainData.subAccuV, mainData.subAccuI);
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

    mainLedStatus.color = CRGB::Black;
    mainLedStatus.blink = BLINK_OFF;
    mainPwrData.sb = CRGB::Black;
    mainPwrData.bb = CRGB::Black;
    xQueueSend(ledStatus, (void *)&mainLedStatus, 0); // update status led
    xQueueSend(ledPwr, (void *)&mainPwrData, 0);    // update power led
    PwrOff = millis();
    mainData.status = IDLE;
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
