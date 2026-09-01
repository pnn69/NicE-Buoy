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
#include "udplog.h"
// #include "soc/soc.h"
#include "soc/rtc_cntl_reg.h" // needed to disable brownout detector

#define HOST_NAME "RoboBuoySub"
TaskHandle_t compassTaskHandle = NULL; // Task handle for compass task

// SET_AS_NORTH is an ENUMERATOR in RoboCompute.h (msg_t, value 87), not a macro. #ifndef only
// sees macros, so the guard that used to stand here was always true and its #define fired every
// time - rewriting every SET_AS_NORTH below it, including "case SET_AS_NORTH:", to the literal
// 125. The Top and the CYD send 87, so the case could never match and Set as North was dead on
// every path except the Sub's own /set_north web endpoint, which bypasses this dispatcher.
// Do not reintroduce a fallback #define for a name that RoboCompute.h already defines.

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
    
    // Drive ESC power pins LOW immediately to keep them completely powered off and silent
    // during the zero-rate gyroscope calibration phase!
    digitalWrite(ESC_SB_PWR_PIN, LOW);
    digitalWrite(ESC_BB_PWR_PIN, LOW);
    pinMode(ESC_SB_PWR_PIN, OUTPUT);
    pinMode(ESC_BB_PWR_PIN, OUTPUT);

    pinMode(PWRENABLE, OUTPUT);
    digitalWrite(PWRENABLE, 1); // enable powersupply
    delay(1000); // Give ICM-20948 and other sensors time to power up completely!
    Wire.begin(21, 22);
    Wire.setClock(400000);
    
    // I2C Scanner
    Serial.println("\n\r--- I2C Scanner ---");
    // mainData.IDr = BUOYIDALL;
    byte error, address;
    int nDevices = 0;
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("I2C device found at address 0x%02X\n\r", address);
            nDevices++;
        }
        else if (error==4) {
            Serial.printf("Unknown error at address 0x%02X\n\r", address);
        }    
    }
    if (nDevices == 0) Serial.println("No I2C devices found\n\r");
    else Serial.println("done\n\r");
    Serial.println("-------------------");

    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(PWRENABLE, OUTPUT);
    digitalWrite(PWRENABLE, 1); // enable batery sample signal
    digitalWrite(PWRENABLE, true);
    mainDataMutex = xSemaphoreCreateMutex();
    printf("Setup running!\n\r");
    printf("Robobuoy Sub Version: %0.1f Sub Build: %s %s\n\r", SUBVERSION, __DATE__, __TIME__);
    mainData.mac = espMac();
    mainData.IDs = mainData.mac;
    mainData.IDr = BUOYIDALL;
    printf("Robobuoy ID: %08x\n\r", mainData.mac);
    initwifi(); // buoyID is mac address esp32
    initMemory();
    pidRudderParameters(&mainData, MEM_GET);
    pidSpeedParameters(&mainData, MEM_GET);
    speedMaxMin(&mainData, MEM_GET);
    CompasOffset(&mainData, MEM_GET);
    computeParameters(&mainData, MEM_GET);
    thrusterInversion(&mainData, MEM_GET);
    thrusterSwap(&mainData, MEM_GET);
    
    initledqueue();
    // Core 0: Start LED Task immediately on boot so we can show dynamic status (e.g. fast yellow blinking) during calibration
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 0);

    InitCompass();
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
void handleKeyPress(void)
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
 * it detects if the system transitions into the IDLING state. If so, it ensures 
 * that the ESC (Electronic Speed Controller) speeds are set to 0, stopping the motors, 
 * and then transitions the system to the IDLE state. It also queues the updated 
 * status for serial transmission.
 * 
 * @param stat Pointer to the `RoboStruct` containing the current system state, speed values, and operational status.
 * @return void This function does not return a value.
 */
void handleStatus(RoboStruct *stat)
{
    if (stat->status == IDLING)
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
//  Handle incoming data
//***************************************************************************************************
/**
 * @brief Handles incoming serial and RF data for the RoboBuoy system.
 *
 * This function processes new data received from either the serial interface or RF (UDP) communication.
 * It updates the system state, executes commands, and manages the control logic for the RoboBuoy.
 * The function checks for new serial data, updates LED status, and processes commands such as IDLE, DIRDIST,
 * DIRSPEED, REMOTE, LOCKED, DOCKED, PID tuning, compass offset, and power settings.
 * It also handles sending acknowledgments and updating PID parameters for rudder and speed controllers.
 *
 * @param ser Pointer to a RoboStruct structure containing the current state and parameters of the RoboBuoy.
 */
void handleSerandRfdata(RoboStruct *ser)
{

    dataIn.IDr = -1;
    if (xQueueReceive(serIn, (void *)&dataIn, 0) == pdTRUE) // New serial data
    {
        printf("Serial command received %d\r\n", dataIn.cmd);
        mainData.lastSerIn = millis();
        udpLog("SER in cmd=%d ack=%d IDr=%08lX IDs=%08lX",
               dataIn.cmd, dataIn.ack, (unsigned long)dataIn.IDr, (unsigned long)dataIn.IDs);
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
            case IDLING:
                if (ser->status != IDLING && ser->status != IDLE)
                {
                    ser->tgDist = 0;
                    ser->tgSpeed = 0;
                    ser->tgDir = 0;
                    ser->status = IDLING;
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
                            ser->status = IDLING;
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
                    printf("TGDIRSPEED command received!");
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
                    printf("REMOTE command received!");
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
                        ser->status = IDLING;
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
                        ser->status = IDLING;
                        memDockPos(&dataIn, MEM_GET);
                        ser->locked = false;
                    }
                    else
                    {
                        printf("DOCKED command received! Instantly transitioning to DOCKED.\r\n");
                        ser->tgDist = 0;
                        initRudPid(ser);
                        initSpeedPid(ser);
                        memDockPos(&dataIn, MEM_GET);
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
                    pidRudderParameters(&response, MEM_GET);
                    xQueueSend(serOut, (void *)&response, 10);
                }
                break;
            case PIDRUDDERSET:
                global_params_rev++;
// printf("New rudder PID settings pr:%0.2f ir:%0.2f dr:%0.2f\r\n", dataIn.Kpr, dataIn.Kir, dataIn.Kdr);
                pidRudderParameters(&dataIn, MEM_PUT);
                pidRudderParameters(ser, MEM_GET);
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
                    pidSpeedParameters(&response, MEM_GET);
                    xQueueSend(serOut, (void *)&response, 10);
                }
                break;
            case PIDSPEEDSET:
                global_params_rev++;
// printf("New speed PID settings ps:%0.2f is:%0.2f ds:%0.2f\r\n", dataIn.Kps, dataIn.Kis, dataIn.Kds);
                pidSpeedParameters(&dataIn, MEM_PUT);
                pidSpeedParameters(ser, MEM_GET);
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
                    memCompassTrim(&trim_val, &trim_en, MEM_PUT);
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
                // Retired, see RoboCompute.h. Accepted and ignored: the heading correction the
                // operator actually wants is compassOffset (STORE_COMPASS_OFFSET, just below).
                printf("STORE_DECLINATION ignored - declination is retired, use the compass offset\r\n");
                break;
            case STORE_COMPASS_OFFSET:
                // Same reason as SET_AS_NORTH: this is the table's own domain, not a display trim.
                if (cal8RefLocked())
                {
                    printf("STORE_COMPASS_OFFSET refused: a calibration is in progress\r\n");
                    break;
                }
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
                    response.status = IDLING; // Force a status sync back to Top
                    xQueueSend(serOut, (void *)&response, 10);
                }
                break;
            case SET_AS_NORTH:
                {
                    // Not while a calibration is running. Set as North moves compassOffset, which is
                    // the domain the table is indexed by, so it rotates the whole deviation curve -
                    // and a run measuring against that curve would end up combining readings from
                    // before and after the rotation. Nor is it needed around a run: the guided run's
                    // first step and the GPS run's north leg both set north themselves, at commit
                    // time, from a measurement rather than from an eyeball.
                    if (cal8RefLocked())
                    {
                        printf("SET_AS_NORTH refused: a calibration is in progress and it sets north "
                               "itself when it commits\r\n");
                        extern QueueHandle_t buzzer;
                        if (buzzer != NULL) beep(-1, buzzer);
                        break;
                    }
                    double newOffset = 0;
                    bool success = false;
                    if (mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(500))) {
                        // Solved through the correction curve rather than assuming the offset
                        // moves the reported heading one for one - see computeSetAsNorthOffset().
                        // The old "compassOffset - dirMag" needed three or four presses to creep
                        // onto north; this lands on it first time.
                        extern float computeSetAsNorthOffset(void);
                        newOffset = computeSetAsNorthOffset();
                        mainData.compassOffset = newOffset;
                        ser->compassOffset = newOffset;
                        success = true;
                        xSemaphoreGive(mainDataMutex);
                    }
                    if (success) {
                        CompasOffset(&mainData, MEM_PUT);
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
                        response.status = IDLING;
                        xQueueSend(serOut, (void *)&response, 10);
                    }
                }
                break;
            case REBOOT:
                {
                    printf("REBOOT command received! Rebooting Sub...\r\n");
                    extern QueueHandle_t buzzer;
                    if (buzzer != NULL) {
                        beep(10, buzzer);
                    }
                    delay(500);
                    ESP.restart();
                }
                break;
            case CALC_COMPASS_OFFSET:
                vTaskSuspend(compassTaskHandle);
                CalibrateCompass();
                vTaskResume(compassTaskHandle);
                {
                    RoboStruct response = mainData;
                    response.status = IDLING;
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
                        speedMaxMin(&response, MEM_GET);
                        xQueueSend(serOut, (void *)&response, 10);
                    }
                    break;
                case MAXMINPWRSET:
                global_params_rev++;
                printf("New Speed settings Max:%d Min:%d Pivot:%0.2f\r\n", dataIn.maxSpeed, dataIn.minSpeed, dataIn.pivotSpeed);
                speedMaxMin(&dataIn, MEM_PUT);
                speedMaxMin(ser, MEM_GET);
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
                    pidRudderParameters(&response, MEM_GET);
                    pidSpeedParameters(&response, MEM_GET);
                    speedMaxMin(&response, MEM_GET);
                    CompasOffset(&response, MEM_GET);
                    thrusterSwap(&response, MEM_GET);
                    thrusterInversion(&response, MEM_GET);
                    computeParameters(&response, MEM_GET); 
                    {
                        // The harmonic (Fourier) correction switch lives in compass.cpp, not in a
                        // MEM_GET helper - report the flag that is actually in effect so the Top
                        // and the CYD can show it.
                        extern volatile bool interp_enabled;
                        response.interpEnabled = interp_enabled;
                    }
                    udpLog("SETUPDATA reply -> IDr=%08lX", (unsigned long)response.IDr);
                    xQueueSend(serOut, (void *)&response, 10);
// printf("Sent SETUPDATA back to %X\r\n", response.IDr);
                }
                else if (dataIn.ack == SET)
                {
// printf("New setup received. Updating PID and Inversion flags.\r\n");
                    pidRudderParameters(&dataIn, MEM_PUT);
                    pidSpeedParameters(&dataIn, MEM_PUT);
                    speedMaxMin(&dataIn, MEM_PUT);
                    CompasOffset(&dataIn, MEM_PUT);
                    thrusterSwap(&dataIn, MEM_PUT);
                    thrusterInversion(&dataIn, MEM_PUT);
                    computeParameters(&dataIn, MEM_PUT);
                    
                    // Reload into running config
                    pidRudderParameters(ser, MEM_GET);
                    pidSpeedParameters(ser, MEM_GET);
                    speedMaxMin(ser, MEM_GET);
                    CompasOffset(ser, MEM_GET);
                    thrusterSwap(ser, MEM_GET);
                    thrusterInversion(ser, MEM_GET);
                    computeParameters(ser, MEM_GET);
                    // A setup save carries compassOffset with it, so an unrelated Save from any page
                    // would move the table's domain mid-run just as surely as Set as North. Keep
                    // what the run is working against; the rest of the frame still applies.
                    if (cal8RefLocked())
                    {
                        printf("SETUPDATA: keeping the running compass offset %.2f, a calibration is "
                               "in progress\r\n", mainData.compassOffset);
                        ser->compassOffset = mainData.compassOffset;
                        CompasOffset(&mainData, MEM_PUT);
                    }
                    else
                    {
                        mainData.compassOffset = ser->compassOffset; // compassTask uses it at once
                    }

                    // The Setup page's "Active Trim Enabled" tick box travels in THIS frame
                    // (SETUPDATA field 16, see RoboCode()), not in ADAPTIVE_TRIM. Until now only
                    // ADAPTIVE_TRIM acted on the flag, so saving the setup stored nothing and the
                    // 1 Hz ADAPTIVE_TRIM broadcast put the old state straight back on the Top's
                    // web page - the tick box looked like it had no effect at all.
                    // Only the FLAG is carried here; the accumulated trim value is not in this
                    // frame and must be left exactly as it is.
                    mainData.compass_trim_enabled = dataIn.compass_trim_enabled;
                    {
                        float trim_val = (float)mainData.compass_trim;
                        bool trim_en = mainData.compass_trim_enabled;
                        memCompassTrim(&trim_val, &trim_en, MEM_PUT);
                    }

                    // Harmonic (Fourier) correction on/off. The 8-point table itself is not in
                    // this frame and is never touched here - only whether it gets applied.
                    // dataIn.interpEnabled keeps its previous value when the sender did not
                    // specify the field (see the tri-state note in RoboCode()), so a short frame
                    // from an older node cannot switch this off behind the operator's back.
                    {
                        extern volatile bool interp_enabled;
                        bool want = dataIn.interpEnabled;
                        if (want != interp_enabled)
                        {
                            interp_enabled = want;
                            memInterpEnabled(&want, MEM_PUT);
                            printf("Harmonic correction switched %s by SETUPDATA\r\n", want ? "ON" : "OFF");
                        }
                        ser->interpEnabled = interp_enabled;
                        mainData.interpEnabled = interp_enabled;
                    }

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
            case CAL8_SESSION:
                {
                    // Remote end of the guided eight point calibration. Everything this does is a
                    // call into compass.cpp, which owns the rules; there is deliberately no
                    // arithmetic here. See the block comment there, and CAL8_SESSION in
                    // RoboCompute.h for the frame layout.
                    if (dataIn.ack == SET)
                    {
                        switch (dataIn.cal8Action)
                        {
                        case CAL8_BEGIN:
                            cal8Begin();
                            // Hold PWRENABLE and park the serial watchdog, exactly as the Sub's own
                            // page does, or the buoy can shut down halfway through a run driven from
                            // the CYD. mancalSessionBegin() is the call that does that -
                            // mancalHoldArm() was written here by mistake and only arms the "put the
                            // correction back on" safety net, which does nothing now that nothing
                            // switches the correction off.
                            mancalSessionBegin();
                            break;
                        case CAL8_LOCK:
                            // A calibration run elsewhere - the GPS Fourier run on the Top - taking
                            // this buoy's heading reference for its duration.
                            cal8Lock(true);
                            mancalSessionBegin();
                            break;
                        case CAL8_UNLOCK:
                            cal8Lock(false);
                            mancalSessionEnd();
                            break;
                        case CAL8_SET:
                            // The leg the sender meant. A press that crossed with an earlier copy of
                            // itself names a leg already captured and is dropped rather than eating
                            // the next one - see cal8Set().
                            if (cal8Set(dataIn.cal8Next) < 0)
                                printf("CAL8: set ignored - wrong leg, no session, or all eight captured\r\n");
                            mancalSessionPing();   // the operator is still working
                            break;
                        case CAL8_SAVE:
                            if (cal8Save()) mancalSessionEnd();
                            else printf("CAL8: save refused - not all eight captured\r\n");
                            break;
                        case CAL8_CANCEL:
                            cal8Cancel();
                            mancalSessionEnd();
                            break;
                        default:
                            printf("CAL8: unknown action %d\r\n", dataIn.cal8Action);
                            break;
                        }
                    }

                    // Answer every frame, GET or SET, with the state as it now stands. The sender
                    // never has to work out what a press did - and two screens watching the same
                    // run stay in step because neither is keeping its own count.
                    RoboStruct response = mainData;
                    response.IDs = mainData.mac;
                    response.IDr = dataIn.IDs;
                    response.cmd = CAL8_SESSION;
                    response.ack = INF;
                    response.cal8Action = dataIn.cal8Action;
                    response.cal8Active = cal8_active;
                    response.cal8Next = cal8_next;
                    for (int i = 0; i < 8; i++) response.cal8Captured[i] = cal8_captured[i];
                    xQueueSend(serOut, (void *)&response, 10);
                }
                break;

            case STORE_INTERPOLATION_TABLE:
                {
                    // The Fourier correction table lives in compass.cpp; subwifi.cpp reaches it
                    // the same way, with local externs rather than a header entry.
                    extern float measured_angles[9];
                    extern void computeFourierCoefficients();
                    extern volatile bool interp_enabled;
                    extern bool interp_table_usable;

                    if (dataIn.ack == GET || dataIn.ack == GETACK)
                    {
                        RoboStruct response = mainData;
                        response.IDs = mainData.mac;
                        response.IDr = dataIn.IDs;
                        response.cmd = STORE_INTERPOLATION_TABLE;
                        response.ack = INF;
                        // Report the table that is actually IN EFFECT, not the one in NVS. With
                        // the harmonic correction switched off the compass is uncorrected, so the
                        // effective table is the identity - and the Top builds its new table on
                        // top of whatever we report here. Reporting a stored-but-unused table
                        // would make it subtract the same deviation twice.
                        for (int i = 0; i < 8; i++)
                            response.interpolationTable[i] = interp_enabled ? measured_angles[i] : (float)(i * 45);
                        // Say so on the wire. Without this the identity table above is
                        // indistinguishable from a genuinely uncalibrated buoy - exactly the
                        // ambiguity that made every UI display zero corrections.
                        response.interpEnabled = interp_enabled;
                        response.interpUsable = interp_table_usable;
                        // A calibration is starting. See mancalHoldService().
                        mancalHoldArm();
                        xQueueSend(serOut, (void *)&response, 10);
                        printf("Sent interpolation table (harmonic correction %s)\r\n", interp_enabled ? "ON" : "OFF");
                    }
                    else if (dataIn.ack == SET)
                    {
                        // Only an explicit SET writes. Our own INF reply is already dropped by the
                        // IDs != mac echo filter in SercomTask, but keying on SET means a frame
                        // that ever did get echoed back could not start a store/reply ping-pong
                        // across NVS.
                        mancalHoldDisarm(); // the session committed a table

                        // The same door the guided run uses: it forces north to zero, folding the
                        // rotation into compassOffset, writes both, and re-enables the correction.
                        // The GPS Fourier run sends a table with north's own deviation still in
                        // entry 0, so without this a GPS calibration silently gave up the property
                        // that makes north true with the correction switched off.
                        bool usable = storeInterpolationTable(dataIn.interpolationTable);

                        // Audible acknowledgement from the buoy itself. Deliberately after the NVS
                        // write, so a beep can only ever mean the table really landed - and a
                        // different tone when the result is unusable, because a cheerful beep on a
                        // table the buoy is about to ignore is worse than no beep at all.
                        {
                            extern QueueHandle_t buzzer;
                            if (buzzer != NULL) beep(usable ? 1000 : -1, buzzer);
                        }

                        // Echo back what actually landed in NVS - which is the ROTATED table, not
                        // the one that was sent. Senders compare the echo against what they sent,
                        // so they have to compare it relative to entry 0; see gpscalib.cpp.
                        RoboStruct response = mainData;
                        response.IDs = mainData.mac;
                        response.IDr = dataIn.IDs;
                        response.cmd = STORE_INTERPOLATION_TABLE;
                        response.ack = INF;
                        for (int i = 0; i < 8; i++)
                            response.interpolationTable[i] = measured_angles[i];
                        response.interpEnabled = interp_enabled; // true here - a store switches it on
                        // Whether the buoy can actually USE it. An out-of-order table is stored and
                        // then ignored, and until now nothing said so: the sender was told the store
                        // succeeded while the buoy sailed on with no compass correction at all.
                        response.interpUsable = usable;
                        xQueueSend(serOut, (void *)&response, 10);
                    }
                }
                break;
            case HARDIRONFACTORS:
                dataIn.mac = espMac();
                hardIron(&dataIn, MEM_PUT);
                InitCompass();
                break;
            case SOFTIRONFACTORS:
                dataIn.mac = espMac();
                softIron(&dataIn, MEM_PUT);
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

//***************************************************************************************************
//  MAN CAL harmonic hold - the last line of defence for an abandoned calibration
//
//  Every front end that runs a manual calibration has to switch this buoy's harmonic correction
//  off to dial against the raw compass, and switch it back on when it leaves. There are four of
//  them (handheld screen, dashboard, this Sub's own page, the Top's page) and they cannot be
//  collapsed into one - a submerged Sub has no WiFi, so the remote paths are not optional. Relying
//  on all four to get the restore right in every exit path has already failed once, and a buoy left
//  with the correction off reports the identity table to everything that asks, so its stored
//  calibration goes invisible and it sails uncorrected.
//
//  So the buoy now guarantees it for itself, and all four inherit it for free.
//
//  Armed only by answering a table GET, which is the unambiguous "a calibration is starting"
//  signal - deliberately NOT by the correction merely being off, because switching it off from the
//  Setup page is a legitimate thing to do and must not be undone behind the operator's back.
//  Disarmed by a table SET (the session saved) or by the correction coming back on by any route.
//
//  The timeout has to span a whole unattended session, because the remote front ends send this
//  buoy no heartbeat - only the Top's page and this Sub's own page have their own, shorter ones.
//***************************************************************************************************
static bool mancalHoldArmed = false;
static unsigned long mancalHoldArmedAt = 0;
// How often the Sub reports to the Top over the shared serial wire. See the comment at the send
// site: this is a collision domain, and every frame the Sub sends is a window in which the Top
// cannot reach it. Raise it and remote commands start getting lost; there is no benefit in lowering
// it, because the steering that needs fast compass data runs in the Sub.
#define SUBDATA_SERIAL_INTERVAL_MS 250

#define MANCAL_HOLD_TIMEOUT_MS (20 * 60 * 1000UL)

void mancalHoldArm()
{
    mancalHoldArmed = true;
    mancalHoldArmedAt = millis();
}

void mancalHoldDisarm()
{
    mancalHoldArmed = false;
}

void mancalHoldService()
{
    extern volatile bool interp_enabled;
    if (!mancalHoldArmed) return;

    // Back on by any route - the session finished, or someone re-enabled it. Nothing to guard.
    if (interp_enabled) { mancalHoldArmed = false; return; }

    if (millis() - mancalHoldArmedAt <= MANCAL_HOLD_TIMEOUT_MS) return;

    mancalHoldArmed = false;
    bool enable = true;
    interp_enabled = true;
    memInterpEnabled(&enable, MEM_PUT);
    global_params_rev++;
    printf("MANCAL: no calibration activity for %lu ms and the harmonic correction was still off - "
           "switching it back on so this buoy does not sail uncorrected\r\n", MANCAL_HOLD_TIMEOUT_MS);
}

//***************************************************************************************************
//  MAN CAL session - see the block comment in main.h
//***************************************************************************************************
static volatile bool mancalActive = false;
static volatile unsigned long mancalLastPing = 0;
// Generous next to the page's 400 ms poll, so a slow buoy or a stuttering WiFi link never drops a
// session out from under someone who is still standing there dialing it in.
#define MANCAL_SESSION_TIMEOUT_MS (2 * 60 * 1000UL)

void mancalSessionBegin()
{
    mancalActive = true;
    mancalLastPing = millis();
    printf("MANCAL: session started - holding PWRENABLE and the serial watchdog off\r\n");
}

void mancalSessionPing()
{
    if (mancalActive) mancalLastPing = millis();
}

void mancalSessionEnd()
{
    if (!mancalActive) return;
    mancalActive = false;
    printf("MANCAL: session ended - normal power and watchdog timers resume\r\n");
}

bool mancalSessionAlive()
{
    if (!mancalActive) return false;
    if (millis() - mancalLastPing > MANCAL_SESSION_TIMEOUT_MS)
    {
        // The page stopped talking - browser closed, tab slept, WiFi dropped. Do not leave the
        // buoy sitting in REMOTE with the supply pinned on: stop it and hand back to the timers.
        mancalActive = false;
        printf("MANCAL: no contact from the page for %lu ms - ending session and idling\r\n",
               MANCAL_SESSION_TIMEOUT_MS);
        mainData.tgSpeed = 0;
        mainData.tgDist = 0;
        mainData.status = IDLING;
        return false;
    }
    return true;
}

/**
 * @brief Checks for serial communication timeouts and triggers safety shutdowns.
 * 
 * This function monitors the time elapsed since the last serial communication. 
 * If no data is received for 5 seconds, it transitions the system to a safe 
 * IDLING state, halts the motors, and turns the status LED red to indicate a 
 * timeout. If communication is lost for an extended period (5 minutes), it 
 * initiates a complete power-down sequence to preserve battery life, disabling 
 * the power supply and ESCs.
 * 
 * @param ser Pointer to the main `RoboStruct` containing system state and timestamps for last communication.
 * @return void This function does not return a value.
 */
void handleSerialTimeOut(RoboStruct *ser)
{
    // A MAN CAL session is driven from the Sub's own web page, with no Top on the other end of the
    // serial link. Both timers below are fed only by traffic FROM the Top, so left alone they would
    // stop the pivot after 5 s and cut the supply after POWEROFFTIME - in the middle of the
    // calibration the operator is standing there doing.
    if (mancalSessionAlive())
    {
        digitalWrite(PWRENABLE, 1); // keep the supply latched on for the whole session
        ser->lastSerIn = millis();
        PwrOff = millis();
        return;
    }

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
            ser->status = IDLING;
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
    case IDLING:
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

    // Telemetry up to the Top. This used to go out every 100 ms, and the Top-to-Sub link is a
    // single half-duplex wire, so at 10 frames a second the Sub was talking over most of what the
    // Top tried to say to it: eight calibration presses sent one per second landed twice. Nothing
    // needs the old rate - the heading loop that actually needs fast compass data is right here in
    // the Sub, and the Top only relays this on for display and for the map. Four a second is still
    // smoother than any screen refresh in this project.
    if (nextSamp < millis())
    {
        nextSamp = SUBDATA_SERIAL_INTERVAL_MS + millis();
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
        nextTrimSendTime = 1000 + millis(); // Every 1 second (fast update!)
        
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
            memCompassTrim(&trim_val, &trim_en, MEM_PUT);
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
    {
        char tag[20];
        snprintf(tag, sizeof(tag), "SUB-%08lx", (unsigned long)espMac());
        udpLogBegin(tag);
    }

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
        //      Status change handling
        //***************************************************************************************************
        handleStatus(&mainData);
        //***************************************************************************************************
        //      Timer routines
        //***************************************************************************************************
        handleTimerRoutines(&mainData);
        //***************************************************************************************************
        //      Check front key
        //***************************************************************************************************
        handleKeyPress();
        //***************************************************************************************************
        //      New compass data
        //***************************************************************************************************
        xQueueReceive(compass, (void *)&mainData.dirMag, 0);
        // Imag - the same heading but before the eight point table and the adaptive trim, which is
        // the value that table is indexed by. Published so a calibration can read it directly
        // instead of having to switch the correction off first. See RoboStruct::imag.
        mainData.imag = GetHeadingNoOffset();
        //***************************************************************************************************
        //      new serial message and udp
        //***************************************************************************************************
        handleSerandRfdata(&mainData);
        //***************************************************************************************************
        //      Serial watchdog
        //***************************************************************************************************
        handleSerialTimeOut(&mainData);
        mancalHoldService();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
