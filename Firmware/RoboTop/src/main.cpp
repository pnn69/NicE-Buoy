#include <Arduino.h>
#include "esp_log.h"
#include <PID_v1.h>
#include "main.h"
#include "io_top.h"
#include "leds.h"
#include "topwifi.h"
#include "gps.h"
#include "datastorage.h"
#include "buzzer.h"
#include "adc.h"
#include "loratop.h"
#include "sercom.h"
#include "gpscalib.h"
#include "gpssim.h"
#include "udplog.h"

// Every beep, tagged with the line that asked for it. Sound is the buoy's only feedback in the
// field, so knowing which beep fired - and whether the next line is a fresh boot banner - is what
// separates "it acknowledged the command" from "it crashed and restarted".
// The high-rate telemetry commands. Logging every one of these put roughly 31 blocking UDP sends
// a second on the wire from the loop task - 98% of it noise, and a plausible way to wedge lwIP.
// Both Tops hung (rather than panicking and recovering) on builds that did it, so the traffic is
// now confined to the commands we are actually hunting.
static inline bool noisyCmd(int cmd)
{
    return cmd == TOPDATA || cmd == BUOYPOS || cmd == SUBDATA || cmd == SUBACCU ||
           cmd == SUBPWR || cmd == DIRMDIRTGDIRG || cmd == DIRDIST || cmd == ADAPTIVE_TRIM;
}

static inline void beepLogged(int sound, QueueHandle_t q, int line)
{
    udpLog("BEEP tone=%d from main.cpp:%d", sound, line);
    beep(sound, q);
}
#define beep(s, q) beepLogged((s), (q), __LINE__)

// Give the Arduino loop task real headroom. RoboStruct is ~500 bytes and the receive path
// (loop -> handleRfData -> AddDataToBuoyBase -> MergeBuoyData) keeps several of them live at
// once, on top of the 3 KB JSON work elsewhere. The 8 KB default left very little margin, and a
// stack overflow on an ESP32 is not a clean error - it panics and reboots.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

// #define BUFLENMHRG 60 // one sampel each sec so 60 sec for stabilisation
RoboStruct mainData;
// RoboStruct b0, b1, b2;
// RoboStruct *buoyPara[4] = {&b0, &b1, &b2};
RoboStruct buoyPara[3] = {};
RoboStruct *buoyParaPtrs[3] = {&buoyPara[0], &buoyPara[1], &buoyPara[2]};
static RoboStruct mainUdpIn;
static RoboWindStruct wind;
static LedData mainCollorStatus;
static LedData mainCollorUtil;
static LedData mainCollorGps;
static PwrData mainPwrData;
static Buzz mainBuzzerData;
static int wifiConfig = 0;
static int msg;
static int presses = -1;
static int blink = BLINK_FAST;
static int loralstmsg = 0;
static unsigned long loraTimerOut = millis();
static unsigned long loraTimerIn = millis();
static unsigned long udpTimerOut = millis();
static unsigned long buttonBlinkTimer = millis();
static unsigned long logtimer = millis();
static unsigned long remotetimer = millis();
static unsigned long updateSubtimer = millis();
static unsigned int gpsErrorCnt = 0;
static unsigned int distErrorCnt = 0;
static uint64_t lastSetupRequester = 0;
static unsigned long lastRealSerIn = 0;
static bool isSerConnected = false;

static QueueHandle_t keyPressQueue = NULL;

bool isLongPress = false;
bool oneSecondBeepReported = false;
unsigned long pressStartTime = 0;
unsigned long lastReleaseTime = 0;
int pressCount = 0;

static TaskHandle_t buttonTaskHandle = NULL;

void IRAM_ATTR buttonISR()
{
    static volatile uint64_t last_isr_time = 0;
    uint64_t current_time = esp_timer_get_time();
    if (current_time - last_isr_time > 50000) // 50ms hardware debounce threshold
    {
        last_isr_time = current_time;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (buttonTaskHandle != NULL)
        {
            vTaskNotifyGiveFromISR(buttonTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void buttonTask(void *arg);
int countKeyPressesWithTimeoutAndFinalLongPress();

//***************************************************************************************************
//  new pid stuff
//***************************************************************************************************
double Setpoint, Input, Output;
double Kp = 20, Ki = 0.05, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//***************************************************************************************************
//      Setup
//***************************************************************************************************
/**
 * @brief Initializes the system, peripherals, and tasks.
 * 
 * This function sets up serial communication, PID controller, pins, 
 * memory, and various queues. It also creates several FreeRTOS tasks 
 * for buzzer, LED, GPS, WiFi, Serial, and LoRa operations.
 */
void setup()
{
    Serial.begin(115200);
    Setpoint = 0;
    myPID.SetMode(AUTOMATIC); // turn the PID on
    myPID.SetOutputLimits(-100, 100);

    pinMode(BUTTON_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_LIGHT_PIN, OUTPUT);
    delay(100);
    printf("\r\n\r\nRobobuoy Top Version: %0.1f\r\n", TOPVERSION);
    printf("Build Date: %s %s\r\n", __DATE__, __TIME__);
    printf("\r\nSetup running!...\r\n");
    printf("Initializing memory...\r\n");
    initMemory();
    printf("Initializing peripherals...\r\n");
    initbuzzerqueue();
    printf("Initializing led queues...\r\n");
    initledqueue();
    printf("Initializing WiFi queue...\r\n");
    initwifiqueue();
    printf("Initializing GPS queue...\r\n");
    initgpsqueue();
    printf("Initializing LoRa queue...\r\n");
    initloraqueue();
    printf("Initializing Serial communication queue...\r\n");
    initserqueue();

    // Wake the Sub before anything else is asked of it. The Sub may well be asleep when we come
    // up, and until now nothing here would ever have woken it: the only two WAKEUP triggers are
    // a status change out of IDLE (handleStatus) and the serial watchdog, and the watchdog is
    // gated on isSerConnected - which only becomes true once the Sub has actually spoken. A Sub
    // that was already asleep at boot therefore stayed asleep until somebody pressed the button.
    //
    // Queued here rather than at the end of setup() because serOut exists from initserqueue()
    // above and no task that could put anything else into it has been created yet, so this is
    // genuinely the first thing SercomTask does once it has brought Serial1 up. It is a local
    // instruction, not a frame: SercomTask turns it into the physical wake pulse on the shared
    // TX line (sercom.cpp, case WAKEUP), so it costs nothing if the Sub was awake all along.
    {
        RoboStruct wakeupMsg;
        wakeupMsg.cmd = WAKEUP;
        xQueueSend(serOut, (void *)&wakeupMsg, 0);
    }
    printf("Creating task buzzer...\r\n");
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 2048, NULL, 1, NULL, 1);
    printf("Creating task led...\r\n");
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    printf("Creating task gps...\r\n");
    xTaskCreatePinnedToCore(GpsTask, "GpsTask", 2000, NULL, configMAX_PRIORITIES - 8, NULL, 1);
    if (digitalRead(BUTTON_PIN) == true)
    {
        delay(100);
        if (digitalRead(BUTTON_PIN) == true)
        {
            wifiConfig = 1; // put in to pair mode
        }
    }
    else
    {
        wifiConfig = 0; // Setup normal accespoint
    }
    printf("Creating task WiFi...\r\n");
    // 8 KB, not 4 KB: WiFiTask runs the whole WebServer (request parsing, ~3 KB /data JSON
    // building, SPIFFS file streaming) plus the UDP handling, and 4 KB left no margin.
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8192, &wifiConfig, configMAX_PRIORITIES - 10, NULL, 0);
    printf("Creating task Serial communication...\r\n");
    // 4000 left this task 1248 bytes of headroom, measured with uxTaskGetStackHighWaterMark().
    // It decodes into ~500-byte RoboStructs and builds Arduino Strings, so that is thinner than
    // it looks. Same reasoning as LoraTask below, one size down.
    xTaskCreatePinnedToCore(SercomTask, "SerialTask", 6000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    printf("Creating task LoRa communication...\r\n");
    // 4000 was not enough: /data reported StkLora = 100, i.e. this task had 100 BYTES of stack
    // left while every other task had kilobytes. That is what the DOCK panics were - a canary
    // trip, which is why every task's breadcrumb pointed at an idle wait rather than at the code
    // that overflowed. rfCode() returns an Arduino String and onReceive() decodes into a ~500-byte
    // RoboStruct, both on this stack, and a dock puts several through it back to back.
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 8192, &mainData, configMAX_PRIORITIES - 2, NULL, 1);
    
    // Load local params
    mainData.mac = espMac();
    mainData.IDs = mainData.mac;
    mainData.status = IDLE;
    // Read the dock into a scratch struct, NOT into mainData. memDockPos() writes tgLat/tgLng,
    // which is the live navigation target - loading it here left every buoy sitting at boot with
    // the dock already entered as its destination. Any route into LOCKED that does not set a
    // target of its own (the COMPUTESTART/COMPUTETRACK bail-outs, SENDTRACK, a LOCKED arriving
    // over serial) then inherited it and sailed for the dock, kilometres away. Leaving tgLat/tgLng
    // at 0 means those paths hit the "no target" guards in handleTimerRoutines() and hold station
    // instead. The dock is read from NVS on demand by the DOCKING case, which is the only place
    // that actually wants it.
    RoboStruct dockCfg;
    memDockPos(&dockCfg, MEM_GET);
    memDockApproach(&mainData, MEM_GET);

    // Automatically migrate stored NVM dock position to the new requested default
    // printf("Updating stored Dock Position to the new default: 52.29302221327865, 4.932541137977593\r\n");
    // mainData.tgLat = 52.29302221327865;
    // mainData.tgLng = 4.932541137977593;
    // memDockPos(&mainData, MEM_PUT);


    // Nothing else is restored here, on purpose. Thruster inversion and swap describe the
    // wiring at the Sub - esc.cpp is the only code in the system that applies them - and the
    // compass, its offset and both PID loops live on the Sub as well. The Sub is the single
    // owner of all of it: the Top asks for SETUPDATA as soon as the serial link comes up
    // (handleSerialData) and keeps the answer in RAM to display and relay, nothing more.
    //
    // A second copy in the Top's own flash is what made a Sub moved to another hull look like
    // corrupted settings - each Top went on reporting its stale cached values until the new Sub
    // got round to answering.

    // Print all loaded parameters from NVM to Serial Port
    printf("\r\n==================================================\r\n");
    printf("        LOADED PARAMETERS FROM NVM / FLASH        \r\n");
    printf("==================================================\r\n");
    printf("MAC/ID          : %08lX\r\n", mainData.mac);
    printf("Last reset      : %s\r\n", resetReasonText());
    printf("Dock Position   : Lat=%.12f, Lng=%.12f\r\n", dockCfg.tgLat, dockCfg.tgLng);
    printf("Dock Approach   : Dist=%d m, Dir=%d deg, ToWayPoint=%s\r\n", mainData.dockApproachDist, mainData.dockApproachDir, mainData.dockingToWaypoint ? "True" : "False");
    printf("Thrusters, PID and compass offset are the Sub's - asked for at serial link-up\r\n");
    printf("==================================================\r\n\r\n");

    // Initialize background task-based button handling
    keyPressQueue = xQueueCreate(5, sizeof(int));
    xTaskCreatePinnedToCore(buttonTask, "buttonTask", 2048, NULL, 3, &buttonTaskHandle, 1);

    // Register GPIO hardware interrupt on BUTTON_PIN
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);

    Serial.println("Main task running!");
    // defautls(&mainData);
}

//***************************************************************************************************
//      keypress detection (Background Task Driven Polling)
//***************************************************************************************************
#define LONG_PRESS_DURATION 5000 // ms
#define PRESS_TIMEOUT 500       // ms

/**
 * @brief Detects and counts button presses with timeout and long-press support.
 * Runs in the background buttonTask.
 * 
 * @return int The number of short presses, or 100 + short press count for a sequence ending in a long press. 
 *         Returns -1 if no action is completed.
 */
int countKeyPressesWithTimeoutAndFinalLongPress()
{
    unsigned long currentTime = millis();
    int rawState = digitalRead(BUTTON_PIN);
    
    // Integrating debounce: button state must be stable for 50ms
    static int stableState = LOW;
    static int lastRawState = LOW;
    static unsigned long lastStateChangeTime = 0;
    
    if (rawState != lastRawState)
    {
        lastStateChangeTime = currentTime;
        lastRawState = rawState;
    }
    
    if ((currentTime - lastStateChangeTime) >= 25) // Must remain stable for 50ms
    {
        if (rawState != stableState)
        {
            stableState = rawState;
            
            // Stable press transition (LOW -> HIGH)
            if (stableState == HIGH)
            {
                pressStartTime = currentTime;
                pressCount++;
                isLongPress = false;
                oneSecondBeepReported = false; // Reset on press
                
                // Play instant feedback beep at the exact moment of physical press (true single 100ms tone)
                if (buzzer != NULL)
                {
                    beep(1000, buzzer);
                }
            }
            // Stable release transition (HIGH -> LOW)
            else
            {
                lastReleaseTime = currentTime;
            }
        }
    }
    
   
    // One second hold beep detection (button held HIGH for more than 1000ms)
    if (stableState == HIGH && (currentTime - pressStartTime > 1000) && !oneSecondBeepReported)
    {
        oneSecondBeepReported = true;
        isLongPress = true;
        if (buzzer != NULL)
        {
            beep(1000, buzzer); // Beep again with a true single 100ms tone!
        }
    }
    
    // Timeout check: button must be stable LOW, some presses registered, and PRESS_TIMEOUT elapsed since last release
    if (stableState == LOW && pressCount > 0 && (currentTime - lastReleaseTime > PRESS_TIMEOUT))
    {
        int result = pressCount;
        if (isLongPress)
        {
            result = 100 + pressCount;
        }
        
        // Reset states for the next sequence
        pressCount = 0;
        isLongPress = false;
       
        return result;
    }
    
    return -1;
}

/**
 * @brief Background task dedicated to polling and debouncing the button at a high frequency (10ms).
 * This eliminates missed presses completely, even when other main task operations block or run slow.
 */
void buttonTask(void *arg)
{
    while (true)
    {
        // Wait for interrupt notification.
        // If a press sequence is in progress, we use a short timeout (10ms) to continue polling and tracking the state machine.
        // If no press is in progress and we are idle, we can block indefinitely (portMAX_DELAY) to save CPU and respond instantly on change.
        TickType_t waitTicks = (pressCount > 0 || digitalRead(BUTTON_PIN) == HIGH) ? pdMS_TO_TICKS(10) : portMAX_DELAY;
        ulTaskNotifyTake(pdTRUE, waitTicks);

        int presses = countKeyPressesWithTimeoutAndFinalLongPress();
        if (presses > 0)
        {
            if (keyPressQueue != NULL)
            {
                xQueueSend(keyPressQueue, &presses, 0);
            }
        }
    }
}

//***************************************************************************************************
//      Short, Short, Short	3
//      Short, Short, Long	103
//      Short, Long, Short	1 (resets after invalid long in middle, or could be ignored)
//      Long only	0
//      Short, Short, Short, Long	104
//      key press stuff
//      One press: lock/unlock
//      Two press: start line computation
//      Three press: compute track
//      Five press: sail to dock position
//      Four short presses and one long: store as docposition
//      nine short presses and onle long: start calibration of magnetic compass
//      Seven short presses and one long: start the GPS Fourier compass calibration (8 legs, ~30 min)
//***************************************************************************************************
/**
 * @brief Processes button press results and updates the buoy status.
 * 
 * @param key Pointer to the RoboStruct containing the buoy's state and data.
 */
void handleKeyPress(RoboStruct *key)
{
    int presses = -1;
    if (keyPressQueue != NULL && xQueueReceive(keyPressQueue, &presses, 0) == pdTRUE)
    {
        if (presses > 0)
        {
            switch (presses)
            {
            case 1: // lock / unlock
                if ((key->status != LOCKED) && (key->status != DOCKED))
                {
                    key->status = LOCKING;
                }
                else
                {
                    key->status = IDLING;
                }
                key->loralstmsg = 0;
                break;
            case 102:
                key->status = COMPUTESTART;
                break;
            case 103:
                key->status = COMPUTETRACK;
                break;
            case 5:
                key->status = DOCKING;
                break;
            case 105:
                key->status = STOREASDOC;
                break;
            case 10:
                key->status = CALC_COMPASS_OFFSET;
                break;
            case 110:
                key->status = START_CALIBRATE_MAGNETIC_COMPASS;
                break;
            case 108:
                // The button cannot ask which mode you want, so it picks the conservative one:
                // pair averaging can only ever leave a residual error, while still-water mode on
                // a day with a current writes that current into the compass table. Use the web
                // Setup dialog to choose still water deliberately.
                key->gpsCalStillWater = false;
                key->status = GPS_FOURIER_CALIBRATE;
                break;
            default:
                beep(-1, buzzer);
                break;
            }
        }
    }
}

//***************************************************************************************************
//      Light button control
//***************************************************************************************************
/**
 * @brief Controls the button's LED behavior based on the buoy's status and GPS fix.
 * 
 * @param sta Pointer to the RoboStruct containing the buoy's status and GPS fix state.
 */
void buttonLight(RoboStruct *sta)
{
    static int lastCalibState = 0;
    int currentCalibState = (sta->status == CALIBRATE_MAGNETIC_COMPASS || sta->status == INFIELD_CALIBRATE ||
                             sta->status == GPS_FOURIER_CALIBRATE) ? 1 : 0;
    
    if (currentCalibState != lastCalibState) {
        if (currentCalibState == 1) {
            mainCollorStatus.color = CRGB::Purple;
            mainCollorStatus.blink = BLINK_FAST;
        } else {
            mainCollorStatus.color = CRGB::Black;
            mainCollorStatus.blink = BLINK_OFF;
            if (sta->status == ERROR) beep(3, buzzer);
            else beep(5, buzzer);
        }
        xQueueSend(ledStatus, (void *)&mainCollorStatus, 0);
        lastCalibState = currentCalibState;
    }

    if (buttonBlinkTimer < millis())
    {
        buttonBlinkTimer = millis() + blink;
        if (sta->status == LOCKED || sta->status == DOCKED)
        {
            digitalWrite(BUTTON_LIGHT_PIN, HIGH);
            blink = 2000;
        }
        else if (sta->status == CALIBRATE_MAGNETIC_COMPASS)
        {
            digitalWrite(BUTTON_LIGHT_PIN, !digitalRead(BUTTON_LIGHT_PIN));
            blink = 50;
        }
        else
        {
            digitalWrite(BUTTON_LIGHT_PIN, !digitalRead(BUTTON_LIGHT_PIN));
            if (sta->gpsFix == true)
            {
                if (blink != 1000)
                {
                    mainCollorGps.color = CRGB::Green;
                    mainCollorGps.blink = BLINK_SLOW;
                    xQueueSend(ledGps, (void *)&mainCollorGps, 0); //
                    blink = 1000;
                }
            }
            else
            {
                if (blink != 100)
                {
                    mainCollorGps.color = CRGB::DarkRed;
                    mainCollorGps.blink = BLINK_FAST;
                    xQueueSend(ledGps, (void *)&mainCollorGps, 0); //
                    blink = 100;
                }
            }
        }
    }
}

/**
 * @brief Copies THIS buoy's freshly computed track position out of the shared buoy base.
 *
 * Slot 0 of buoyPara is always us - loop() refills it from mainData every pass and clears any
 * other slot that claims our identity. It has to be read HERE, in the same handleStatus() call
 * that computed it: SENDTRACK runs on the NEXT loop iteration, and by then "buoyPara[0] = mainData"
 * at the top of the loop has already overwritten the computed target with our old lock position.
 *
 * Identifying ourselves by ID instead does not work, which is what the old SENDTRACK code tried:
 * mainData.IDs holds the SUB's MAC (re-synced in handleSerialData) or 0, never the Top MAC it was
 * compared against, so the copy never happened and this buoy stayed on its lock position while the
 * others sailed to their ends of the line.
 */
static void adoptOwnTrackTarget(RoboStruct *stat, RoboStruct buoyPara[3])
{
    if (buoyPara[0].tgLat == 0.0 || buoyPara[0].tgLng == 0.0)
    {
        printf("#Own track position was not computed - holding current position\r\n");
        return;
    }
    stat->tgLat = buoyPara[0].tgLat;
    stat->tgLng = buoyPara[0].tgLng;
    stat->trackPos = buoyPara[0].trackPos;
    printf("#Own track position = ");
    trackPosPrint(stat->trackPos);
    printf(" (%.12f,%.12f)\r\n", stat->tgLat, stat->tgLng);
}

//***************************************************************************************************
//  status actions
//***************************************************************************************************
/**
 * @brief Handles state machine transitions and associated actions for the buoy.
 * 
 * @param stat Pointer to the current buoy's RoboStruct.
 * @param buoyPara Array of RoboStructs for all buoys in the system.
 */
// How many buoys are holding a lock position right now.
//
// COMPUTESTART squares the start line through the two ends' LOCK positions, and COMPUTETRACK
// needs a third for the upwind mark. Run against buoys that are not locked, the arithmetic uses
// whatever stale tgLat/tgLng they happen to be carrying - a dock position kilometres away, or the
// leftovers of an earlier course - and the "start line" that comes out has nothing to do with
// where the fleet is. Slot 0 is us, so it is tested through stat rather than through buoyPara,
// which is only refreshed when a report comes in.
//
// LOCKING counts as locked: the buoy owns a real lock position and is on its way to it. DOCKED
// and DOCKING deliberately do not - a buoy going home is not part of a course.
static int lockedBuoyCount(RoboStruct *stat, RoboStruct buoyPara[3])
{
    int n = 0;
    if (stat->status == LOCKED || stat->status == LOCKING) n++;
    for (int i = 1; i < 3; i++)
    {
        if (buoyPara[i].IDs == 0) continue;
        if (buoyPara[i].status == LOCKED || buoyPara[i].status == LOCKING) n++;
    }
    return n;
}

// Ask our own Sub to report its stored settings. See the call site in the serial handler for
// why this is on a timer and not only on connect. reason == NULL keeps the periodic tick quiet in
// the log; a one-off request passes a string so it is traceable.
#define SUB_SETUP_RESYNC_MS 20000
static unsigned long lastSubSetupResyncMs = 0;

static void requestSubSetup(const char *reason)
{
    lastSubSetupResyncMs = millis();

    RoboStruct req = {};
    req.cmd = SETUPDATA;
    req.ack = GET;
    req.IDr = BUOYIDALL;
    req.IDs = espMac();
    if (xQueueSend(serOut, (void *)&req, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        printf("ERROR: Failed to queue SETUPDATA request to serOut!\r\n");
    }
    else if (reason != NULL)
    {
        printf("#Asked the Sub for its setup (%s)\r\n", reason);
    }
}

void handleStatus(RoboStruct *stat, RoboStruct buoyPara[3])
{
    static int lastStatus = IDLE;
    if (lastStatus == IDLE && stat->status != IDLE && stat->status != IDLING)
    {
        RoboStruct wakeupMsg;
        wakeupMsg.cmd = WAKEUP;
        xQueueSend(serOut, (void *)&wakeupMsg, 0);
    }
    lastStatus = stat->status;

    RoboStruct LoraTx;
    // stat->mac, not stat->buoyId: buoyId is never assigned anywhere in RoboTop, so this line used
    // to zero our own sender ID on every pass of the main loop. Both transports patch a zero back
    // to the MAC on the way out, which is why it was invisible - but buoyPara[0] is filled from
    // mainData at the top of the loop, so the buoy base and the dashboard saw ID 0 for us. This
    // now matches what handleTimerRoutines() already does a few lines earlier in the same loop.
    stat->IDs = stat->mac;
    LoraTx = *stat;
    stat->IDr = BUOYIDALL;
    switch (stat->status)
    {
    case IDLING:
        beep(2, buzzer); // Play confirmation beep sequence upon successfully entering idle/unlocking mode
        stat->cmd = IDLE;
        stat->status = IDLE;
        xQueueSend(udpOut, (void *)stat, 0);  // update WiFi
        xQueueSend(loraOut, (void *)stat, 0); // update Lora
        stat->ack = GETACK;
        xQueueSend(serOut, (void *)stat, 20); // update sub
        break;
    case LOCKING:
        if (stat->gpsFix == true)
        {
            beep(1, buzzer); // Play the beautiful 3-tone arpeggio sequence upon successfully entering LOCK mode
            stat->status = LOCKED;
            stat->tgDist = 0.0;
            stat->cmd = LOCKPOS;
            // INF, not SET: stat->IDr is BUOYIDALL here, and an ACK for a broadcast can never
            // match in removeAckMsg(), so a SET entered the LoRa retry table and stayed there
            // for its full 5 retransmits - blocking the table and re-broadcasting the lock.
            // stat->ack is not reset below, so the RESET_SPEED_RUD_PID and DIRDIST sends that
            // follow inherited the same fate. Neither the Sub nor the other Tops look at ack
            // for these commands. Same rationale as the waypoint beacon further down.
            stat->ack = INF;
            stat->tgLat = stat->lat;
            stat->tgLng = stat->lng;
            AddDataToBuoyBase(*stat, buoyParaPtrs); // store position for later calculations Track positioning
            // IDr,IDs,ACK,MSG,LAT,LON
            xQueueSend(udpOut, (void *)stat, 0);   // update WiFi
            xQueueSend(loraOut, (void *)stat, 10); // send out through Lora
            RouteToPoint(stat->lat, stat->lng, stat->tgLat, stat->tgLng, &stat->tgDist, &stat->tgDir);
            
            /* 
             * IMPORTANT NAVIGATION SAFETY FIX:
             * On entering the LOCKED state from LOCKING, we must reset tgDist to 0.0.
             * This prevents any initial transient/stale distance values from being sent to the sub-unit.
             * Without this reset, the PID speed controllers would see a non-zero distance on the very first 
             * frame and immediately spin the motors at full burst before the routing calculations stabilize.
             */
            stat->tgDist = 0.0;
            
            stat->cmd = RESET_SPEED_RUD_PID;
            xQueueSend(serOut, (void *)stat, 0);  // send course and distance to sub
            xQueueSend(loraOut, (void *)stat, 0); // send course and distance to sub
            stat->cmd = DIRDIST;
            xQueueSend(serOut, (void *)stat, 0);  // send course and distance to sub
            xQueueSend(loraOut, (void *)stat, 0); // send course and distance to sub
        }
        else
        {
            beep(2, buzzer);
            stat->status = IDLE;
        }
        break;

    case DOCKING:
        crumb(10);
        udpLog("DOCK 10 enter gpsFix=%d lat=%.6f lng=%.6f", (int)stat->gpsFix, stat->lat, stat->lng);
        beep(1, buzzer); // Play the beautiful 3-tone arpeggio sequence upon successfully entering DOCK mode
        crumb(11);
        memDockPos(stat, MEM_GET);
        crumb(12);
        memDockApproach(stat, MEM_GET);
        crumb(13);
        udpLog("DOCK 13 loaded tg=%.6f,%.6f dist=%d dir=%d wp=%d",
               stat->tgLat, stat->tgLng, stat->dockApproachDist, stat->dockApproachDir,
               (int)stat->dockingToWaypoint);
        stat->status = DOCKED;
        stat->tgDist = 0.0;
        stat->tgDir = 0.0;
        printf("Retreved data for docking tgLat:%.8f tgLng:%.8f\r\n", stat->tgLat, stat->tgLng);
        printf("Retrieved waypoint for docking Direction: %d\xC2\xB0 distance: %dM Waypoint set %s\r\n", stat->dockApproachDir, stat->dockApproachDist, stat->dockingToWaypoint ? "YES" : "NO");
        // Broadcast the loaded dock position (tgLat and tgLng) to update the Web UI and LoRa network
        if (stat->dockingToWaypoint == true)
        {
            //change the waypoint with the given offset
            adjustPositionDirDist((double)stat->dockApproachDir, (double)stat->dockApproachDist, stat->tgLat, stat->tgLng, &stat->tgLat, &stat->tgLng);
            printf("Docking waypoint-> tgLat:%.8f tgLng:%.8f\r\n", stat->tgLat, stat->tgLng);
        }
        crumb(14);
        udpLog("DOCK 14 after waypoint adjust tg=%.6f,%.6f", stat->tgLat, stat->tgLng);
        stat->cmd = DOCKPOS;
        // INF, not SET - the same fix case LOCKING already carries, which was never applied here.
        // stat->IDr is BUOYIDALL at this point, so an ACK can never match in removeAckMsg() and a
        // SET sits in the LoRa retry table for all five retransmits. stat->ack is not reset below
        // either, so the RESET_SPEED_RUD_PID and DIRDIST sends that follow inherited it too: one
        // dock jammed three of the ten pendingMsg slots and put the buoy on the air five times
        // over for each. Nothing downstream reads ack for these commands.
        stat->ack = INF;
        xQueueSend(udpOut, (void *)stat, 0);
        xQueueSend(loraOut, (void *)stat, 10);

        crumb(15);
        udpLog("DOCK 15 broadcast done, about to route");
        if (stat->lat != 0.0 && stat->lng != 0.0 && stat->tgLat != 0.0 && stat->tgLng != 0.0) {
            RouteToPoint(stat->lat, stat->lng, stat->tgLat, stat->tgLng, &stat->tgDist, &stat->tgDir);
            crumb(16);
            udpLog("DOCK 16 routed dist=%.2f dir=%.2f", stat->tgDist, stat->tgDir);
        } else {
            if (stat->tgLat == 0.0) printf("WARNING: Dock position not set in memory!\r\n");
        }
        
        /* 
         * IMPORTANT DOCKING SAFETY FIX:
         * To prevent thrusters from reacting at full power on entering DOCKED mode,
         * we immediately override and reset the computed tgDist and tgDir to 0.0 during state transition.
         * This ensures safe, quiet operation at the dock until explicit command inputs are received.
         */
        stat->tgDist = 0.0;
        stat->tgDir = 0.0;
        
        stat->cmd = RESET_SPEED_RUD_PID;
        xQueueSend(serOut, (void *)stat, 0);  // send course and distance to sub
        xQueueSend(loraOut, (void *)stat, 0); // send course and distance to sub
        stat->cmd = DIRDIST;
        xQueueSend(serOut, (void *)stat, 0);  // send course and distance to sub
        xQueueSend(loraOut, (void *)stat, 0); // send course and distance to sub
        break;

    case COMPUTESTART:
        // Two buoys have to be locked before there is a line to square. Checked before the wind
        // test because it is the commoner mistake, and because an unlocked buoy's stale target
        // sends the whole fleet somewhere absurd. This one test covers the web page, the CYD and
        // the physical button alike - all three arrive here as status = COMPUTESTART.
        if (lockedBuoyCount(stat, buoyPara) < 2)
        {
            printf("#Start line NOT computed - need TWO locked buoys, have %d\r\n",
                   lockedBuoyCount(stat, buoyPara));
            beep(-1, buzzer);
            stat->status = (stat->tgLat != 0.0 && stat->tgLng != 0.0) ? LOCKED : IDLE;
            break;
        }
        // The start line is squared against THIS buoy's wind reading, so refuse when we do not
        // have one. A buoy with a failed compass reports wDir/wStd as 0/0, which is
        // indistinguishable from a real due-north calm - computing anyway silently lays the line
        // out against a fake northerly and sends both buoys to the wrong places.
        if (stat->wDir == 0.0 && stat->wStd == 0.0)
        {
            printf("#Start line NOT computed - this buoy has no wind reading (wDir/wStd are 0)\r\n");
            beep(-1, buzzer);
            // Hold whatever waypoint we already had, but never claim LOCKED without one:
            // tgLat/tgLng is 0 until something sets a real target, and a buoy sitting in LOCKED
            // on a zero target reports a bogus destination to the fleet and the display.
            stat->status = (stat->tgLat != 0.0 && stat->tgLng != 0.0) ? LOCKED : IDLE;
            break;
        }
        buoyPara[0].wDir = stat->wDir;
        calcTrackPos(buoyPara);
        for (int i = 0; i < 3; i++)
        {
            trackPosPrint(buoyPara[i].trackPos);
            printf(" = (%.12f,%.12f)\r\n", buoyPara[i].tgLat, buoyPara[i].tgLng);
        }
        // Trust the return value, not trackPos: calcTrackPos() above has already filled trackPos
        // in, so testing it here reported success even when recalcStartLine() had bailed out on
        // missing lock positions - complete with the confirmation beep and a SENDTRACK that
        // pushed uncomputed (often zero) positions to the other buoys.
        if (recalcStartLine(buoyPara))
        {
            beep(1, buzzer);
            adoptOwnTrackTarget(stat, buoyPara);
            stat->status = SENDTRACK;
            printf("#Send track info\r\n");
        }
        else
        {
            printf("#Start line NOT computed - lock all deployed buoys first\r\n");
            beep(-1, buzzer);
            // Hold whatever waypoint we already had, but never claim LOCKED without one:
            // tgLat/tgLng is 0 until something sets a real target, and a buoy sitting in LOCKED
            // on a zero target reports a bogus destination to the fleet and the display.
            stat->status = (stat->tgLat != 0.0 && stat->tgLng != 0.0) ? LOCKED : IDLE;
        }
        break;
    case COMPUTETRACK:
        // Same lock guard as COMPUTESTART, but a full course needs the third buoy for the upwind
        // mark - calcTrackPos() bails without it anyway, just later and less clearly.
        if (lockedBuoyCount(stat, buoyPara) < 3)
        {
            printf("#Track NOT computed - need THREE locked buoys, have %d\r\n",
                   lockedBuoyCount(stat, buoyPara));
            beep(-1, buzzer);
            stat->status = (stat->tgLat != 0.0 && stat->tgLng != 0.0) ? LOCKED : IDLE;
            break;
        }
        // Same wind guard as COMPUTESTART: the whole track is laid out relative to wDir.
        if (stat->wDir == 0.0 && stat->wStd == 0.0)
        {
            printf("#Track NOT computed - this buoy has no wind reading (wDir/wStd are 0)\r\n");
            beep(-1, buzzer);
            // Hold whatever waypoint we already had, but never claim LOCKED without one:
            // tgLat/tgLng is 0 until something sets a real target, and a buoy sitting in LOCKED
            // on a zero target reports a bogus destination to the fleet and the display.
            stat->status = (stat->tgLat != 0.0 && stat->tgLng != 0.0) ? LOCKED : IDLE;
            break;
        }
        buoyPara[0].wDir = stat->wDir;
        for (int i = 0; i < 3; i++)
        {
            trackPosPrint(buoyPara[i].trackPos);
            printf(" = (%.12f,%.12f)\r\n", buoyPara[i].tgLat, buoyPara[i].tgLng);
        }
        // Same fix as COMPUTESTART. The old test was even weaker here: it ORed the three
        // trackPos values, so one stale entry left over from a previous computation was enough
        // to report success after reCalcTrack() had bailed out.
        if (reCalcTrack(buoyPara))
        {
            beep(1, buzzer);
            adoptOwnTrackTarget(stat, buoyPara);
            stat->status = SENDTRACK;
        }
        else
        {
            printf("#Track NOT computed - needs three buoys with a lock position\r\n");
            beep(-1, buzzer);
            // Hold whatever waypoint we already had, but never claim LOCKED without one:
            // tgLat/tgLng is 0 until something sets a real target, and a buoy sitting in LOCKED
            // on a zero target reports a bogus destination to the fleet and the display.
            stat->status = (stat->tgLat != 0.0 && stat->tgLng != 0.0) ? LOCKED : IDLE;
        }
        break;
    case SENDTRACK:
        // Start at 1: slot 0 is us, and our own end of the line was already taken by
        // adoptOwnTrackTarget() in COMPUTESTART/COMPUTETRACK. The old loop started at 0 and tried
        // to recognise us by "IDs != stat->buoyId" - buoyId is always 0, so that test never
        // excluded anything, and we broadcast a SETLOCKPOS addressed to our own Sub's MAC. Our own
        // is_for_me test in handleRfData() matches that ID, so the packet came back in over UDP and
        // re-set the target we had just computed.
        for (int i = 1; i < 3; i++)
        {
            if (buoyPara[i].trackPos != 0 && buoyPara[i].IDs != 0)
            {
                memcpy(&LoraTx, &buoyPara[i], sizeof(RoboStruct));
                trackPosPrint(buoyPara[i].trackPos);
                printf("n = (%.12f,%.12f)\r\n", buoyPara[i].tgLat, buoyPara[i].tgLng);
                LoraTx.IDr = buoyPara[i].IDs;
                LoraTx.IDs = stat->mac;
                LoraTx.cmd = SETLOCKPOS;
                LoraTx.ack = GETACK;
                xQueueSend(loraOut, (void *)&LoraTx, 10); // send out through Lora
                xQueueSend(udpOut, (void *)&LoraTx, 10);  // send out through WiFi
            }
        }
        stat->status = LOCKED;
        break;
    case START_CALIBRATE_MAGNETIC_COMPASS:
        LoraTx.cmd = CALIBRATE_MAGNETIC_COMPASS;
        LoraTx.ack = ACK;
        xQueueSend(serOut, (void *)&LoraTx, 10); // send out through Lora
        stat->status = CALIBRATE_MAGNETIC_COMPASS;
        break;
    case CALC_COMPASS_OFFSET:
        LoraTx.cmd = CALC_COMPASS_OFFSET;
        LoraTx.ack = ACK;
        xQueueSend(serOut, (void *)&LoraTx, 10); // send out through Lora
        stat->status = IDLING;
        break;
    case STOREASDOC:
        if (stat->gpsFix == true)
        {
            printf("Storing docpositoin\r\n");
            stat->tgLat = stat->lat;
            stat->tgLng = stat->lng;
            memDockPos(stat, MEM_PUT);
            beep(1000, buzzer);
        }
        else
        {
            beep(-1, buzzer);
        }
        stat->status = IDLING;
        break;
    default:
        break;
    }
}

//***************************************************************************************************
//  In-Field Compass Calibration State Machine
//***************************************************************************************************
/**
 * @brief Manages the in-field compass calibration state machine.
 * 
 * @param timer Pointer to the RoboStruct containing calibration state and data.
 */
void handleInfieldCompassCalibration(RoboStruct *timer)
{
    static unsigned long calibStartTime = 0;
    static int calibPhase = 0;
    static double lat0, lon0;

    if (timer->status != INFIELD_CALIBRATE)
    {
        calibPhase = 0;
        return;
    }

    if (calibPhase == 0)
    {
        if (timer->gpsFix)
        {
            lat0 = timer->lat;
            lon0 = timer->lng;
            calibStartTime = millis();
            calibPhase = 1;
            printf("#INFIELD_COMPASS: Phase 0 (Home recorded: %f, %f)\r\n", lat0, lon0);
            
            // Trigger sub to do the spins
            RoboStruct cmdMsg;
            cmdMsg.cmd = INFIELD_CALIBRATE;
            cmdMsg.IDs = timer->mac;
            cmdMsg.ack = INF;
            xQueueSend(serOut, (void *)&cmdMsg, 0);
        }
        else
        {
            printf("#INFIELD_COMPASS: Waiting for GPS fix...\r\n");
            delay(1000);
            return;
        }
    }
    else if (calibPhase == 1)
    {
        // Waiting for sub to finish (takes ~3 mins). Sub will send IDLE command when done.
        // We catch IDLE in handleRfData / handleSerialData which resets timer->status to IDLING/IDLE.
        // Wait, if timer->status becomes IDLE, this function exits!
        // To catch the end, we need to know it finished.
        // We'll just wait for the timeout (~190s) here as a backup if IDLE isn't caught,
        // or actually, if we let `handleSerialData` change `status` to `IDLE`, this state machine breaks before returning home!
        // We should intercept `IDLE` inside handleSerialData, or better, we just check if elapsed time > 160000ms
        
        unsigned long elapsed = millis() - calibStartTime;
        if (elapsed >= 165000) // 165 seconds = 2 mins 45 seconds (3 phases = 60+60+30 + 15 buffer)
        {
            printf("#INFIELD_COMPASS: Calibration time complete. Returning to Home (P0).\r\n");
            timer->tgLat = lat0;
            timer->tgLng = lon0;
            timer->status = LOCKED;
            calibPhase = 0;
            beep(5, buzzer);
        }
    }
}

//***************************************************************************************************
//  In-Field Offset Calibration State Machine
//***************************************************************************************************
/**
 * @brief Manages the in-field compass offset calibration state machine.
 * 
 * @param timer Pointer to the RoboStruct containing calibration state and data.
 */
void handleInfieldOffsetCalibration(RoboStruct *timer)
{
    static unsigned long calibStartTime = 0;
    static int calibPhase = 0;
    static double lat0, lon0, lat1, lon1, lat2, lon2, lat2_stable, lon2_stable, lat3, lon3;
    static unsigned long lastUpdate = 0;

    if (timer->status != INFIELD_OFFSET_CALIBRATE)
    {
        calibPhase = 0;
        return;
    }

    if (calibPhase == 0)
    {
        if (timer->gpsFix)
        {
            lat0 = timer->lat;
            lon0 = timer->lng;
            calibStartTime = millis();
            calibPhase = 1;
            printf("#INFIELD_OFFSET: Phase 0 (Home recorded: %f, %f)\r\n", lat0, lon0);
        }
        else
        {
            printf("#INFIELD_OFFSET: Waiting for GPS fix...\r\n");
            delay(1000);
            return;
        }
    }

    unsigned long elapsed = millis() - calibStartTime;

    if (millis() - lastUpdate > 500)
    {
        lastUpdate = millis();
        RoboStruct cmdMsg;
        cmdMsg.cmd = TGDIRSPEED;
        cmdMsg.IDs = timer->mac;
        cmdMsg.ack = INF;

        if (elapsed < 10000)
        {
            // Phase 0: Stabilization (Sail South for 10s before recording start point)
            cmdMsg.tgDir = 180.0;
            cmdMsg.speedSet = 50;
            xQueueSend(serOut, (void *)&cmdMsg, 0);
        }
        else if (calibPhase == 1 && elapsed >= 10000)
        {
            // Phase 1: Record Start Point P1 (after 10s of sailing South)
            lat1 = timer->lat;
            lon1 = timer->lng;
            calibPhase = 2;
            printf("#INFIELD_OFFSET: Phase 1 (P1 recorded: %f, %f)\r\n", lat1, lon1);
        }
        else if (calibPhase == 2 && elapsed < 130000)
        {
            // Phase 2: Calibration Leg
            cmdMsg.tgDir = 180.0;
            cmdMsg.speedSet = 50;
            xQueueSend(serOut, (void *)&cmdMsg, 0);
        }
        else if (calibPhase == 2 && elapsed >= 130000)
        {
            // Phase 3: Record End Point P2 and Calculate Offset
            lat2 = timer->lat;
            lon2 = timer->lng;
            
            double gpsCourse1 = calculateAngle(lat1, lon1, lat2, lon2);
            double newOffset = gpsCourse1 - 180.0;
            
            while (newOffset > 180.0) newOffset -= 360.0;
            while (newOffset < -180.0) newOffset += 360.0;

            timer->compassOffset += newOffset;
            
            printf("#INFIELD_OFFSET: Phase 3 (P2 recorded: %f, %f). GPS Course: %.2f. New Offset: %.2f\r\n", lat2, lon2, gpsCourse1, newOffset);
            
            // Send new offset to Sub
            RoboStruct offsetMsg;
            offsetMsg.cmd = STORE_COMPASS_OFFSET;
            offsetMsg.IDs = timer->mac;
            offsetMsg.ack = INF;
            offsetMsg.compassOffset = timer->compassOffset;
            xQueueSend(serOut, (void *)&offsetMsg, 0);
            
            calibPhase = 4;
        }
        // else if (calibPhase == 4 && elapsed < 140000)
        // {
        //     // Phase 4: Return Leg Stabilization (wait for buoy to turn around)
        //     cmdMsg.tgDir = 0.0;
        //     cmdMsg.speedSet = 50;
        //     xQueueSend(serOut, (void *)&cmdMsg, 0);
        // }
        // else if (calibPhase == 4 && elapsed >= 140000)
        // {
        //     // Phase 5: Record Stable Return Start Point
        //     lat2_stable = timer->lat;
        //     lon2_stable = timer->lng;
        //     calibPhase = 6;
        //     printf("#INFIELD_OFFSET: Phase 5 (P2_stable recorded for return leg: %f, %f)\r\n", lat2_stable, lon2_stable);
        // }
        // else if (calibPhase == 6 && elapsed < 260000)
        // {
        //     // Phase 6: Return Leg Sailing
        //     cmdMsg.tgDir = 0.0;
        //     cmdMsg.speedSet = 50;
        //     xQueueSend(serOut, (void *)&cmdMsg, 0);
        // }
        else if (calibPhase == 4)
        {
            // // Phase 7: Record Final Point P3 and Validate
            // lat3 = timer->lat;
            // lon3 = timer->lng;
            
            // double gpsCourse2 = calculateAngle(lat2_stable, lon2_stable, lat3, lon3);
            // double validationError = abs(gpsCourse2 - 0.0);
            // while (validationError > 180.0) validationError -= 360.0;
            
            // printf("#INFIELD_OFFSET: Phase 7 (P3 recorded). GPS Course: %.2f. Error: %.2f\r\n", gpsCourse2, abs(validationError));
            
            // // Stop motors
            // cmdMsg.tgDir = 0.0;
            // cmdMsg.speedSet = 0;
            // xQueueSend(serOut, (void *)&cmdMsg, 0);
            
            // Phase 8: Return Home
            timer->tgLat = lat0;
            timer->tgLng = lon0;
            timer->status = LOCKED;
            calibPhase = 0;
            printf("#INFIELD_OFFSET: Complete. Returning to Home (P0).\r\n");
            beep(5, buzzer);
        }
    }
}

//***************************************************************************************************
//  Timer routines
//***************************************************************************************************
/**
 * @brief Executes periodic tasks and handles timed data transmissions.
 * 
 * @param timer Pointer to the RoboStruct containing the buoy's data and timers.
 */
void handleTimerRoutines(RoboStruct *timer)
{
    handleInfieldCompassCalibration(timer);
    handleInfieldOffsetCalibration(timer);
    handleGpsFourierCalibration(timer);

    timer->IDs = timer->mac;
    timer->IDr = BUOYIDALL;
    timer->ack = INF;

    static int lastStatus = -1;
    bool statusChanged = false;
    if ((timer->status == LOCKED || timer->status == DOCKED) &&
        (lastStatus != LOCKED && lastStatus != DOCKED))
    {
        statusChanged = true;
    }
    lastStatus = timer->status;

    //***************************************************************************************************
    // sub data out
    //***************************************************************************************************
    if (timer->lastSerOut < millis())
    {
        timer->lastSerOut = millis() + 5000;
        if (timer->status == LOCKED || timer->status == DOCKED)
        {
            timer->lastSerOut = millis() + 250;
            if (statusChanged) 
            {
                timer->tgDist = 0.0;
            } 
            else if (timer->lat != 0.0 && timer->lng != 0.0 && timer->tgLat != 0.0 && timer->tgLng != 0.0) 
            {
                RouteToPoint(timer->lat, timer->lng, timer->tgLat, timer->tgLng, &timer->tgDist, &timer->tgDir);
                // Second leg of a two-stage dock approach: once the offset waypoint is reached,
                // switch the target to the dock itself. DOCKED only - this branch is shared with
                // LOCKED, and there it was silently throwing the lock away. Locking puts the buoy
                // on its own position, so tgDist is ~0 and this fired within a few hundred ms,
                // replacing the just-captured GPS fix with the stored dock position and sending
                // the buoy off towards it. dockingToWaypoint is reloaded from NVS as true on
                // every boot, so it struck the first lock after each restart and then appeared
                // to cure itself for the rest of the power cycle.
                if (timer->status == DOCKED && timer->dockingToWaypoint == true && timer->tgDist < 5)
                {
                    memDockPos(timer, MEM_GET);
                    timer->dockingToWaypoint = false;
                }
            } else {
                timer->tgDist = 0;
                timer->tgDir = 0;
            }
            if (timer->tgDist > 10000)
            {
                distErrorCnt++;
                if (distErrorCnt > 100)
                {
                    distErrorCnt = 0;
                    timer->status = IDLING;
                }
                return;
            }
            distErrorCnt = 0;
            timer->cmd = DIRDIST;
            xQueueSend(serOut, (void *)timer, 0); // send course and distance to sub
            timer->cmd = DIRMDIRTGDIRG;
            xQueueSend(udpOut, (void *)timer, 0); // send course and distance to udp

            // Periodically re-announce our waypoint while holding station.
            // LOCKPOS/DOCKPOS is otherwise sent exactly once, on the state transition, and it is
            // the only message type carrying tgLat/tgLng - the recurring BUOYPOS/TOPDATA
            // telemetry does not. So a single lost packet left the other buoys and the CYD
            // without our target until the next manual lock, and their field view drew nothing.
            // ack = INF keeps this out of the LoRa ACK retry table: it is a beacon, and the next
            // one is only 5 s away, so retransmitting each copy 5x would just add channel load.
            static unsigned long nextWaypointBeacon = 0;
            if (timer->tgLat != 0.0 && timer->tgLng != 0.0 && millis() > nextWaypointBeacon)
            {
                nextWaypointBeacon = millis() + 5000;
                unsigned long savedIDr = timer->IDr;
                int savedAck = timer->ack;
                timer->IDr = BUOYIDALL;
                timer->cmd = (timer->status == DOCKED) ? DOCKPOS : LOCKPOS;
                timer->ack = INF;
                xQueueSend(udpOut, (void *)timer, 0);
                xQueueSend(loraOut, (void *)timer, 0);
                timer->IDr = savedIDr;
                timer->ack = savedAck;
            }
        }
        else if (timer->status == REMOTE) // Remote controlled
        {
            timer->lastSerOut = millis() + 500;
            timer->cmd = REMOTE;
            xQueueSend(serOut, (void *)timer, 0);
            timer->cmd = TGDIRSPEED;
            xQueueSend(udpOut, (void *)timer, 10); // send out through wifi
        }
    }
    //********************************************************************************//
    // RF and WiFi/UDP Transmit Rates (Slowing down LoRa specifically to 5-second intervals)
    //********************************************************************************//
    static unsigned long lastLoraTx = 0;
    static unsigned long lastUdpTx = 0;

    // UDP/WiFi transmission (fast: every 250ms, increased to 100ms during manual calibration for smooth real-time updates)
    unsigned long udpInterval = 250;
    if (timer->status == REMOTE)
    {
        udpInterval = 100; // Ultra-fast 10Hz UDP updates during calibration!
    }

    if (lastUdpTx + udpInterval < millis())
    {
        lastUdpTx = millis();
        if ((timer->status == LOCKED || timer->status == DOCKED))
        {
            timer->cmd = TOPDATA;
            xQueueSend(udpOut, (void *)timer, 10);  // send out through wifi
        }
        else if (timer->status == REMOTE)
        {
            timer->cmd = TOPDATA; // Send standard TOPDATA (51) so the CYD can parse it at 10Hz!
            xQueueSend(udpOut, (void *)timer, 10);  // send out through wifi
        }
        else
        {
            timer->cmd = BUOYPOS;
            xQueueSend(udpOut, (void *)timer, 10);  // send out through wifi
            timer->cmd = TOPDATA;
            xQueueSend(udpOut, (void *)timer, 10);  // send out through wifi
        }
    }

    // LoRa transmission (Dynamic rate: 250ms during manual calibration, 1000ms when active, 5000ms when idle to conserve battery)
    unsigned long loraInterval = 5000;
    if (timer->status == REMOTE)
    {
        loraInterval = 250; // Ultra-fast 4Hz LoRa updates during manual calibration!
    }
    else if (timer->status != IDLE && timer->status != IDLING)
    {
        loraInterval = 1000;
    }

    if (lastLoraTx + loraInterval < millis())
    {
        lastLoraTx = millis() + random(0, 100); // Add small jitter to prevent packets colliding
        if ((timer->status == LOCKED || timer->status == DOCKED))
        {
            timer->cmd = TOPDATA;
            xQueueSend(loraOut, (void *)timer, 10); // send out through Lora
        }
        else if (timer->status == REMOTE)
        {
            timer->cmd = TOPDATA; // Send standard TOPDATA (51) so the CYD can parse it at 4Hz!
            xQueueSend(loraOut, (void *)timer, 10); // send out through Lora
        }
        else
        {
            timer->cmd = BUOYPOS;
            xQueueSend(loraOut, (void *)timer, 10); // send out through Lora
            timer->cmd = TOPDATA;
            xQueueSend(loraOut, (void *)timer, 10); // send out through Lora
        }
    }

    if (logtimer < millis())
    {
        logtimer = millis() + 5000;
        battVoltage(timer->topAccuV, timer->topAccuP);
        addNewSampleInBuffer(&wind, timer->dirMag); // add sample to buffer for wind direction calculation.
        deviationWindRose(&wind);
        timer->wStd = wind.wStd;
        timer->wDir = wind.wDir; // averige wind dir
        timer->cmd = TOPDATA;
        xQueueSend(udpOut, (void *)timer, 10); // send out through wifi
    }
}

// ***************************************************************************************************
//  Command Hash Generator (FNV-1a 32-bit Hash)
// ***************************************************************************************************
/**
 * @brief Computes a lightweight 32-bit FNV-1a hash of the core control and parameter fields 
 *        of an incoming RoboStruct telemetry/command structure.
 * 
 * DESIGN RATIONALE:
 * Inside the main execution thread, raw command strings (e.g. from LoRa or UDP WiFi packets)
 * have already been parsed and discarded by background listener tasks before being pushed 
 * into binary queues (`loraIn` and `udpIn`). Therefore, we cannot compare the raw string 
 * or its direct CRC checksum in this routine.
 * 
 * To implement highly fast and memory-efficient duplicate detection without storing bulky 
 * 500-byte structures in a history cache, we serialize only the active control and 
 * parameter-bearing fields of the struct and feed them into a 32-bit FNV-1a non-cryptographic hash.
 * 
 * FIELDS INCLUDED IN THE HASH:
 * - `cmd`: The command identifier (identifies the operation)
 * - `IDs`: The source/sender identifier (identifies who sent it)
 * - `IDr`: The recipient/destination identifier (identifies who should execute it)
 * - `status`: The target physical state/status of the buoy
 * - `ack`: The acknowledgment behavior/flag (e.g. SET, GET, ACK)
 * - `tgLat` / `tgLng`: The destination GPS coordinates for navigation target updates
 * - `tgDir` / `tgSpeed` / `tgDist`: The direct steering control parameters
 * 
 * FIELDS EXCLUDED (To avoid false mismatches on identical commands):
 * - `loralstmsg`: Excluded because it holds the real-time RSSI signal strength (which varies).
 * - `lastLoraIn`: Excluded because it holds the receipt timestamp (which changes on every packet).
 * 
 * @param msg The incoming decoded command structure to hash.
 * @return uint32_t A highly unique 32-bit hash representing the semantic payload of the command.
 */
static uint32_t calculateCommandHash(const RoboStruct &msg)
{
    // FNV-1a 32-bit offset basis and prime constants
    uint32_t hash = 2166136261U;
    const uint32_t prime = 16777619U;

    // Helper lambda to sequentially feed byte arrays into the FNV-1a multiplier step
    auto hashBytes = [&](const uint8_t* bytes, size_t len) {
        for (size_t i = 0; i < len; i++) {
            hash ^= bytes[i];
            hash *= prime;
        }
    };

    // Feed the core operational identity fields
    hashBytes((const uint8_t*)&msg.cmd, sizeof(msg.cmd));
    hashBytes((const uint8_t*)&msg.IDs, sizeof(msg.IDs));
    hashBytes((const uint8_t*)&msg.IDr, sizeof(msg.IDr));
    hashBytes((const uint8_t*)&msg.status, sizeof(msg.status));
    hashBytes((const uint8_t*)&msg.ack, sizeof(msg.ack));

    // Feed the target geographical coordinates (critical for waypoint setting deduplication)
    hashBytes((const uint8_t*)&msg.tgLat, sizeof(msg.tgLat));
    hashBytes((const uint8_t*)&msg.tgLng, sizeof(msg.tgLng));

    // Feed direct navigation metrics (critical for steering commands)
    hashBytes((const uint8_t*)&msg.tgDir, sizeof(msg.tgDir));
    hashBytes((const uint8_t*)&msg.tgSpeed, sizeof(msg.tgSpeed));
    hashBytes((const uint8_t*)&msg.tgDist, sizeof(msg.tgDist));

    return hash;
}

// ***************************************************************************************************
// handle external data input
// ***************************************************************************************************
/**
 * @brief Processes incoming data from LoRa or UDP (WiFi).
 * 
 * @param RfOut Pointer to the RoboStruct to be updated with incoming data.
 * @param buoyPara Array of pointers to RoboStructs for all buoys.
 */
void handleRfData(RoboStruct *RfOut, RoboStruct *buoyPara[3])
{
    RoboStruct RfIn;
    RfIn.IDr = -1;
    bool from_udp = false;

    if (xQueueReceive(loraIn, (void *)&RfIn, 1) == pdTRUE) // new lora data
    {
        // printf("handleRfData: Received from LoRa\r\n");
    }
    else if (xQueueReceive(udpIn, (void *)&RfIn, 1) == pdTRUE) // new udp data
    {
        from_udp = true;
    }

    if (RfIn.IDr != -1)
    {
        crumb(1);
        if (!noisyCmd(RfIn.cmd))
        {
            udpLog("RF in  cmd=%d ack=%d st=%d IDr=%08lX IDs=%08lX via=%s",
                   RfIn.cmd, RfIn.ack, RfIn.status, (unsigned long)RfIn.IDr,
                   (unsigned long)RfIn.IDs, from_udp ? "udp" : "lora");
        }

        // ===========================================================================================
        //  5-Second Control/State Command Deduplication Filter
        // ===========================================================================================
        // To prevent duplicate command execution, we cache the 32-bit FNV-1a hashes of recently 
        // processed commands. When a command arrives via LoRa and UDP simultaneously, the second 
        // transmission will be silently discarded if its contents are identical.
        // ===========================================================================================
        #define CMD_HISTORY_SIZE 32
        #define CMD_DUP_TIMEOUT_MS 5000UL // 5-second duplicate suppression window

        struct CommandHistoryEntry {
            uint32_t hash = 0;           // 32-bit FNV-1a hash of the command fields
            unsigned long timestamp = 0; // The millisecond timestamp when the command was processed
        };

        static CommandHistoryEntry cmdHistory[CMD_HISTORY_SIZE];
        static int cmdHistoryIndex = 0;

        // CRITICAL FILTERING RULE:
        // We only apply duplicate detection to active control or state-changing commands.
        // We explicitly skip telemetry packets (BUOYPOS, TOPDATA, SUBDATA, SUBACCU, SUBPWR). 
        // Telemetry must bypass deduplication so that they always update the live database 
        // with "last seen" timestamps and current metrics, even if the values have not changed.
        if (RfIn.cmd != BUOYPOS && RfIn.cmd != TOPDATA && RfIn.cmd != SUBDATA && RfIn.cmd != SUBACCU && RfIn.cmd != SUBPWR && RfIn.ack != GET && RfIn.ack != GETACK)
        {
            uint32_t currentHash = calculateCommandHash(RfIn);
            bool isDuplicate = false;
            unsigned long now = millis();

            // Search the history cache for a matching command hash within the 5-second window
            for (int i = 0; i < CMD_HISTORY_SIZE; i++)
            {
                if (cmdHistory[i].hash == currentHash)
                {
                    // Check if the match is within our 5-second tracking timeout
                    if (now - cmdHistory[i].timestamp < CMD_DUP_TIMEOUT_MS)
                    {
                        isDuplicate = true;
                        break;
                    }
                }
            }

            if (isDuplicate)
            {
                // Silent return to discard the redundant duplicate command
                return;
            }

            // Save the unique command hash and timestamp in the circular history buffer
            cmdHistory[cmdHistoryIndex].hash = currentHash;
            cmdHistory[cmdHistoryIndex].timestamp = now;
            
            // Advance the circular buffer index
            cmdHistoryIndex = (cmdHistoryIndex + 1) % CMD_HISTORY_SIZE;
        }

        // Capture telemetry data from other buoys regardless of target ID before bridging
        if (RfIn.cmd == BUOYPOS || RfIn.cmd == TOPDATA || RfIn.cmd == SUBDATA || RfIn.cmd == SUBACCU || RfIn.cmd == SUBPWR)
        {
            if (RfIn.IDs != 0 && RfIn.IDs != RfOut->IDs && RfIn.IDs != RfOut->mac)
            {
                AddDataToBuoyBase(RfIn, buoyPara);
            }
        }

        // --- BRIDGING LOGIC ---
        // If the message is NOT for us and NOT a broadcast, bridge it to the other interface
        // treat IDr == 0 as a legacy broadcast
        // A broadcast is addressed to every buoy, so it must be executed locally as well as
        // forwarded. Whether a broadcast is actually allowed to change our state is decided
        // per command further down, where the sender must be the web/remote (0x98/0x99).
        bool is_broadcast = (RfIn.IDr == BUOYIDALL || RfIn.IDr == 0);
        bool is_for_me = (RfIn.IDr == RfOut->mac || RfIn.IDr == RfOut->IDs || is_broadcast);
        crumb(2);
        if (!noisyCmd(RfIn.cmd))
        {
            udpLog("RF route bcast=%d forme=%d mymac=%08lX myids=%08lX",
                   (int)is_broadcast, (int)is_for_me, (unsigned long)RfOut->mac,
                   (unsigned long)RfOut->IDs);
        }

        if (!is_for_me)
        {
            // Always bridge to the interface the packet did NOT arrive on. Sending it back out
            // on the arrival interface re-broadcasts it with the original sender ID, which the
            // IDs != espMac() self-filter in udp_setup cannot catch -> endless rebroadcast loop.
            if (from_udp) {
                xQueueSend(loraOut, (void *)&RfIn, 0);
            } else {
                xQueueSend(udpOut, (void *)&RfIn, 0);
            }
            return; // Done with bridging
        }
        else if (from_udp && is_broadcast && (RfIn.IDs == 0x99 || RfIn.IDs == 0x98))
        {
            // Broadcast COMMAND arrived over UDP/WiFi: handle it locally (below) and forward a
            // copy to LoRa so buoys without a WiFi link receive it too.
            //
            // Restricted to the web (0x99) and the display (0x98), the same authority test the
            // command cases below use. Without it this relayed every peer's telemetry as well:
            // the other Top broadcasts BUOYPOS and TOPDATA over UDP every 250 ms, and we put all
            // eight frames a second back on the air under ITS sender id. Measured on the bench,
            // one idle buoy appeared to be transmitting ~3 LoRa frames/s - roughly 40% channel
            // occupancy at the library default SF7/125 kHz - when its own timer only sends two
            // frames every 5 s. The relayed copies are pure waste: every buoy already broadcasts
            // its own telemetry on LoRa, so nobody needs us to repeat it, and the collisions it
            // caused were losing the packets that mattered.
            xQueueSend(loraOut, (void *)&RfIn, 0);
        }

        // --- LOCAL HANDLING (For this buoy or ALL) ---
        if (is_for_me)
        {
            crumb(1000 + RfIn.cmd);
            switch (RfIn.cmd)
            {
            case BUOYPOS:
            case TOPDATA:
                if (RfIn.IDs != 0 && RfIn.IDs != RfOut->IDs && RfIn.IDs != RfOut->mac)
                {
                    AddDataToBuoyBase(RfIn, buoyPara);
                }
                break;
            case SUBDATA:
                if (RfIn.IDs != 0 && RfIn.IDs != RfOut->IDs && RfIn.IDs != RfOut->mac)
                {
                    AddDataToBuoyBase(RfIn, buoyPara);
                }
                else
                {
                    RfOut->dirMag = RfIn.dirMag;
                    RfOut->speedBb = RfIn.speedBb;
                    RfOut->speedSb = RfIn.speedSb;
                    RfOut->ip = RfIn.ip;
                    RfOut->ir = RfIn.ir;
                    RfOut->subAccuV = RfIn.subAccuV;
                    RfOut->subAccuP = RfIn.subAccuP;
                    RfOut->subAccuI = RfIn.subAccuI;

                    mainPwrData.ledBb = RfOut->speedBb;
                    mainPwrData.ledSb = RfOut->speedSb;
                    xQueueSend(ledPwr, (void *)&mainPwrData, 0);
                }
                break;
            case DOCKING:
                if (RfIn.IDr == RfOut->mac || ((RfIn.IDr == BUOYIDALL || RfIn.IDr == 0) && (RfIn.IDs == 0x99 || RfIn.IDs == 0x98)))
                {
                    if (RfOut->status != DOCKING && RfOut->status != DOCKED)
                    {
                        printf("#Status set to DOCKING\r\n");
                        RfOut->status = DOCKING;
                        RfOut->lastSerOut = 0; // Force immediate update to sub
                    }
                }
                else
                {
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;

            case COMPUTESTART:
                if (RfIn.IDr == RfOut->mac || ((RfIn.IDr == BUOYIDALL || RfIn.IDr == 0) && (RfIn.IDs == 0x99 || RfIn.IDs == 0x98)))
                {
                    printf("#Status set to COMPUTESTART\r\n");
                    RfOut->status = COMPUTESTART;
                    RfOut->lastSerOut = 0; // Force immediate update to sub
                }
                else
                {
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;

            case COMPUTETRACK:
                if (RfIn.IDr == RfOut->mac || ((RfIn.IDr == BUOYIDALL || RfIn.IDr == 0) && (RfIn.IDs == 0x99 || RfIn.IDs == 0x98)))
                {
                    printf("#Status set to COMPUTETRACK\r\n");
                    RfOut->status = COMPUTETRACK;
                    RfOut->lastSerOut = 0; // Force immediate update to sub
                }
                else
                {
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;
            case LOCKPOS: // store new data into position database
                AddDataToBuoyBase(RfIn, buoyParaPtrs);
                break;
            case INFIELD_CALIBRATE:
            case INFIELD_OFFSET_CALIBRATE:
                if (RfIn.ack == GET || RfIn.ack == GETACK || RfIn.ack == SET)
                {
                    xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                }
                else
                {
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;
            case PIDRUDDER:
                if (RfIn.ack == GET || RfIn.ack == GETACK)
                {
                    RfIn.IDr = BUOYIDALL;
                    xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                }
                else
                {
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;
            case PIDRUDDERSET:
                printf("#PIDRUDDERSET: %05.2f %05.2f %05.2f\r\n", RfIn.Kpr, RfIn.Kir, RfIn.Kdr);
                RfIn.ack = SET; // Tell Sub to save to EEPROM
                RfIn.IDr = BUOYIDALL;
                if (xQueueSend(serOut, (void *)&RfIn, pdMS_TO_TICKS(250)) != pdTRUE) {
                    printf("ERROR: Failed to queue PIDRUDDERSET to serOut!\r\n");
                }
                break;
            case PIDSPEED:
                if (RfIn.ack == GET || RfIn.ack == GETACK)
                {
                    RfIn.IDr = BUOYIDALL;
                    xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                }
                else
                {
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;
            case SETUPDATA:
            {
                printf("Received SETUPDATA command. ack=%d, IDs=0x%08lX, dockDist=%d, dockDir=%d, dockWP=%s\r\n",
                       RfIn.ack, RfIn.IDs, RfIn.dockApproachDist, RfIn.dockApproachDir, RfIn.dockingToWaypoint ? "YES" : "NO");
                // Check if the command/response is local to the master or if it originates from a remote buoy.
                // Without this gate a SETUPDATA relayed over LoRa by another buoy would overwrite our own
                // PID / compass / thruster configuration with that buoy's values.
                bool is_local = (RfIn.IDs == RfOut->mac || RfIn.IDs == 0x98 || RfIn.IDs == 0x99 || from_udp);
                if (is_local && (RfIn.ack == 1 || RfIn.ack == 3 || RfIn.ack == 2)) // 1=GET, 2=SET, 3=GETACK
                {
                    
                    if (RfIn.ack == 2 || RfIn.ack == 3) { // 2=SET, 3=GETACK
                        // Deliberately not written to our own flash: every value below belongs
                        // to the Sub, which commits it when we forward this frame with ack = SET.
                        // We keep a RAM copy only so the web page and the CYD have something to
                        // show until the Sub reports back.
                        RfOut->Kpr = RfIn.Kpr; RfOut->Kir = RfIn.Kir; RfOut->Kdr = RfIn.Kdr;
                        RfOut->Kps = RfIn.Kps; RfOut->Kis = RfIn.Kis; RfOut->Kds = RfIn.Kds;
                        RfOut->maxSpeed = RfIn.maxSpeed; RfOut->minSpeed = RfIn.minSpeed;
                        RfOut->pivotSpeed = RfIn.pivotSpeed;
                        RfOut->compassOffset = RfIn.compassOffset;
                        RfOut->holdRad = RfIn.holdRad;
                        RfOut->revBB = RfIn.revBB;
                        RfOut->revSB = RfIn.revSB;
                        RfOut->swap_BB_SB = RfIn.swap_BB_SB;
                        // SETUPDATA field 16. The Sub owns the trim and re-announces it every
                        // second over ADAPTIVE_TRIM, but taking it here means the Setup page shows
                        // the new state at once instead of on the next broadcast.
                        RfOut->compass_trim_enabled = RfIn.compass_trim_enabled;
                        RfOut->interpEnabled = RfIn.interpEnabled;
                        if (RfIn.IDs == 0x98 || RfIn.IDs == 0x99) {
                            RfOut->dockApproachDist = RfIn.dockApproachDist;
                            RfOut->dockApproachDir = RfIn.dockApproachDir;
                            RfOut->dockingToWaypoint = RfIn.dockingToWaypoint;
                            memDockApproach(RfOut, MEM_PUT);
                        }
                        
                        // Never commit an all-zero PID block to the Sub.
                        //
                        // We hold these in RAM only, filled from the Sub's SETUPDATA reply. When
                        // the Sub has not answered - a fresh reboot, or the one-wire serial dead
                        // in the Top->Sub direction - every field here still reads 0, the web page
                        // and the CYD show zeros, and pressing SAVE forwarded those zeros with
                        // ack = SET for the Sub to write over a good calibration. No buoy can run
                        // on an all-zero rudder AND speed PID, so this cannot reject a real save.
                        if (RfIn.Kpr == 0.0 && RfIn.Kir == 0.0 && RfIn.Kdr == 0.0 &&
                            RfIn.Kps == 0.0 && RfIn.Kis == 0.0 && RfIn.Kds == 0.0)
                        {
                            // Dropping the whole frame was too blunt. The harmonic correction
                            // switch travels in THIS frame, so a sender with a stale cache could
                            // not turn it off - and MAN CAL, which must have it off before it
                            // captures anything, would calibrate against a corrected compass.
                            //
                            // Patch the PIDs from our own mirror instead and let the rest through:
                            // the flags still take effect and nothing good is overwritten. Only
                            // when we have no good values either is there nothing to do but refuse.
                            if (RfOut->Kpr != 0.0 || RfOut->Kir != 0.0 || RfOut->Kdr != 0.0 ||
                                RfOut->Kps != 0.0 || RfOut->Kis != 0.0 || RfOut->Kds != 0.0)
                            {
                                printf("#SETUPDATA: all-zero PID block from a stale sender - "
                                       "substituting ours and forwarding the rest\r\n");
                                RfIn.Kpr = RfOut->Kpr; RfIn.Kir = RfOut->Kir; RfIn.Kdr = RfOut->Kdr;
                                RfIn.Kps = RfOut->Kps; RfIn.Kis = RfOut->Kis; RfIn.Kds = RfOut->Kds;
                                if (RfIn.maxSpeed == 0 && RfIn.minSpeed == 0) {
                                    RfIn.maxSpeed = RfOut->maxSpeed;
                                    RfIn.minSpeed = RfOut->minSpeed;
                                }
                            }
                            else
                            {
                                printf("#SETUPDATA REFUSED - all-zero PID block and no known good "
                                       "values here either. Nothing forwarded.\r\n");
                                beep(-1, buzzer);
                                break;
                            }
                        }

                        // FORWARD TO SUB: Force the Sub to physically commit these parameters to its local persistent EEPROM/flash.
                        RfIn.ack = SET;
                        if (xQueueSend(serOut, (void *)&RfIn, pdMS_TO_TICKS(250)) != pdTRUE) {
                            printf("ERROR: Failed to queue SETUPDATA forward request to serOut!\r\n");
                        }
                        else{
                            printf("#SETUPDATA: Updated parameters from Sub. Forwarding to Sub to save permanently. maxSpeed=%d\r\n", RfIn.maxSpeed);
                        } 

                    } 
                    
                    // Route Tracking: Record the sender ID who initiated the request,
                    // allowing us to route the Sub's asynchronous reply back to the correct requester.
                    lastSetupRequester = RfIn.IDs;

                    // Forward to Sub to trigger a fresh update. 
                    // Overwrite the sender ID with our own Top MAC address to ensure the half-duplex serial driver
                    // recognizes and ignores self-echoed transactions.
                    // Only forward to our local Sub if the command is explicitly addressed to us.
                    // A broadcast SETUPDATA would otherwise make every buoy commit another buoy's
                    // configuration to its own Sub EEPROM.
                    if (RfIn.IDr == RfOut->mac) {
                        RfIn.IDr = BUOYIDALL;
                        RfIn.IDs = espMac();
                        printf("DEBUG_SETUPDATA: Forwarding to Sub via serOut. maxSpeed=%d, ack=%d\r\n", RfIn.maxSpeed, RfIn.ack);
                        if (xQueueSend(serOut, (void *)&RfIn, pdMS_TO_TICKS(250)) != pdTRUE) {
                            printf("ERROR: Failed to queue SETUPDATA forward request to serOut!\r\n");
                        }
                    }
                }
                else
                {
                    printf("Received SETUPDATA command. ack=%d Updating parameters\r\n", RfIn.ack);
                    // Update buoyPara base so web interface shows correct remote setup
                    // Prioritize Buoy 0 (Main) if its IDs matches or if it hasn't synced its Sub ID yet.

                    int targetIdx = -1;
                    // A reply carrying either of OUR identities is ours, and slot 0 is the only
                    // slot that may hold it. We answer to both the physical MAC and the logical
                    // ID re-synced from the Sub; the old test matched slot 0 by IDs alone and
                    // fell back to "first free slot from 1", so when those two diverged our own
                    // setup reply was filed under a remote buoy. The web Setup dialog then
                    // showed our PID / limits / compass values under that buoy's number, and its
                    // rev counter advanced, which is what made the wrong data look freshly
                    // fetched rather than stale.
                    if (RfIn.IDs == RfOut->mac || RfIn.IDs == RfOut->IDs) {
                        targetIdx = 0;
                    }
                    else {
                        for (int i = 1; i < 3; i++) {
                            if (buoyPara[i]->IDs == RfIn.IDs) {
                                targetIdx = i;
                                break;
                            }
                        }
                        if (targetIdx == -1) {
                            for (int i = 1; i < 3; i++) {
                                if (buoyPara[i]->IDs == 0) {
                                    targetIdx = i;
                                    break;
                                }
                            }
                        }
                    }

                    if (targetIdx != -1) {
                        // Merge instead of assigning: a SETUPDATA reply carries no position or
                        // status, so a plain overwrite would blank the remote buoy's fix until
                        // its next telemetry packet arrived.
                        int rev = buoyPara[targetIdx]->sub_status;
                        MergeBuoyData(buoyPara[targetIdx], RfIn);
                        // The revision counter is not part of the wire format, so bump it here the
                        // same way handleSerialData() does for our own Sub. The web Setup dialog
                        // only opens once rev advances past the value it saw when it asked, so
                        // without this a remote buoy's Setup would sit on "Collecting data..."
                        // until it timed out, even though the data had already arrived.
                        buoyPara[targetIdx]->sub_status = rev + 1;
                        // Sync mainData ID on first contact only. This was a no-op before, since
                        // slot 0 could only be selected when its IDs already equalled RfIn.IDs.
                        // Now that a reply matching our MAC also lands here, an unguarded
                        // assignment would overwrite the logical ID synced from the Sub.
                        if (targetIdx == 0 && RfOut->IDs == 0) RfOut->IDs = RfIn.IDs;
                    }
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;
            }
            case CAL8_SESSION:
            {
                // Guided calibration, driven from the handheld or another client. The Sub owns the
                // session and the Top has no opinion about any of it - but the Top does own the
                // serial hop, which drops most of what is sent, so a SET goes through the retry
                // here instead of down the wire once and hopefully. That means the handheld, the
                // dashboard and this Top's own page all get the same reliability without any of
                // them implementing it. See cal8Service().
                bool is_local = (RfIn.IDs == RfOut->mac || RfIn.IDs == 0x98 || RfIn.IDs == 0x99 || from_udp);
                if (is_local && (RfIn.ack == GET || RfIn.ack == GETACK || RfIn.ack == SET))
                {
                    if (RfIn.IDr == RfOut->mac || RfIn.IDr == RfOut->IDs || RfIn.IDr == BUOYIDALL || RfIn.IDr == 0) {
                        if (RfIn.ack == SET) {
                            cal8NotePress(RfIn.cal8Action, RfIn.cal8Next);
                        } else {
                            RfIn.IDr = BUOYIDALL;
                            RfIn.IDs = espMac();
                            if (xQueueSend(serOut, (void *)&RfIn, pdMS_TO_TICKS(250)) != pdTRUE) {
                                printf("ERROR: Failed to queue CAL8_SESSION forward to serOut!\r\n");
                            }
                        }
                    }
                }
                else
                {
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;
            }
            case STORE_INTERPOLATION_TABLE:
            {
                printf("Received STORE_INTERPOLATION_TABLE command. ack=%d, IDs=0x%08lX\r\n", RfIn.ack, RfIn.IDs);
                bool is_local = (RfIn.IDs == RfOut->mac || RfIn.IDs == 0x98 || RfIn.IDs == 0x99 || from_udp);
                if (is_local && (RfIn.ack == 1 || RfIn.ack == 3 || RfIn.ack == 2)) // 1=GET, 2=SET, 3=GETACK
                {
                    if (RfIn.IDr == RfOut->mac || RfIn.IDr == RfOut->IDs || RfIn.IDr == BUOYIDALL || RfIn.IDr == 0) {
                        RfIn.IDr = BUOYIDALL;
                        RfIn.IDs = espMac();
                        printf("DEBUG_INTERPOLATION_TABLE: Forwarding to Sub via serOut. ack=%d\r\n", RfIn.ack);
                        if (xQueueSend(serOut, (void *)&RfIn, pdMS_TO_TICKS(250)) != pdTRUE) {
                            printf("ERROR: Failed to queue STORE_INTERPOLATION_TABLE forward request to serOut!\r\n");
                        }
                    }
                }
                else
                {
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;
            }
            case PIDSPEEDSET:
                printf("#PIDSPEEDSET: %05.2f %05.2f %05.2f\r\n", RfIn.Kps, RfIn.Kis, RfIn.Kds);
                RfOut->cmd = PIDSPEEDSET;
                RfIn.ack = SET; // Tell Sub to save to EEPROM
                RfIn.IDr = BUOYIDALL;
                if (xQueueSend(serOut, (void *)&RfIn, pdMS_TO_TICKS(250)) != pdTRUE) {
                    printf("ERROR: Failed to queue PIDSPEEDSET to serOut!\r\n");
                }
                RfOut->Kps = RfIn.Kps;
                RfOut->Kis = RfIn.Kis;
                RfOut->Kds = RfIn.Kds;
                break;
            case TGDIRSPEED:
                if (RfIn.IDr == RfOut->mac || ((RfIn.IDr == BUOYIDALL || RfIn.IDr == 0) && (RfIn.IDs == 0x99 || RfIn.IDs == 0x98)))
                {
                    RfOut->tgDir = RfIn.tgDir;
                    RfOut->speedSet = RfIn.speedSet;
                    RfOut->status = TGDIRSPEED;
                    RfOut->lastSerOut = 0; // Force immediate update to sub
                }
                else
                {
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;
            case REMOTE:
                if (RfIn.IDr == RfOut->mac || ((RfIn.IDr == BUOYIDALL || RfIn.IDr == 0) && (RfIn.IDs == 0x99 || RfIn.IDs == 0x98)))
                {
                    if (RfOut->status != REMOTE) {
                        beep(1, buzzer);
                        RfOut->status = REMOTE;
                    }
                    RfOut->tgDir = RfIn.tgDir;
                    RfOut->tgSpeed = RfIn.tgSpeed;
                    xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                    RfOut->lastSerOut = 0; // Force immediate update to sub
                }
                else
                {
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;
            case DIRDIST:
                if (RfIn.IDr == RfOut->mac || ((RfIn.IDr == BUOYIDALL || RfIn.IDr == 0) && (RfIn.IDs == 0x99 || RfIn.IDs == 0x98)))
                {
                    double lat, lon;
                    adjustPositionDirDist(RfIn.tgDir, RfIn.tgDist, RfOut->lat, RfOut->lng, &RfOut->tgLat, &RfOut->tgLng);
                    RouteToPoint(RfOut->lat, RfOut->lng, RfOut->tgLat, RfOut->tgLng, &RfOut->tgDist, &RfOut->tgDir);

                    if (RfOut->status != LOCKED) {
                        RfIn.cmd = RESET_SPEED_RUD_PID;
                        xQueueSend(serOut, (void *)&RfIn, 0); // update sub only if transitioning
                    }
                    RfOut->status = LOCKED;
                    RfOut->lastSerOut = 0; // Force immediate update to sub
                }
                else
                {
                    // If it is just a broadcast from another buoy, store its data in the buoyPara base,
                    // but do NOT execute it as a command for ourselves!
                    AddDataToBuoyBase(RfIn, buoyPara);
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;
            case LOCKING:
                if (RfIn.IDr == RfOut->mac || ((RfIn.IDr == BUOYIDALL || RfIn.IDr == 0) && (RfIn.IDs == 0x99 || RfIn.IDs == 0x98)))
                {
                    if (RfOut->gpsFix == true && RfOut->status != LOCKING && RfOut->status != LOCKED)
                    {
                        RfOut->status = LOCKING;
                        beep(1, buzzer);
                        RfOut->lastSerOut = 0; // Force immediate update to sub
                    }
                }
                else
                {
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;
            case DOCKPOS: // Someone is asking where our dock is - answer, do not act on it
            {
                // An INF frame is somebody's ANSWER, not a question. The reply built below is
                // itself a DOCKPOS, so without this test two Tops answer each other's answers:
                // on the wire after a dock, .78 broadcast DOCKPOS, .71 replied, and .78 then
                // replied to that reply. Only questions get answered.
                if (RfIn.ack == INF)
                {
                    break;
                }
                crumb(20);
                // Answered out of RfIn rather than a second RoboStruct. Reading the dock straight
                // into RfOut (= mainData) is what we must not do - memDockPos() writes tgLat/tgLng
                // and that silently retargeted us at the dock whatever we were doing, including
                // holding a lock. But a private copy does not have to be a fresh ~500-byte struct
                // on a stack that already carries RfIn; case SETLOCKPOS reuses RfIn for the same
                // reason. RfIn is ours to scribble on, and we are finished reading it here.
                memDockPos(&RfIn, MEM_GET);
                crumb(21);
                memDockApproach(&RfIn, MEM_GET);
                crumb(22);
                // Address the answer at whoever asked. It used to inherit IDr = BUOYIDALL from
                // mainData, so every answer was broadcast to the whole fleet and every other Top
                // took it for a fresh question.
                RfIn.IDr = RfIn.IDs;
                RfIn.IDs = RfOut->mac;
                RfIn.cmd = DOCKPOS;
                RfIn.ack = INF;
                if (from_udp) xQueueSend(udpOut, (void *)&RfIn, 0);
                else xQueueSend(loraOut, (void *)&RfIn, 0);
                crumb(23);
                udpLog("DOCKPOS answered to %08lX via %s", (unsigned long)RfIn.IDr,
                       from_udp ? "udp" : "lora");
                break;
            }
            case STOREASDOC: // Store location as doc location
                if (RfOut->gpsFix == true)
                {
                    RfOut->tgLat = RfOut->lat;
                    RfOut->tgLng = RfOut->lng;
                    memDockPos(RfOut, MEM_PUT);
                    beep(1000, buzzer);
                }
                else
                {
                    beep(-1, buzzer);
                }
                break;
            case SETLOCKPOS: // store new data into position database and sail to it
                RfOut->tgLat = RfIn.tgLat;
                RfOut->tgLng = RfIn.tgLng;
                RfIn.IDs = RfOut->mac; // Put this Id in field for positioning
                AddDataToBuoyBase(RfIn, buoyParaPtrs);
                RfOut->status = LOCKED;
                RfOut->lastSerOut = 0; // Force immediate update to sub

                // Announce the commanded waypoint, the way a manual lock does. This case sets
                // LOCKED directly instead of going through LOCKING, so handleStatus() never runs
                // its LOCKPOS broadcast and nobody else learned the new target - after a
                // COMPUTE STARTLINE the other buoy and the CYD kept showing the old one.
                //
                // Deliberately NOT routed through case LOCKING: that sets tgLat/tgLng to the
                // CURRENT position, which would throw away the waypoint we were just given.
                //
                // Reuses RfIn rather than declaring another RoboStruct - it is ~500 bytes and
                // this runs in the loop task.
                RfIn.IDr = BUOYIDALL;
                RfIn.cmd = LOCKPOS;
                RfIn.status = RfOut->status;
                RfIn.wDir = RfOut->wDir;
                RfIn.wStd = RfOut->wStd;
                RfIn.ack = INF; // INF, not SET: SET would enter the LoRa ACK retry table and be
                                // retransmitted 5x, and the periodic re-broadcast already covers loss
                xQueueSend(udpOut, (void *)&RfIn, 0);
                xQueueSend(loraOut, (void *)&RfIn, 10);
                break;
            case IDLING:
            case IDLE:
                if (RfIn.IDr == RfOut->mac || ((RfIn.IDr == BUOYIDALL || RfIn.IDr == 0) && (RfIn.IDs == 0x99 || RfIn.IDs == 0x98)))
                {
                    if (RfOut->status != IDLING && RfOut->status != IDLE)
                    {
                        printf("#Status set to IDLE (by lora input)\r\n");
                        RfOut->status = IDLING;
                        RfOut->lastSerOut = 0; // Force immediate update to sub
                    }
                }
                else
                {
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;
            case SUBACCU:
                if (RfIn.IDs != 0 && RfIn.IDs != RfOut->IDs && RfIn.IDs != RfOut->mac)
                {
                    AddDataToBuoyBase(RfIn, buoyPara);
                }
                else
                {
                    RfOut->subAccuV = RfIn.subAccuV;
                    RfOut->subAccuP = RfIn.subAccuP;
                }
                break;
            case SUBPWR:
                if (RfIn.IDs != 0 && RfIn.IDs != RfOut->IDs && RfIn.IDs != RfOut->mac)
                {
                    AddDataToBuoyBase(RfIn, buoyPara);
                }
                else
                {
                    RfOut->speedBb = RfIn.speedBb;   // set speed for bow
                    RfOut->speedSb = RfIn.speedSb;   // set speed for stern
                    RfOut->subAccuV = RfIn.subAccuV; // set sub accu voltage
                }
                break;
            case STORE_DECLINATION:
                // Retired, see RoboCompute.h. No longer stored, and no longer forwarded to the
                // Sub - the heading correction that actually does something is compassOffset.
                printf("STORE_DECLINATION ignored - declination is retired\r\n");
                break;
            case MAXMINPWR:
                if (RfIn.ack == GET || RfIn.ack == GETACK)
                {
                    xQueueSend(serOut, (void *)&RfIn, 0); // update sub
                }
                else
                {
                    // For status updates from other buoys, only forward to UDP for local display
                    xQueueSend(udpOut, (void *)&RfIn, 0);
                    xQueueSend(loraOut, (void *)&RfIn, 0); // For status updates from other buoys
                }
                break;
            case MAXMINPWRSET:
                RfIn.ack = SET; // Tell Sub to save to EEPROM
                if (xQueueSend(serOut, (void *)&RfIn, pdMS_TO_TICKS(250)) != pdTRUE) {
                    printf("ERROR: Failed to queue MAXMINPWRSET to serOut!\r\n");
                }
                break;
            case STORE_COMPASS_OFFSET:
                RfOut->compassOffset = RfIn.compassOffset; // Update Top Buoy's local data
                break;
            case ADAPTIVE_TRIM:
                {
                    RfOut->compass_trim = RfIn.compass_trim;
                    RfOut->compass_trim_enabled = RfIn.compass_trim_enabled;

                    int targetIdx = -1;
                    for (int i = 0; i < 3; i++) {
                        if (buoyPara[i]->IDs == RfIn.IDs) {
                            targetIdx = i;
                            break;
                        }
                    }
                    if (targetIdx == -1) {
                        for (int i = 0; i < 3; i++) {
                            if (buoyPara[i]->IDs == 0) {
                                targetIdx = i;
                                break;
                            }
                        }
                    }
                    if (targetIdx != -1) {
                        buoyPara[targetIdx]->compass_trim = RfIn.compass_trim;
                        buoyPara[targetIdx]->compass_trim_enabled = RfIn.compass_trim_enabled;
                    }
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                
                RfIn.IDr = BUOYIDALL;
                xQueueSend(serOut, (void *)&RfIn, 0); // Forward the command to the sub
                break;
            case SET_AS_NORTH:
                RfIn.IDr = BUOYIDALL;
                xQueueSend(serOut, (void *)&RfIn, 0); // Forward the command to the sub
                // This one changes compassOffset in the Sub's NVS, so re-read it rather than
                // waiting up to a resync interval. Queued behind the command, so the Sub has
                // already applied it by the time it answers.
                requestSubSetup("after SET_AS_NORTH");
                break;
            // The Sub has always handled this (its case CALIBRATE_MAGNETIC_COMPASS), and the web
            // page and the CYD both offer it, but nothing here forwarded it - so "Desk
            // Calibration" pressed anywhere other than the Top's own button did nothing at all.
            case CALIBRATE_MAGNETIC_COMPASS:
                RfIn.IDr = BUOYIDALL;
                xQueueSend(serOut, (void *)&RfIn, 0); // Forward the command to the sub
                break;
            case GPS_FOURIER_CALIBRATE:
                // Runs entirely on the Top - it steers the buoy and does the arithmetic, and only
                // talks to the Sub through TGDIRSPEED and STORE_INTERPOLATION_TABLE. So this is
                // NOT forwarded to the Sub, it just arms our own state machine.
                if (RfIn.IDr == RfOut->mac || ((RfIn.IDr == BUOYIDALL || RfIn.IDr == 0) && (RfIn.IDs == 0x99 || RfIn.IDs == 0x98)))
                {
                    if (RfOut->status != GPS_FOURIER_CALIBRATE)
                    {
                        // Latched into mainData before the status flips, because the state machine
                        // reads the mode out of mainData on the pass where it starts the run.
                        RfOut->gpsCalStillWater = RfIn.gpsCalStillWater;
                        RfOut->gpsCalStartHeading = RfIn.gpsCalStartHeading;
                        printf("#Status set to GPS_FOURIER_CALIBRATE (%s, start dir: %.0f deg)\r\n",
                               RfIn.gpsCalStillWater ? "still water" : "pair averaged",
                               (double)RfIn.gpsCalStartHeading);
                        RfOut->status = GPS_FOURIER_CALIBRATE;
                    }
                }
                else
                {
                    // Forward across interfaces
                    if (from_udp) xQueueSend(loraOut, (void *)&RfIn, 0);
                    else xQueueSend(udpOut, (void *)&RfIn, 0);
                }
                break;
            case REBOOT:
                // Belt and braces against a reboot storm. A sender that puts REBOOT in the LoRa
                // retransmit table (ack GETACK/SET -> retry = 5 in loratop.cpp) can never be
                // acknowledged by a buoy that is busy restarting, so every retry lands on a
                // freshly booted Top and reboots it again. Ignoring reboots for the first few
                // seconds of a boot breaks that cycle whatever the sender does.
                if (millis() < 15000) {
                    printf("REBOOT ignored: only %lu ms since boot, treating it as a retry of the "
                           "reboot we just performed.\r\n", millis());
                    break;
                }
                RfIn.IDr = BUOYIDALL;
                xQueueSend(serOut, (void *)&RfIn, 0); // Forward the command to the sub
                printf("REBOOT command received! Forwarding to Sub and rebooting Top...\r\n");
                {
                    extern QueueHandle_t buzzer;
                    if (buzzer != NULL) {
                        beep(2, buzzer);
                    }
                }
                delay(500); // Give serial/sercom transmission a moment to finish, and play the beep
                ESP.restart();
                break;
            case RAWCOMPASSDATA:
                printf("Raw:0,0,0,0,0,0,%5.3f,%5.3f,%5.3f\r\n", RfIn.magHard[0], RfIn.magHard[1], RfIn.magHard[2]);
                break;
            default:
                break;
            }
        }
    }
}

// ***************************************************************************************************
// handle new gps datainput
// ***************************************************************************************************
/**
 * @brief Processes incoming GPS data from the GPS queue and applies filtering.
 * 
 * @param gps Pointer to the RoboStruct where GPS coordinates and fix status will be updated.
 */
void handleGpsData(RoboStruct *gps)
{
    RoboStruct gpsin;

    if (xQueueReceive(gpsQue, (void *)&gpsin, 0) == pdTRUE) // New gps data
    {
        // A fix at exactly 0,0 is not a position, it is an uninitialised one. The receiver hands
        // these out occasionally with the valid flag still set, and the outlier filter below
        // treats them as a 5800 km jump - the distance from here to null island - so 30 of them
        // in a row would be "accepted as the new position" and the buoy would try to sail there.
        if (gpsin.lat == 0.0 && gpsin.lng == 0.0)
        {
            gpsin.gpsFix = false;
        }

        if (gpsin.gpsFix)
        {
            // --- First valid fix initialization ---
            if (!gps->gpsFix)
            {
                gpsErrorCnt = 0;
                gps->lat = gpsin.lat;
                gps->lng = gpsin.lng;
                gps->gpsSat = gpsin.gpsSat;
                gps->gpsFix = true;
                // First time fix event
                Serial.printf("First GPS fix acquired\r\n");
                beep(2000, buzzer);
                return;
            }

            // --- Accept small movements immediately ---
            double dist = distanceBetween(gpsin.lat, gpsin.lng, gps->lat, gps->lng);
            if (dist < 50)
            {
                gpsErrorCnt = 0;
                gps->lat = gpsin.lat;
                gps->lng = gpsin.lng;
                gps->gpsSat = gpsin.gpsSat;
                gps->gpsFix = true;
            }
            else
            {
                // Large jump: might be an outlier OR the buoy was moved
                gpsErrorCnt++;
                Serial.printf("Potential GPS outlier (dist: %.2f m), count: %u\r\n", dist, gpsErrorCnt);
                
                // If we get many consecutive outliers, accept the new position
                if (gpsErrorCnt > 30) 
                {
                    Serial.println("Accepting new GPS position after persistent outliers.");
                    gpsErrorCnt = 0;
                    gps->lat = gpsin.lat;
                    gps->lng = gpsin.lng;
                    gps->gpsSat = gpsin.gpsSat;
                    gps->gpsFix = true;
                }
            }
        }
        else
        {
            // Fix lost
            gps->gpsFix = false;
            gps->gpsSat = gpsin.gpsSat;
        }
    }
}
//***************************************************************************************************
//      New serial data
//***************************************************************************************************
/**
 * @brief Processes incoming serial data from the sub-controller.
 * 
 * @param ser Pointer to the RoboStruct to be updated with serial data.
 */
void handleSerialData(RoboStruct *ser, RoboStruct *buoyPara[3])
{
    RoboStruct serDataIn;
    if (xQueueReceive(serIn, (void *)&serDataIn, 1) == pdTRUE)
    {
        lastRealSerIn = millis();
        crumb(2000 + serDataIn.cmd);
        if (!noisyCmd(serDataIn.cmd))
        {
            udpLog("SER in cmd=%d ack=%d IDr=%08lX IDs=%08lX",
                   serDataIn.cmd, serDataIn.ack, (unsigned long)serDataIn.IDr,
                   (unsigned long)serDataIn.IDs);
        }
        if (!isSerConnected)
        {
            isSerConnected = true;
            printf("Serial connection made to the sub (Serial1). Requesting SETUPDATA...\r\n");
            requestSubSetup("serial link came up");
        }

        // Re-read the Sub's stored settings periodically, not just once when the link comes up.
        //
        // Everything in this struct - the PIDs, the compass offset, the speed limits, the thruster
        // flags - is OWNED by the Sub. We keep a RAM mirror so /data has something to serve, and
        // that mirror only ever refreshed when somebody opened a Setup dialog, which is what sends
        // a SETUPDATA GET. Anything that changed a setting without going through that dialog left
        // us serving a stale value indefinitely:
        //
        //   - SET_AS_NORTH: we forward it and the Sub commits a new compassOffset, but nothing
        //     here consumes its reply. Measured on the bench: Sub on -43.30, this Top still
        //     reporting -53.00 minutes later.
        //   - The Sub's OWN web page. /set_north and /setparam write straight to its NVS and never
        //     touch this Top at all, so no amount of handling commands on our side can catch them.
        //   - In-field calibration, MAN CAL, anything else that lands on the Sub directly.
        //
        // Re-asking the owner is the only thing that covers all of them, and it is self-healing:
        // however the value got out of step, the next tick puts it right. One small frame every
        // SUB_SETUP_RESYNC_MS is nothing next to the telemetry already on this link.
        //
        // Safe against the Setup dialog, which waits for the revision counter to go UP before it
        // populates: an extra bump can only make it open with data that is equally fresh, never
        // stale, and the dialog fills its fields once rather than tracking /data live, so this
        // cannot overwrite what an operator is typing.
        // Only while the Sub is genuinely talking to us. This poll is a convenience - it keeps
        // /data honest - and it is NOT worth keeping a buoy powered up for.
        //
        // The Sub refreshes its own POWEROFFTIME timer on every serial frame it RECEIVES
        // (RoboSub/src/main.cpp, PwrOff = millis() in handleSerandRfdata), and after 60 minutes
        // without one it pulls PWRENABLE low and switches itself off to save the battery. An
        // unconditional poll every 20 s would refresh that timer forever and the buoy would never
        // sleep again - so a quiet Sub is left strictly alone and allowed to power down.
        //
        // Gated on lastRealSerIn rather than isSerConnected: that flag is cleared by the watchdog
        // and only set again when a frame arrives, so it can sit true for seconds after the Sub
        // has actually gone.
        if (isSerConnected && millis() - lastRealSerIn < 5000 &&
            millis() - lastSubSetupResyncMs > SUB_SETUP_RESYNC_MS)
        {
            requestSubSetup(NULL);
        }

        // For serial data, we ALWAYS update the local 'ser' (mainData)
        // This prevents remote buoy slots from "stealing" the update if IDs match.
        RoboStruct *target = ser;
        
        // Update local identity if the sub reports a different ID
        if (serDataIn.IDs != 0 && serDataIn.IDs != ser->IDs) {
            // printf("Syncing local buoy ID from %08X to %08X\r\n", (unsigned int)ser->IDs, (unsigned int)serDataIn.IDs);
            ser->IDs = serDataIn.IDs;
        }

        target->lastSerIn = millis();
        if (mainCollorUtil.color != CRGB::DarkBlue)
        {
            mainCollorUtil.color = CRGB::DarkBlue;
            mainCollorUtil.blink = BLINK_SLOW;
            xQueueSend(ledUtil, (void *)&mainCollorUtil, 0); // update GPS led
        }
        switch (serDataIn.cmd)
        {
        case ADAPTIVE_TRIM:
            if (target) {
                target->compass_trim = serDataIn.compass_trim;
                target->compass_trim_enabled = serDataIn.compass_trim_enabled;
            }
            break;
        case SUBDATA:
            target->dirMag = serDataIn.dirMag;
            // Imag - the heading before the Sub's compass table and trim. Relayed onward in
            // TOPDATA so a calibration can read it without switching anything off on the buoy.
            // Copied field by field here, so a new one has to be added explicitly or it is lost.
            target->imag = serDataIn.imag;
            target->speedBb = serDataIn.speedBb;
            target->speedSb = serDataIn.speedSb;
            target->ip = serDataIn.ip;
            target->ir = serDataIn.ir;
            target->subAccuV = serDataIn.subAccuV;
            target->subAccuP = serDataIn.subAccuP;
            target->subAccuI = serDataIn.subAccuI;
            
            mainPwrData.ledBb = target->speedBb;
            mainPwrData.ledSb = target->speedSb;
            xQueueSend(ledPwr, (void *)&mainPwrData, 0);
            break;
        case SETUPDATA:
            if (serDataIn.ack == 1 || serDataIn.ack == 3 || serDataIn.ack == 2) { // 1=GET, 2=SET, 3=GETACK
                // This is a request from PC (Serial) -> Forward to Sub
                serDataIn.IDr = BUOYIDALL; // Force Sub to accept the request
                xQueueSend(serOut, (void *)&serDataIn, 0);
            } else if (serDataIn.ack == INF) {
                // This is the response coming FROM the Sub
                // Update the correct buoy's data
                target->Kpr = serDataIn.Kpr;
                target->Kir = serDataIn.Kir;
                target->Kdr = serDataIn.Kdr;
                target->Kps = serDataIn.Kps;
                target->Kis = serDataIn.Kis;
                target->Kds = serDataIn.Kds;
                target->maxSpeed = serDataIn.maxSpeed;
                target->minSpeed = serDataIn.minSpeed;
                target->pivotSpeed = serDataIn.pivotSpeed;
                target->compassOffset = serDataIn.compassOffset;
                target->holdRad = serDataIn.holdRad;
                target->revBB = serDataIn.revBB;
                target->revSB = serDataIn.revSB;
                target->swap_BB_SB = serDataIn.swap_BB_SB;
                // What the Sub reports here is what it actually has in NVS, so it is the
                // authority for the Setup page's "Active Trim Enabled" tick box.
                target->compass_trim_enabled = serDataIn.compass_trim_enabled;
                // The harmonic correction switch lives on the Sub; this reply is the only
                // place the Top ever learns its state.
                target->interpEnabled = serDataIn.interpEnabled;
                target->sub_status++; // Use sub_status as a local revision counter for Web UI

                // Inject our local docking parameters into the sub response before forwarding!
                serDataIn.dockApproachDist = target->dockApproachDist;
                serDataIn.dockApproachDir = target->dockApproachDir;
                serDataIn.dockingToWaypoint = target->dockingToWaypoint;

                // IMPORTANT: Send the actual protocol string to the PC via Serial so RoboControl.py sees it!
                Serial.println(rfCode(&serDataIn));

                // FORWARD to LoRa/UDP
                // The SETUPDATA response from the Sub MUST be broadcasted with IDr = BUOYIDALL (which is 1)
                // before pushing the response to UDP and LoRa.
                serDataIn.IDr = BUOYIDALL;

                uint64_t subID = serDataIn.IDs; // Save Sub's actual ID before overwriting

                // Ensure sender ID matches the Top's MAC address so the PC's UI maps it to the correct buoy
                serDataIn.IDs = espMac(); 

                if (xQueueSend(udpOut, (void *)&serDataIn, pdMS_TO_TICKS(250)) != pdTRUE) {
                    printf("ERROR: Failed to queue SETUPDATA response to udpOut!\r\n");
                }
                
                // For LoRa, always broadcast with IDr = BUOYIDALL (1) matching the protocol standard and ensuring
                // both the physical screen (0x98) and its webpage Setup clients receive the response successfully.
                RoboStruct loraDataOut = serDataIn;
                if (xQueueSend(loraOut, (void *)&loraDataOut, pdMS_TO_TICKS(250)) != pdTRUE) {
                    printf("ERROR: Failed to queue SETUPDATA response to loraOut!\r\n");
                }

                // Send an explicit ACK back to the Sub to clear its retransmission queue
                serDataIn.ack = ACK;
                serDataIn.IDr = subID; // Send back to the Sub's actual ID
                serDataIn.IDs = espMac();
                if (xQueueSend(serOut, (void *)&serDataIn, pdMS_TO_TICKS(250)) != pdTRUE) {
                    printf("ERROR: Failed to queue SETUPDATA ACK to serOut!\r\n");
                }

                // printf("DEBUG: Received Kpr=%f, Kir=%f, Kdr=%f, Kps=%f, Kis=%f, Kds=%f\r\n", serDataIn.Kpr, serDataIn.Kir, serDataIn.Kdr, serDataIn.Kps, serDataIn.Kis, serDataIn.Kds);
                // printf("Setup data PID and Compass received from Sub and updated (Rev: %d)\r\n", target->sub_status);
            }
            break;
        case CAL8_SESSION:
            // The Sub's answer to a guided calibration press. Cache it for this Top's own page,
            // then pass it on unchanged: the handheld and the dashboard are watching the same run
            // and must see the same step, which only works if nobody re-derives it.
            cal8NoteState(serDataIn.cal8Active, serDataIn.cal8Next, serDataIn.cal8Captured);
            serDataIn.IDr = BUOYIDALL;
            serDataIn.IDs = espMac();
            xQueueSend(udpOut, (void *)&serDataIn, pdMS_TO_TICKS(100));
            xQueueSend(loraOut, (void *)&serDataIn, pdMS_TO_TICKS(100));
            break;
        case STORE_INTERPOLATION_TABLE:
            // Answer to either half of the calibration handshake: the table the Sub is running, or
            // the echo of the one it has just stored.
            printf("Interpolation table from Sub:");
            for (int i = 0; i < 8; i++) printf(" %.2f", serDataIn.interpolationTable[i]);
            printf("\r\n");
            gpsCalibTableReply(&serDataIn);
            // Also feed the web page's MAN CAL working copy - see topwifi.h.
            mancalNoteTable(serDataIn.interpolationTable, serDataIn.interpEnabled);

            // Broadcast the table over UDP and LoRa so clients can receive it!
            serDataIn.IDr = BUOYIDALL;
            serDataIn.IDs = espMac();
            if (xQueueSend(udpOut, (void *)&serDataIn, pdMS_TO_TICKS(100)) != pdTRUE) {
                printf("ERROR: Failed to queue table broadcast to udpOut!\r\n");
            }
            if (xQueueSend(loraOut, (void *)&serDataIn, pdMS_TO_TICKS(100)) != pdTRUE) {
                printf("ERROR: Failed to queue table broadcast to loraOut!\r\n");
            }
            break;
        case PONG:
            break;
        case DIRSPEED:
            ser->dirMag = serDataIn.dirMag;
            ser->speedBb = serDataIn.speedBb;
            ser->speedSb = serDataIn.speedSb;
            printf("M:%03.0f T:%03.0f D:%03.1f   BB:%d SB:%d\r\n", ser->dirMag, ser->tgDir, ser->tgDist, ser->speedBb, ser->speedSb);
            
            mainPwrData.ledBb = ser->speedBb;
            mainPwrData.ledSb = ser->speedSb;
            xQueueSend(ledPwr, (void *)&mainPwrData, 0);
            break;
        case SUBACCU:
            ser->subAccuV = serDataIn.subAccuV;
            ser->subAccuP = serDataIn.subAccuP;
            break;
        case IDLING:
            if (ser->status != IDLING && ser->status != IDLE)
            {
                printf("#Status set to IDLING (by serial input)\r\n");
                ser->status = IDLING;
            }
            break;
        case IDLE:
            // Prevent delayed IDLE from Sub buoy from canceling an active state
            if (ser->status == IDLING || ser->status == IDLE) {
                ser->status = IDLE;
            }
            break;
        case LOCKED:
            if (ser->status != LOCKED && ser->status != LOCKING && ser->status != LOCKPOS) {
                ser->status = LOCKED;
            }
            break;
        case DOCKED:
            if (ser->status != DOCKED && ser->status != DOCKING) {
                ser->status = DOCKED;
            }
            break;
        case STOREASDOC:
            if (ser->gpsFix == true)
            {
                printf("Store Doc pos)\r\n");
                ser->tgLat = ser->lat;
                ser->tgLng = ser->lng;
                memDockPos(ser, MEM_PUT);
            }
            ser->status = IDLING;
            break;
        case RAWCOMPASSDATA:
            printf("Raw:0,0,0,0,0,0,%.0f,%.0f,%.0f\r\n", serDataIn.magHard[0], serDataIn.magHard[1], serDataIn.magHard[2]);
            break;
        case PIDRUDDER:
        case PIDRUDDERSET:
        case PIDSPEED:
        case PIDSPEEDSET:
        case MAXMINPWR:
        case MAXMINPWRSET:
            if (serDataIn.ack == GET || serDataIn.ack == GETACK)
            {
                // This is a request from PC (Serial) -> Forward to Sub
                serDataIn.IDr = BUOYIDALL; // Force Sub to accept the request
                xQueueSend(serOut, (void *)&serDataIn, 0);
            }
            else
            {
                // This is a response from Sub -> Forward to PC/LoRa/UDP
                // printf("Received PID/PWR data from sub. CMD: %d\r\n", serDataIn.cmd);
                
                // IMPORTANT: Send the actual protocol string to the PC via Serial so RoboControl.py sees it!
                Serial.println(rfCode(&serDataIn));

                // FORWARD info to LoRa/UDP BEFORE transforming into ACK
                // Copy the message, then force broadcast routing and align the sender ID to our Top MAC
                RoboStruct forwardMsg = serDataIn;
                forwardMsg.IDr = BUOYIDALL;
                forwardMsg.IDs = espMac();

                if (xQueueSend(udpOut, (void *)&forwardMsg, pdMS_TO_TICKS(250)) != pdTRUE) {
                    printf("ERROR: Failed to queue PID/PWR response to udpOut!\r\n");
                }
                if (xQueueSend(loraOut, (void *)&forwardMsg, pdMS_TO_TICKS(250)) != pdTRUE) {
                    printf("ERROR: Failed to queue PID/PWR response to loraOut!\r\n");
                }

                // Acknowledge the Sub to stop its retry loop
                serDataIn.ack = GETACK;
                if (xQueueSend(serOut, (void *)&serDataIn, pdMS_TO_TICKS(250)) != pdTRUE) {
                    printf("ERROR: Failed to queue PID/PWR ACK to serOut!\r\n");
                }
            }
            break;

        default:
            break;
        }
    }
    if (ser->lastSerIn + 2000 < millis())
    {
        ser->lastSerIn = millis();
        if (mainCollorUtil.color != CRGB::Red)
        {
            mainCollorUtil.color = CRGB::Red;
            mainCollorUtil.blink = BLINK_SLOW;
            xQueueSend(ledUtil, (void *)&mainCollorUtil, 0); // update GPS led
        }
    }

    if (isSerConnected && (millis() - lastRealSerIn > 2000))
    {
        isSerConnected = false;
        printf("Serial connection to the sub lost (no messages for 2000ms).\r\n");
    }
}

/**
 * @brief Is our own Sub still sending data over the serial link?
 *
 * Used by the web dashboard (/data -> "SubOk") to grey out SETUP: every value in that dialog comes
 * from the Sub, so with a mute Sub the request can only time out on an empty form.
 *
 * Deliberately based on lastRealSerIn and not on mainData.lastSerIn: handleSerialTimeOut() above
 * resets lastSerIn to millis() when it turns the util LED red, so that field never stays stale and
 * cannot be used to detect silence. The 3 s window is slightly wider than that LED's 2 s so the
 * button does not flicker on a single dropped telemetry frame.
 */
bool subSerialAlive(void)
{
    // Nothing received since boot: millis() - 0 is small for the first seconds, which would
    // otherwise read as "alive" before the Sub has ever spoken.
    if (lastRealSerIn == 0) return false;
    // Note that the serial watchdog parks lastRealSerIn in the future as a grace period after a
    // wakeup. The unsigned subtraction then wraps to a huge value, which correctly reads as dead -
    // the Sub really is silent while it reboots.
    return (millis() - lastRealSerIn) < 3000;
}

//***************************************************************************************************
//      Main loop
//***************************************************************************************************
/**
 * @brief Main execution loop that continuously calls various handler functions.
 */
void loop(void)
{
    mainData.mac = espMac();
    if (mainData.IDs == 0) mainData.IDs = espMac();
    Serial.println("Main loop running!");
    {
        char tag[20];
        snprintf(tag, sizeof(tag), "TOP-%08lx", (unsigned long)espMac());
        udpLogBegin(tag);
    }
    mainData.status = IDLE;
    mainCollorGps.color = CRGB::DarkRed;
    mainCollorGps.blink = BLINK_FAST;
    xQueueSend(ledGps, (void *)&mainCollorGps, 0); // update GPS led
    mainCollorUtil.blink = BLINK_SLOW;
    beep(1, buzzer);
    mainData.lastLoraOut = millis();
    while (true)
    {
        // Safety: Ensure status is never 0 (which greys out buttons)
        if (mainData.status <= 0) mainData.status = IDLE;

        // Ensure mac is always set correctly
        mainData.mac = espMac();

        // Keep placeholder in sync for dashboard
        buoyPara[0] = mainData;

        // Slot 0 is us, and only us. We answer to two identities: the physical MAC and the
        // logical ID, which is re-synced from whatever the Sub reports (see the serial handler
        // and the targetIdx == 0 case in handleRfData). Once those two diverge, a record filed
        // under the identity that does NOT match slot 0 no longer matches in
        // AddDataToBuoyBase()'s first pass, so it is allocated a slot of its own - and we end up
        // in the base twice. The dashboard then draws a third buoy carrying our own ID with no
        // GPS fix and stale coordinates, and calcTrackPos()/recalcStartLine() see a zero-length
        // leg between us and our own phantom, which always wins the shortest-leg comparison.
        for (int i = 1; i < 3; i++)
        {
            if (buoyPara[i].IDs != 0 && (buoyPara[i].IDs == mainData.mac || buoyPara[i].IDs == mainData.IDs))
            {
                buoyPara[i] = RoboStruct();
            }
        }

        //***************************************************************************************************
        //      Timer routines
        //***************************************************************************************************
        crumb(90);
        handleTimerRoutines(&mainData);
        mancalSessionService();
    cal8Service();   // resend a calibration press until the Sub shows it landed
        {
            static unsigned long lastHealth = 0;
            if (millis() - lastHealth > 2000)
            {
                lastHealth = millis();
                udpLogHealth();
            }
        }
        //***************************************************************************************************
        //      Check front key
        //***************************************************************************************************
        crumb(91);
        handleKeyPress(&mainData);
        //***************************************************************************************************
        //      status actions
        //***************************************************************************************************
        crumb(92);
        handleStatus(&mainData, buoyPara);
        //***************************************************************************************************
        //      New data Rf data (UTP or Lora)
        //***************************************************************************************************
        crumb(93);
        handleRfData(&mainData, buoyParaPtrs);
        //****************************************************************************************************
        //      New GPS data
        //***************************************************************************************************
        crumb(94);
        handleGpsData(&mainData);
        //***************************************************************************************************
        //      New serial data
        //***************************************************************************************************
        // mainData = handleSerialData(mainData);
        crumb(95);
        handleSerialData(&mainData, buoyParaPtrs);
        //***************************************************************************************************
        //      Bench simulation of position and heading (normally disarmed, see gpssim.h)
        //***************************************************************************************************
        // Deliberately after handleSerialData(): that is where the Sub's real compass lands, so
        // this has to run later to overrule it. Does nothing at all unless armed.
        gpsSimUpdate(&mainData);
        //***************************************************************************************************
        //      Light button control
        //***************************************************************************************************
        crumb(96);
        buttonLight(&mainData);
        //***************************************************************************************************
        //      Serial watchdog
        //***************************************************************************************************
        if (isSerConnected && (millis() - lastRealSerIn > 5000))
        {
            printf("[Serial Watchdog] Connection lost (no RX for 5s). Triggering recovery...\n");
            // Force a re-init and wakeup of the Sub serial interface:
            RoboStruct wakeupMsg;
            wakeupMsg.cmd = WAKEUP;
            xQueueSend(serOut, (void *)&wakeupMsg, 0);
            
            // Set a future grace period to prevent spamming wakeup before sub can respond
            lastRealSerIn = millis() + 10000; // 15-second delay until next potential watchdog trigger
            isSerConnected = false;
        }

        //***************************************************************************************************
        //      Network health heartbeat
        //***************************************************************************************************
        // Printed from here, not from the WiFi task: this loop runs on core 1 and the WiFi task on
        // core 0, so the line keeps coming even when the WiFi task is the thing that stopped, and
        // then says loop=0. Without it, a Top that drops off the network while the button still
        // responds looks identical whether the task hung, the radio deassociated, or the heap ran dry.
        static unsigned long netHealthTimer = 0;
        if (millis() - netHealthTimer > 10000)
        {
            netHealthTimer = millis();
            printf("%s\r\n", netHealthLine().c_str());
        }

        // Immediate wireless broadcast on local status changes (evaluated at the end of the loop,
        // AFTER all state machine, GPS, serial, and navigation changes have been fully handled and calculated).
        static int lastLocalStatus = -1;
        if (mainData.status != lastLocalStatus)
        {
            lastLocalStatus = mainData.status;
            RoboStruct statusUpdate = mainData;
            statusUpdate.IDs = mainData.mac;
            statusUpdate.IDr = BUOYIDALL;
            statusUpdate.ack = INF;
            statusUpdate.cmd = (mainData.status == LOCKED || mainData.status == DOCKED) ? TOPDATA : BUOYPOS;
            xQueueSend(udpOut, (void *)&statusUpdate, 0);
            xQueueSend(loraOut, (void *)&statusUpdate, 0);
        }
    }
}
