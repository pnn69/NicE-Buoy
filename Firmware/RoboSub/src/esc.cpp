/*
    Motor control for RoboBuoy Sub
*/
#include <Arduino.h>
#include <ESP32Servo.h>
#include "main.h"
#include "esc.h"
#include "io_sub.h"
#include "leds.h"
#include "datastorage.h"
#include "udplog.h"

LedPwrtruct powerIndicator;
QueueHandle_t escspeed;

Servo servoBB;
Servo servoSB;

// ESC hardware parameters
#define ESC_FREQ 50       
#define ESC_MIN_US 1000   
#define ESC_MAX_US 2000   

unsigned long escStamp = 0;

// Per-thruster stop pulse. Loaded from NVS in EscTask before the ESCs are ever armed.
int esc_neutral_bb = ESC_NEUTRAL_NOMINAL_US;
int esc_neutral_sb = ESC_NEUTRAL_NOMINAL_US;

/**
 * @brief Converts speed % to a microsecond pulse, hinged on this thruster's own neutral.
 *
 * Two straight lines rather than one, meeting exactly at the trimmed neutral: -100% still reaches
 * ESC_MIN_US and +100% still reaches ESC_MAX_US, so trimming the stop point does not cost travel
 * at either end - it only moves where "stop" sits. A single map() across the whole range would
 * have skewed full scale as soon as the neutral moved off centre.
 *
 * Zero is returned as the neutral verbatim, never computed, so stop is always exactly the number
 * that was trimmed and cannot drift by a microsecond of integer rounding.
 */
uint16_t speedToPulse(int speed, bool invert, int neutralUs)
{
    if (invert) speed = -speed;
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    if (neutralUs < ESC_NEUTRAL_MIN_US || neutralUs > ESC_NEUTRAL_MAX_US) neutralUs = ESC_NEUTRAL_NOMINAL_US;

    if (speed == 0) return (uint16_t)neutralUs;
    if (speed > 0)  return (uint16_t)(neutralUs + (long)(ESC_MAX_US - neutralUs) * speed / 100);
    return (uint16_t)(neutralUs + (long)(neutralUs - ESC_MIN_US) * speed / 100);
}

void triggerESC(void)
{
    Serial.println("Triggering ESC Test Sequence...");
    servoBB.writeMicroseconds(speedToPulse(10, false, esc_neutral_bb));
    servoSB.writeMicroseconds(speedToPulse(10, false, esc_neutral_sb));
    vTaskDelay(pdMS_TO_TICKS(1000));
    servoBB.writeMicroseconds(speedToPulse(0, false, esc_neutral_bb));
    servoSB.writeMicroseconds(speedToPulse(0, false, esc_neutral_sb));
}

void playTone(int frequency) { }
void beepESC(void) { startESC(); }

void initescqueue(void)
{
    escspeed = xQueueCreate(10, sizeof(Message));
}

void startESC(void)
{
    // Write HIGH to the latches first to prevent any transition drop (glitch)
    digitalWrite(ESC_SB_PWR_PIN, HIGH);
    digitalWrite(ESC_BB_PWR_PIN, HIGH);

    // Configure Power Pins
    pinMode(ESC_SB_PWR_PIN, OUTPUT);
    pinMode(ESC_BB_PWR_PIN, OUTPUT);
    
    Serial.println("ESCs Power Pins Driven HIGH");
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait half a second for ESCs to boot up
    
    // Attach Servos via ESP32Servo library (Handles LEDC timers automatically and safely)
    servoBB.setPeriodHertz(ESC_FREQ);
    servoSB.setPeriodHertz(ESC_FREQ);

    // ESP32Servo defaults to a 10-bit LEDC timer. Over a 20 ms frame that is 19.53 us per tick,
    // and usToTicks() truncates - so writeMicroseconds(1500) became 76 ticks and the ESC actually
    // received 1484 us. Every stop pulse this firmware has ever sent was 15.6 us below neutral,
    // on both channels. One ESC's deadband swallowed it; the other read it as a small command and
    // the thruster turned slowly for as long as it had power.
    //
    // It also made the neutral trim almost useless: the whole 1400..1600 range was about ten
    // distinct ticks, so most values changed nothing at all.
    //
    // 16 bits brings the error to 0.06 us and gives the trim ~327 real steps.
    //
    // AFTER attach, not before. The library's own comment says to call setTimerWidth() first, but
    // attach() does the opposite of what that promises: when pinNumber < 0 - a fresh object, or
    // any object that has been through detach() - it resets timer_width straight back to
    // DEFAULT_TIMER_WIDTH. Setting it first is silently undone, which is why the first attempt at
    // this fix changed nothing and the buoy still reported 1484 us. Called while attached, it
    // detaches and re-attaches the LEDC pin at the new width, which is what we actually want.
    servoBB.attach(ESC_BB_PIN, ESC_MIN_US, ESC_MAX_US);
    servoSB.attach(ESC_SB_PIN, ESC_MIN_US, ESC_MAX_US);
    servoBB.setTimerWidth(16);
    servoSB.setTimerWidth(16);
    
    // Each thruster gets ITS OWN stop pulse, not a shared 1500. An ESC whose neutral is calibrated
    // a little low reads a nominal 1500 as a small forward command and creeps for as long as it has
    // power - which is exactly what the starboard thruster was doing after every arming.
    servoBB.writeMicroseconds(speedToPulse(0, false, esc_neutral_bb));
    servoSB.writeMicroseconds(speedToPulse(0, false, esc_neutral_sb));

    // What the pin is REALLY doing, read back from the library rather than assumed. If these do
    // not match the trim above, the timer resolution is wrong again.
    udpLog("ESC arm: BB asked %d us got %d us | SB asked %d us got %d us",
           esc_neutral_bb, servoBB.readMicroseconds(),
           esc_neutral_sb, servoSB.readMicroseconds());

    // Keep neutral for 3 seconds to guarantee arming sequence completes
    vTaskDelay(pdMS_TO_TICKS(3000));
    Serial.printf("ESCs armed (neutral BB %d us, SB %d us)\r\n", esc_neutral_bb, esc_neutral_sb);
}

// What the pin is actually producing, read back from the library rather than assumed. The
// difference between this and the trim is pure timer resolution, and it is the thing that was
// quietly 15.6 us out - so it is worth being able to read it without a serial cable.
int escActualPulseBb(void) { return servoBB.attached() ? servoBB.readMicroseconds() : 0; }
int escActualPulseSb(void) { return servoSB.attached() ? servoSB.readMicroseconds() : 0; }

// ---------------------------------------------------------------------------------------------
//  Thruster cleaning sequence - see the block comment in esc.h
// ---------------------------------------------------------------------------------------------
// Mirrors EscTask's own esc_power_on, which is a local to that task and so invisible from here.
// Written at the three places that change it, read by the wake wait in cleanService().
//
// volatile because the writer and the reader are different tasks on different cores: EscTask sets
// it, the main loop spins waiting for it. Without this the compiler is entitled to hoist the read
// out of the wait and the wait would never see it change.
static volatile bool esc_power_state = false;

// Ask for the ESCs to be brought up, WITHOUT going through the speed queue.
//
// The queue cannot be relied on for this. EscTask wakes when it RECEIVES a non-zero speed, and
// escspeed is ten deep - while the buoy sits in IDLE the main loop pushes a zero into it on every
// pass of loop(), a few hundred times a second, so it stands permanently full of zeros. A wake that
// has to queue behind that backlog arrives late, or not at all when the send times out. That is a
// silly thing for "start the motors" to depend on, so it no longer does.
static volatile bool esc_wake_request = false;
void escRequestWake(void) { esc_wake_request = true; }

// One row per step: which WAY each thruster turns, and for how long. The magnitude is not in the
// table - see cleanPower below.
//
// CLEAN_BURST_MS is the 2 s the sequence is specified in. The stops between the bursts are not idle
// time - they exist because driving a spinning prop straight through zero into the other direction
// is what stalls an ESC, and a fouled prop is already close to stalling. So the blade is given a
// moment to slow first: 100 ms across the astern-to-ahead reversal, where it is only unloading, and
// 400 ms where a thruster is being stopped and left stopped while the other one works.
//
// Nine steps, about 9.7 s in total.
#define CLEAN_BURST_MS 2000
#define CLEAN_SETTLE_MS 400
#define CLEAN_PAUSE_MS 100

// How long to wait for the ESCs to come back up before giving up on the run. Generous next to the
// ~3.5 s startESC() actually takes, because CLEAN NOW may be pressed on a buoy that has been sat
// still long enough for EscTask to have cut their supply.
#define CLEAN_WAKE_TIMEOUT_MS 8000

struct CleanStep
{
    int bbDir; // -1 astern, 0 stopped, +1 ahead
    int sbDir;
    unsigned long ms;
    const char *what;
};

static const CleanStep cleanSteps[] = {
    {  0,  0, CLEAN_SETTLE_MS, "stop sailing" },
    { -1, -1, CLEAN_BURST_MS,  "both astern"  },
    {  0,  0, CLEAN_PAUSE_MS,  "pause"        },
    { +1, +1, CLEAN_BURST_MS,  "both ahead"   },
    {  0,  0, CLEAN_SETTLE_MS, "stop"         },
    { -1,  0, CLEAN_BURST_MS,  "BB astern"    },
    {  0,  0, CLEAN_SETTLE_MS, "BB stop"      },
    {  0, -1, CLEAN_BURST_MS,  "SB astern"    },
    {  0,  0, CLEAN_SETTLE_MS, "SB stop"      },
};
#define CLEAN_STEPS (int)(sizeof(cleanSteps) / sizeof(cleanSteps[0]))

// How hard the bursts push, in percent, captured when the run starts.
//
// maxSpeed, NOT full scale. The configured limit is the most this buoy is ever asked for while it
// is sailing - pidrudspeed.cpp clamps the station keeping output to exactly +/-maxSpeed - and a
// cleaning burst has no business exceeding what the hull, the mounts and the battery are set up
// for. On the boats as configured today that makes the bursts +15% and -15% rather than +/-100%.
//
// minSpeed is deliberately not used for the astern direction even though it reads like the natural
// pair. It is a different quantity - a floor on the drive the speed PID applies, not a ceiling on
// reverse - and its default is 0, so a buoy that had never had it set would run the astern half of
// this sequence at nothing at all and report a clean it had not performed.
//
// Captured once, at the start, so a SETUPDATA landing mid-run cannot change the thrust half way
// through a burst.
static int cleanPower = 0;

static bool cleanRunning = false;
static bool cleanWaking = false;
static int cleanStep = 0;
static unsigned long cleanStepEnd = 0;
static unsigned long cleanWakeDeadline = 0;

bool cleanActive(void) { return cleanRunning; }

void cleanStart(void)
{
    // Already running: a repeat of the command, which is expected - the Top resends it until the
    // Sub shows it landed. Restarting here would stretch one press into an endless wash cycle.
    if (cleanRunning) return;

    // Never over the top of a compass calibration. EscTask forces neutral for the whole run
    // (global_is_calibrating), so the bursts would not reach the water anyway, and the vibration
    // and current draw are exactly what that guard exists to keep out of the readings.
    extern bool global_is_calibrating;
    if (global_is_calibrating)
    {
        printf("CLEAN refused - a compass calibration is running\r\n");
        return;
    }

    // Refused rather than run at zero thrust. A buoy with maxSpeed 0 cannot move at all, so the
    // sequence would be ten seconds of stopped props reported as a completed clean - and "it says
    // CLEANING and nothing turns" is precisely the fault this whole path has already cost a
    // morning to. If it cannot do the job it says so instead.
    cleanPower = mainData.maxSpeed;
    if (cleanPower > 100) cleanPower = 100;
    if (cleanPower < 1)
    {
        printf("CLEAN refused - maxSpeed is %d, there is no thrust to clean with\r\n", cleanPower);
        udpLog("CLEAN refused - maxSpeed=%d", cleanPower);
        return;
    }

    cleanRunning = true;
    cleanStep = 0;
    // Wake first, time the steps afterwards. If the ESCs are asleep, EscTask blocks for ~3.5 s in
    // startESC() and writes neutral throughout - so a sequence that started its clock now would
    // spend its first two bursts commanding a thruster that was not listening yet.
    //
    // ALWAYS ask, and ALWAYS wait for the answer - never "only if the flag says they are asleep".
    // That test looks equivalent and is not: it trusts the flag in the one direction where being
    // wrong is invisible. If the flag says awake and they are not, the sequence drives ten seconds
    // of full-scale bursts into hardware with no power, reports that it cleaned, and nothing turns.
    // Waiting unconditionally costs nothing when they really are up - EscTask confirms it on the
    // next pass - and removes the only way this can fail silently.
    escRequestWake();
    cleanWaking = true;
    cleanWakeDeadline = millis() + CLEAN_WAKE_TIMEOUT_MS;
    cleanStepEnd = millis() + cleanSteps[0].ms;
    printf("CLEAN start: %d steps at %d%%, waiting for the ESCs\r\n", CLEAN_STEPS, cleanPower);
    udpLog("CLEAN start steps=%d power=%d%% escpwr=%d", CLEAN_STEPS, cleanPower, (int)esc_power_state);
}

void cleanAbort(void)
{
    if (!cleanRunning) return;
    cleanRunning = false;
    cleanWaking = false;
    printf("CLEAN aborted at step %d\r\n", cleanStep);
    udpLog("CLEAN aborted at step %d", cleanStep);
}

bool cleanService(int *speedBbOut, int *speedSbOut)
{
    if (!cleanRunning)
    {
        *speedBbOut = 0;
        *speedSbOut = 0;
        return false;
    }

    if (cleanWaking)
    {
        // Stopped while waiting, not already at the first burst. escRequestWake() is what brings
        // the supply back now, so there is nothing to gain by commanding thrust here - and plenty
        // to lose: in->speedBb is published in the telemetry, so asking for -100% while the ESCs
        // are still arming puts a full astern burst on every screen for three and a half seconds
        // while the props are not moving at all. Which is exactly the reading that made this
        // fault so hard to see from the outside.
        *speedBbOut = 0;
        *speedSbOut = 0;
        if (esc_power_state)
        {
            cleanWaking = false;
            cleanStepEnd = millis() + cleanSteps[0].ms;
            printf("CLEAN: ESCs are up, starting the sequence\r\n");
            udpLog("CLEAN ESCs up, running the sequence");
        }
        else if ((long)(millis() - cleanWakeDeadline) >= 0)
        {
            printf("CLEAN abandoned - the ESCs never came up\r\n");
            udpLog("CLEAN abandoned - ESCs never came up");
            cleanRunning = false;
            return false;
        }
        return true;
    }

    if ((long)(millis() - cleanStepEnd) >= 0)
    {
        cleanStep++;
        if (cleanStep >= CLEAN_STEPS)
        {
            cleanRunning = false;
            *speedBbOut = 0;
            *speedSbOut = 0;
            printf("CLEAN done\r\n");
            udpLog("CLEAN done");
            return false;
        }
        cleanStepEnd = millis() + cleanSteps[cleanStep].ms;
        printf("CLEAN step %d/%d: %s (bb %d%% sb %d%%)\r\n", cleanStep + 1, CLEAN_STEPS,
               cleanSteps[cleanStep].what, cleanSteps[cleanStep].bbDir * cleanPower,
               cleanSteps[cleanStep].sbDir * cleanPower);
    }

    *speedBbOut = cleanSteps[cleanStep].bbDir * cleanPower;
    *speedSbOut = cleanSteps[cleanStep].sbDir * cleanPower;
    return true;
}

void calculateLedColor(int speed, uint8_t& r, uint8_t& g) {
    if (speed > 0) { r = 0; g = map(speed, 0, 100, 0, 255); }
    else if (speed < 0) { r = map(speed, -100, 0, 255, 0); g = 0; }
    else { r = 0; g = 0; }
}

float global_speed_bb = 0;
float global_speed_sb = 0;
extern RoboStruct mainData;

void EscTask(void *arg)
{
    unsigned long offStamp = 0;
    unsigned long ledUpdateStamp = 0;
    int spsb = 0, spbb = 0;
    int spsbAct = 0, spbbAct = 0;
    bool esc_power_on = false;
    Message rcv_msg;
    
    // Allow allocation of all timers for ESP32Servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Wait until the gyroscope calibration has completely finished (icm_ready becomes true)
    // to prevent any ESC beeps, vibrations, or initialization currents from polluting the zero-rate gyro calibration!
    extern bool icm_ready;
    while (!icm_ready) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Before the first arming: the trim has to be in hand when the neutral pulse is first sent.
    memEscNeutral(&esc_neutral_bb, &esc_neutral_sb, MEM_GET);
    printf("ESC neutral trim: BB %d us, SB %d us\r\n", esc_neutral_bb, esc_neutral_sb);

    // Start with power enabled
    startESC();
    esc_power_on = true;
    esc_power_state = true;
    offStamp = millis() + 60000; // 60s initial grace period
    printf("ESC control task started.\r\n");
    
    extern bool global_is_calibrating;
    while (1)
    {
        if (global_is_calibrating) {
            // Trimmed neutral, not a bare 1500 - see speedToPulse(). A hard 1500 here was enough
            // to keep an off-centre ESC turning right through a compass calibration.
            servoBB.writeMicroseconds(speedToPulse(0, false, esc_neutral_bb));
            servoSB.writeMicroseconds(speedToPulse(0, false, esc_neutral_sb));
            spsb = 0; spbb = 0; spsbAct = 0; spbbAct = 0;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Check for new speed commands
        if (xQueueReceive(escspeed, (void *)&rcv_msg, 0) == pdTRUE)
        {
            spbb = rcv_msg.speedbb;
            spsb = rcv_msg.speedsb;
        }

        // Power Management Logic
        //
        // esc_wake_request is the second way in, for a caller that needs the ESCs up before it has
        // any speed to ask for - see escRequestWake(). It is cleared here, once, whether or not the
        // supply had to be switched: the request means "make sure they are up", and they now are.
        // The 30 s timer is pushed out either way, so the zero-speed steps of a cleaning sequence
        // cannot let the supply drop out from under the run.
        bool wakeAsked = esc_wake_request;
        if (wakeAsked) esc_wake_request = false;
        if (spsb != 0 || spbb != 0 || wakeAsked)
        {
            offStamp = millis() + 30000; // Reset 30s timer
            if (!esc_power_on)
            {
                printf("ESCs Waking Up from sleep.\r\n");
                startESC();
                esc_power_on = true;
                esc_power_state = true;
                spsbAct = 0; spbbAct = 0;
            }
            else
            {
                // Already up. The flag has to be set even so, because it is what the cleaning
                // sequence waits on and it would otherwise stay false from boot until the first
                // sleep/wake cycle - see the note in cleanStart().
                esc_power_state = true;
            }
        }
        else if (millis() > offStamp)
        {
            if (esc_power_on)
            {
                digitalWrite(ESC_SB_PWR_PIN, LOW);
                digitalWrite(ESC_BB_PWR_PIN, LOW);
                servoBB.detach(); // Free the PWM pins when powered down
                servoSB.detach();
                esc_power_on = false;
                esc_power_state = false;
                Serial.println("ESCs entered sleep mode (power pins LOW)");
            }
            spsb = 0; spbb = 0; spsbAct = 0; spbbAct = 0;
        }

        // Pulse Generation (No Ramping - Ramping moved to PID)
        if (millis() >= escStamp)
        {
            escStamp = millis() + 20;
            
            // Apply speed directly to actuators (Ramping is now handled in pidrudspeed.cpp)
            spsbAct = spsb;
            spbbAct = spbb;

            if (esc_power_on) {
                
                int s_sb = spsbAct;
                int s_bb = spbbAct;
                if (mainData.swap_BB_SB) {
                    s_sb = spbbAct;
                    s_bb = spsbAct;
                }
                servoSB.writeMicroseconds(speedToPulse(s_sb, mainData.revSB, esc_neutral_sb));
                servoBB.writeMicroseconds(speedToPulse(s_bb, mainData.revBB, esc_neutral_bb));
                global_speed_bb = s_bb;
                global_speed_sb = s_sb;
            } else {
                global_speed_bb = 0;
                global_speed_sb = 0;
            }
        }

        // Push telemetry to visual LED queue
        if (millis() >= ledUpdateStamp) {
            ledUpdateStamp = millis() + 100;
            powerIndicator.ledSb = (int)global_speed_sb;
            powerIndicator.ledBb = (int)global_speed_bb;
            powerIndicator.blinkBb = BLINK_OFF;
            powerIndicator.blinkSb = BLINK_OFF;
            xQueueOverwrite(ledPwr, (void *)&powerIndicator);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}