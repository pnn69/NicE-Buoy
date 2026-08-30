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
        if (spsb != 0 || spbb != 0)
        {
            offStamp = millis() + 30000; // Reset 30s timer
            if (!esc_power_on)
            {
                printf("ESCs Waking Up from sleep.\r\n");
                startESC();
                esc_power_on = true;
                spsbAct = 0; spbbAct = 0;
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