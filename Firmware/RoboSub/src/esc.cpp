/*
    Motor control for RoboBuoy Sub
*/
#include <Arduino.h>
#include <ESP32Servo.h>
#include "main.h"
#include "esc.h"
#include "io_sub.h"
#include "leds.h"

LedPwrtruct powerIndicator;
QueueHandle_t escspeed;

Servo servoBB;
Servo servoSB;

// ESC hardware parameters
#define ESC_FREQ 50       
#define ESC_MIN_US 1000   
#define ESC_MAX_US 2000   

unsigned long escStamp = 0;

/**
 * @brief Converts speed % to microsecond pulse (1000-2000us)
 */
uint16_t speedToPulse(int speed, bool invert)
{
    if (invert) speed = -speed;
    return (uint16_t)map(speed, -100, 100, ESC_MIN_US, ESC_MAX_US);
}

void triggerESC(void)
{
    Serial.println("Triggering ESC Test Sequence...");
    servoBB.writeMicroseconds(speedToPulse(10, false));
    servoSB.writeMicroseconds(speedToPulse(10, false));
    vTaskDelay(pdMS_TO_TICKS(1000));
    servoBB.writeMicroseconds(speedToPulse(0, false));
    servoSB.writeMicroseconds(speedToPulse(0, false));
}

void playTone(int frequency) { }
void beepESC(void) { startESC(); }

void initescqueue(void)
{
    escspeed = xQueueCreate(10, sizeof(Message));
}

void startESC(void)
{
    // Configure Power Pins
    pinMode(ESC_SB_PWR_PIN, OUTPUT);
    pinMode(ESC_BB_PWR_PIN, OUTPUT);
    
    // Explicitly drive power HIGH
    digitalWrite(ESC_SB_PWR_PIN, HIGH);
    digitalWrite(ESC_BB_PWR_PIN, HIGH);
    Serial.println("ESCs Power Pins Driven HIGH");
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait half a second for ESCs to boot up
    
    // Attach Servos via ESP32Servo library (Handles LEDC timers automatically and safely)
    servoBB.setPeriodHertz(ESC_FREQ);
    servoSB.setPeriodHertz(ESC_FREQ);
    
    // attach(pin, min, max)
    servoBB.attach(ESC_BB_PIN, ESC_MIN_US, ESC_MAX_US);
    servoSB.attach(ESC_SB_PIN, ESC_MIN_US, ESC_MAX_US);
    
    // Write 1500us neutral signal
    uint16_t neutral = speedToPulse(0, false);
    servoBB.writeMicroseconds(neutral);
    servoSB.writeMicroseconds(neutral);
    
    // Keep neutral for 3 seconds to guarantee arming sequence completes
    vTaskDelay(pdMS_TO_TICKS(3000));
    Serial.println("ESCs Armed (Neutral 1500us sent)");
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

    // Start with power enabled
    startESC();
    esc_power_on = true;
    offStamp = millis() + 60000; // 60s initial grace period
    printf("ESC control task started.\r\n");
    
    extern bool global_is_calibrating;
    while (1)
    {
        if (global_is_calibrating) {
            servoBB.writeMicroseconds(1500); // Force neutral thrusters during calibration for safety and efficiency
            servoSB.writeMicroseconds(1500);
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
                servoSB.writeMicroseconds(speedToPulse(s_sb, mainData.revSB));
                servoBB.writeMicroseconds(speedToPulse(s_bb, mainData.revBB));
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