/*
https://github.com/madhephaestus/ESP32Servo
https://dronebotworkshop.com/esp32-servo/
 */
#include <Arduino.h>
#include "main.h"
#include "esc.h"
#include "io_sub.h"
#include "leds.h"
// cannels for esc

LedPwrtruct powerIndicator;
QueueHandle_t escspeed;
//  ESC parameters
#define ESC_FREQ 50       // 50 Hz PWM for standard ESCs
#define ESC_RESOLUTION 16 // 16-bit resolution for fine control
#define ESC_MIN_US 1000   // 1000 microseconds (1 ms pulse)
#define ESC_MAX_US 2000   // 2000 microseconds (2 ms pulse)
// ESC PWM channel definitions
#define ESC_BB_CHANNEL 0
#define ESC_SB_CHANNEL 1
#define ESC_RES 16
#define ESC_ARM_TIME 200 // 2 seconds

// Heartbeat monitoring
unsigned long escStamp = 0;

//************************************************************************************
// Map microseconds to duty cycle (for 50Hz and 16-bit resolution)
//************************************************************************************
/**
 * @brief Converts a pulse width in microseconds to a 16-bit PWM duty cycle.
 * Assumes a 50Hz PWM frequency (20,000us period) and 16-bit resolution (0-65535).
 * 
 * @param us The desired pulse width in microseconds (e.g., 1000 to 2000).
 * @return uint32_t The corresponding 16-bit duty cycle value.
 */
uint32_t microsecondsToDuty(uint16_t us)
{
    // 50 Hz = 20,000 us period → 65535 max duty
    return map(us, 0, 20000, 0, 65535);
}

/**
 * @brief Writes a specific microsecond pulse width to an ESC channel.
 * Constrains the input to the safe [1000, 2000] us range before converting to duty cycle.
 * 
 * @param channel The LEDC PWM channel (e.g., ESC_BB_CHANNEL).
 * @param pulse_us The desired pulse width in microseconds.
 */
void writeESC(uint8_t channel, uint16_t pulse_us)
{
    pulse_us = constrain(pulse_us, ESC_MIN_US, ESC_MAX_US);
    uint32_t duty = microsecondsToDuty(pulse_us);
    ledcWrite(channel, duty);
}

/**
 * @brief Converts a percentage speed (-100 to 100) to a 16-bit PWM duty cycle directly.
 * Maps speed to pulse width (1000-2000us), then pulse width to duty cycle (3276-6553).
 * 
 * @param speed Target motor speed as a percentage (-100 to 100).
 * @return uint16_t The calculated 16-bit duty cycle.
 */
uint16_t speedToPulse(int speed)
{
    // Convert speed (-100 to 100) to microsecond pulse (1000 to 2000)
    int pulse = map(speed, -100, 100, 1000, 2000);
    // Then convert microseconds to 16-bit duty cycle (5%-10% of 20ms)
    return map(pulse, 1000, 2000, 3276, 6553);
}

/**
 * @brief Test sequence to trigger the ESCs (Forward/Neutral/Reverse/Neutral).
 * Used for debugging or forced arming sequences.
 */
void triggerESC(void)
{
    Serial.println("Trigger ESC");
    ledcWrite(ESC_BB_CHANNEL, speedToPulse(5));
    ledcWrite(ESC_SB_CHANNEL, speedToPulse(5));
    vTaskDelay(pdMS_TO_TICKS(ESC_ARM_TIME));
    
    ledcWrite(ESC_BB_CHANNEL, speedToPulse(0));
    ledcWrite(ESC_SB_CHANNEL, speedToPulse(0));
    vTaskDelay(pdMS_TO_TICKS(ESC_ARM_TIME));
    
    ledcWrite(ESC_BB_CHANNEL, speedToPulse(-5));
    ledcWrite(ESC_SB_CHANNEL, speedToPulse(-5));
    vTaskDelay(pdMS_TO_TICKS(ESC_ARM_TIME));
    
    ledcWrite(ESC_BB_CHANNEL, speedToPulse(0));
    ledcWrite(ESC_SB_CHANNEL, speedToPulse(0));
    vTaskDelay(pdMS_TO_TICKS(ESC_ARM_TIME));
}

/**
 * @brief Attempts to play a tone through the ESC motors (brushless singing).
 * Modulates the PWM signal around the neutral point to create audible vibrations in the motor.
 * 
 * @param frequency The desired tone frequency (currently fixed modulation in logic).
 */
void playTone(int frequency)
{
    if (frequency == 0)
    {
        ledcWrite(0, microsecondsToDuty(1000)); // Send 1000us = neutral
        return;
    }

    int toneWidth = 1500 + (sin(millis() / 100.0) * 200); // Optionally modulate

    ledcWrite(0, microsecondsToDuty(toneWidth)); // Slightly modulate PWM to trick ESC
}

/**
 * @brief Plays an initialization beep sequence through the ESCs.
 * Restores normal PWM operation on the channels afterward.
 */
void beepESC(void)
{
    Serial.println("Beep ESC");
    playTone(880); // A5    delay(500);
    vTaskDelay(pdMS_TO_TICKS(1000));   // Wait for 1 second
    playTone(660); // E5
    vTaskDelay(pdMS_TO_TICKS(1000));   // Wait for 1 second
    ledcSetup(ESC_BB_CHANNEL, ESC_FREQ, ESC_RESOLUTION);
    ledcSetup(ESC_SB_CHANNEL, ESC_FREQ, ESC_RESOLUTION);
    ledcAttachPin(ESC_BB_PIN, ESC_BB_CHANNEL); // e.g., GPIO 25
    ledcAttachPin(ESC_SB_PIN, ESC_SB_CHANNEL); // e.g., GPIO 26
}

//***************************************************************************************************
//* ESC queue
//***************************************************************************************************
/**
 * @brief Initializes the FreeRTOS queue for receiving ESC speed commands.
 */
void initescqueue(void)
{
    escspeed = xQueueCreate(10, sizeof(Message));
}

//***************************************************************************************************
//* ESC init
//***************************************************************************************************
/**
 * @brief Powers on the ESCs and sends a neutral signal to arm them.
 * 1. Switches on the power control pins for the Starboard and Port ESCs.
 * 2. Configures the LEDC PWM hardware.
 * 3. Sends a neutral (0 speed) pulse for 2 seconds to allow ESC initialization/arming.
 */
void startESC(void)
{
    digitalWrite(ESC_SB_PWR_PIN, HIGH);
    Serial.println("ESC BB ON");
    vTaskDelay(pdMS_TO_TICKS(250));
    digitalWrite(ESC_BB_PWR_PIN, HIGH);
    Serial.println("ESC SB ON");
    vTaskDelay(pdMS_TO_TICKS(250));
    // Set up PWM channels
    ledcSetup(ESC_BB_CHANNEL, ESC_FREQ, ESC_RESOLUTION);
    ledcSetup(ESC_SB_CHANNEL, ESC_FREQ, ESC_RESOLUTION);
    ledcAttachPin(ESC_BB_PIN, ESC_BB_CHANNEL); // e.g., GPIO 25
    ledcAttachPin(ESC_SB_PIN, ESC_SB_CHANNEL); // e.g., GPIO 26
    uint16_t neutral = speedToPulse(0);
    for (int i = 0; i < 100; i++)
    { // 2 seconds at 20ms
        ledcWrite(ESC_BB_CHANNEL, neutral);
        ledcWrite(ESC_SB_CHANNEL, neutral);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

//***************************************************************************************************
//* Calculate LED color based on speed
//***************************************************************************************************
/**
 * @brief Calculates the RGB LED color representation based on motor speed.
 * Green for forward (negative speed), Red for reverse (positive speed).
 * 
 * @param speed Current motor speed (-100 to 100).
 * @param r Output parameter for the red color component.
 * @param g Output parameter for the green color component.
 */
void calculateLedColor(int speed, uint8_t& r, uint8_t& g) {
    if (speed < 0) {
        // Forward: Green
        r = 0;
        g = map(speed, -100, 0, 255, 0);
    } else if (speed > 0) {
        // Reverse: Red
        r = map(speed, 0, 100, 0, 255);
        g = 0;
    } else {
        r = 0;
        g = 0;
    }
}

//***************************************************************************************************
//* ESC task
//***************************************************************************************************
/**
 * @brief FreeRTOS task managing motor control, smooth ramping, and power saving.
 * 1. Monitors the `escspeed` queue for new target speeds.
 * 2. Implements a smooth ramping algorithm (updates actual speed by 1% every 20ms) to prevent current spikes.
 * 3. Handles power management: Automatically powers off ESCs after 30 seconds of 0 speed,
 *    and seamlessly re-arms them when new commands arrive.
 * 4. Pushes real-time speed data and LED color states to the `ledPwr` queue for visual feedback.
 * 
 * @param arg Unused FreeRTOS task argument.
 */
void EscTask(void *arg)
{
    unsigned long offStamp = 0;
    unsigned long ledUpdateStamp = 0;
    int spsb = 0, spbb = 0;
    int spsbAct = 0, spbbAct = 0;
    bool ledChanged = false;
    Message rcv_msg;
    startESC();
    printf("ESC task running!");
    while (1)
    {
        if (xQueueReceive(escspeed, (void *)&rcv_msg, 0) == pdTRUE)
        {
            spbb = -rcv_msg.speedbb;
            spsb = -rcv_msg.speedsb;
            ledChanged = true;
        }

        if (spsb != 0 || spbb != 0)
        {
            offStamp = millis() + 1000 * 30; // 30 seconds
            if (digitalRead(ESC_SB_PWR_PIN) == 0 || digitalRead(ESC_BB_PWR_PIN) == 0)
            {
                startESC();
                printf("ESC'S  ON");
                spsbAct = 0;
                spbbAct = 0;
            }
        }
        else if (millis() > offStamp)
        {
            if (digitalRead(ESC_SB_PWR_PIN) == HIGH || digitalRead(ESC_BB_PWR_PIN) == HIGH)
            {
                printf("ESC'S  OFF");
                digitalWrite(ESC_SB_PWR_PIN, LOW);
                digitalWrite(ESC_BB_PWR_PIN, LOW);
            }
            spsb = 0; spbb = 0; spsbAct = 0; spbbAct = 0;
        }

        // Ramping logic (every 20ms for smooth transition)
        if (millis() >= escStamp)
        {
            escStamp = millis() + 20;
            bool changed = false;

            if (spsb > spsbAct) { spsbAct++; changed = true; }
            else if (spsb < spsbAct) { spsbAct--; changed = true; }

            if (spbb > spbbAct) { spbbAct++; changed = true; }
            else if (spbb < spbbAct) { spbbAct--; changed = true; }

            if (changed) {
                ledcWrite(ESC_SB_CHANNEL, speedToPulse(spsbAct));
                ledcWrite(ESC_BB_CHANNEL, speedToPulse(spbbAct));
                ledChanged = true;
            }
        }

        // Update LED data and send to queue only when changed
        if (ledChanged && millis() >= ledUpdateStamp) {
            ledUpdateStamp = millis() + 100;
            
            // Update BB LED color based on actual speed
            uint8_t r, g;
            calculateLedColor(spbbAct, r, g);
            powerIndicator.bb[0] = r;
            powerIndicator.bb[1] = g;
            powerIndicator.bb[2] = 0;
            powerIndicator.blinkBb = BLINK_OFF;

            // Update SB LED color based on actual speed
            calculateLedColor(spsbAct, r, g);
            powerIndicator.sb[0] = r;
            powerIndicator.sb[1] = g;
            powerIndicator.sb[2] = 0;
            powerIndicator.blinkSb = BLINK_OFF;
            
            // Update bar graph values
            powerIndicator.ledSb = spsbAct;
            powerIndicator.ledBb = spbbAct;
            
            xQueueOverwrite(ledPwr, (void *)&powerIndicator);
            ledChanged = false;  // Clear flag after successful update
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
