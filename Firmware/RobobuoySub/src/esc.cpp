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

unsigned long escStamp = 0;

//************************************************************************************
// Map microseconds to duty cycle (for 50Hz and 16-bit resolution)
//************************************************************************************
uint32_t microsecondsToDuty(uint16_t us)
{
    // 50 Hz = 20,000 us period → 65535 max duty
    return map(us, 0, 20000, 0, 65535);
}
//************************************************************************************
//* ESC parameters
//************************************************************************************
void writeESC(uint8_t channel, uint16_t pulse_us)
{
    pulse_us = constrain(pulse_us, ESC_MIN_US, ESC_MAX_US);
    uint32_t duty = microsecondsToDuty(pulse_us);
    ledcWrite(channel, duty);
}
//************************************************************************************
//* ESC speedToPulse
//************************************************************************************
uint16_t speedToPulse(int speed)
{
    // Convert speed (-100 to 100) to microsecond pulse (1000 to 2000)
    int pulse = map(speed, -100, 100, 1000, 2000);
    // Then convert microseconds to 16-bit duty cycle (5%-10% of 20ms)
    return map(pulse, 1000, 2000, 3276, 6553);
}

void triggerESC(void)
{
    unsigned long now;
    Serial.println("Trigger ESC");
    ledcWrite(ESC_BB_CHANNEL, speedToPulse(5));
    ledcWrite(ESC_SB_CHANNEL, speedToPulse(5));
    now = millis();
    while (millis() < now + ESC_ARM_TIME)
        ;
    ledcWrite(ESC_BB_CHANNEL, speedToPulse(0));
    ledcWrite(ESC_SB_CHANNEL, speedToPulse(0));
    now = millis();
    while (millis() < now + ESC_ARM_TIME)
        ;
    ledcWrite(ESC_BB_CHANNEL, speedToPulse(-5));
    ledcWrite(ESC_SB_CHANNEL, speedToPulse(-5));
    now = millis();
    while (millis() < now + ESC_ARM_TIME)
        ;
    ledcWrite(ESC_BB_CHANNEL, speedToPulse(0));
    ledcWrite(ESC_SB_CHANNEL, speedToPulse(0));
    now = millis();
    while (millis() < now + ESC_ARM_TIME)
        ;
}

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

void beepESC(void)
{
    Serial.println("Beep ESC");
    playTone(880); // A5    delay(500);
    delay(1000);   // Wait for 1 second
    playTone(660); // E5
    delay(1000);   // Wait for 1 second
    ledcSetup(ESC_BB_CHANNEL, ESC_FREQ, ESC_RESOLUTION);
    ledcSetup(ESC_SB_CHANNEL, ESC_FREQ, ESC_RESOLUTION);
    ledcAttachPin(ESC_BB_PIN, ESC_BB_CHANNEL); // e.g., GPIO 25
    ledcAttachPin(ESC_SB_PIN, ESC_SB_CHANNEL); // e.g., GPIO 26
}

//***************************************************************************************************
//* ESC queue
//***************************************************************************************************
void initescqueue(void)
{
    escspeed = xQueueCreate(10, sizeof(Message));
}
//***************************************************************************************************
//* ESC init
//***************************************************************************************************
void startESC(void)
{
    digitalWrite(ESC_SB_PWR_PIN, HIGH);
    Serial.println("ESC BB ON");
    delay(500);
    digitalWrite(ESC_BB_PWR_PIN, HIGH);
    Serial.println("ESC SB ON");
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
        delay(20);
    }
    for (int i = 0; i < 2; i++)
    {
        neutral = speedToPulse(i);
        ledcWrite(ESC_BB_CHANNEL, neutral);
        ledcWrite(ESC_SB_CHANNEL, neutral);
        delay(50);
    }
    for (int i = 2; i > -2; i--)
    {
        neutral = speedToPulse(i);
        ledcWrite(ESC_BB_CHANNEL, neutral);
        ledcWrite(ESC_SB_CHANNEL, neutral);
        delay(50);
    }
    for (int i = -2; i < 0; i++)
    {
        neutral = speedToPulse(i);
        ledcWrite(ESC_BB_CHANNEL, neutral);
        ledcWrite(ESC_SB_CHANNEL, neutral);
        delay(50);
    }
}

//***************************************************************************************************
//* ESC task
//***************************************************************************************************
void EscTask(void *arg)
{
    unsigned long logStamp = 0;
    unsigned long offStamp = 0;
    int spsb = 0, spbb = 0;
    int spsbAct = 0, spbbAct = 0;
    Message rcv_msg;
    startESC();
    printf("ESC task running!\r\n");
    while (1)
    {
        if (xQueueReceive(escspeed, (void *)&rcv_msg, 0) == pdTRUE)
        {
            spbb = -rcv_msg.speedbb;
            spsb = -rcv_msg.speedsb;
            // powerIndicator.ledBb = rcv_msg.speedbb;
            // powerIndicator.ledSb = rcv_msg.speedsb;
            uint8_t r, g;

            if (rcv_msg.speedbb < 0)
            {
                r = map(rcv_msg.speedbb, -100, 0, 255, 0);
                g = 0;
            }
            else
            {
                r = 0;
                g = map(rcv_msg.speedbb, 100, 0, 255, 0);
            }
            powerIndicator.bb[0] = r;
            powerIndicator.bb[1] = g;
            powerIndicator.bb[2] = 0;
            powerIndicator.blinkBb = BLINK_OFF;
            if (rcv_msg.speedsb < 0)
            {
                r = map(rcv_msg.speedsb, -100, 0, 255, 0);
                g = 0;
            }
            else
            {
                r = 0;
                g = map(rcv_msg.speedsb, 100, 0, 255, 0);
            }
            powerIndicator.sb[0] = r;
            powerIndicator.sb[1] = g;
            powerIndicator.sb[2] = 0;
            powerIndicator.blinkSb = BLINK_OFF;
        }
        if (spsb != 0 || spbb != 0)
        {
            // offStamp = millis() + 1000 * 60 * 5; // 5 minutes
            offStamp = millis() + 1000 * 30; // 30 seconds
            if (digitalRead(ESC_SB_PWR_PIN) == 0 || digitalRead(ESC_BB_PWR_PIN) == 0)
            {
                startESC();
                printf("ESC'S  ON\r\n");
                spsbAct = 0;
                spbbAct = 0;
            }
        }
        if (offStamp < millis())
        {
            if (digitalRead(ESC_SB_PWR_PIN) == HIGH || digitalRead(ESC_BB_PWR_PIN) == HIGH)
            {
                printf("ESC'S  OFF\r\n");
                digitalWrite(ESC_SB_PWR_PIN, LOW);
                digitalWrite(ESC_BB_PWR_PIN, LOW);
            }
            spsb = 0;
            spbb = 0;
            spsbAct = 0;
            spbbAct = 0;
        }
        if (logStamp + 100 < millis())
        {
            logStamp = millis();
            // printf("ESC bb=%03d %03d    sb=%03d %03d\r\n", spbb, spbbAct, spsb, spsbAct);
        }
        if (escStamp < millis())
        {
            // escStamp = millis() + 10;
            escStamp = millis();
            xQueueSend(ledPwr, (void *)&powerIndicator, 0);
            if (spsb > spsbAct)
            {

                spsbAct++;
            }
            else
            {
                spsbAct--;
            }
            powerIndicator.ledSb = -spsbAct;
            ledcWrite(ESC_SB_CHANNEL, speedToPulse(spsbAct));
            if (spbb > spbbAct)
            {
                spbbAct++;
            }
            else
            {
                spbbAct--;
            }
            powerIndicator.ledBb = -spbbAct;
            ledcWrite(ESC_BB_CHANNEL, speedToPulse(spbbAct));
        }
        delay(1);
    }
}