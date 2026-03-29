#include <Arduino.h>
#include <RoboTone.h>
#include "buzzer.h"
#include "io_top.h"
const int squareWavePin = 2; // Pin 2 for square wave
const int pwmChannel = 4;    // PWM channel (0-15)
const int pwmResolution = 8; // 8-bit resolution (duty cycle values from 0 to 255)

QueueHandle_t buzzer;
static Buzz buzzerData;

void initbuzzerqueue(void)
{
    buzzer = xQueueCreate(50, sizeof(Buzz));
}

void setSquareWaveFrequency(int frequency)
{
    // Configure PWM channel with the desired frequency
    ledcSetup(pwmChannel, frequency, pwmResolution);
    // Attach the channel to the pin
    ledcAttachPin(BUZZER_PIN, pwmChannel);
    // Set a 50% duty cycle to create a square wave (128 for 50% in 8-bit resolution)
    ledcWrite(pwmChannel, 128);
}

void buzzerTask(void *arg)
{
    unsigned long nextActionTime = 0;
    int remainingRepeats = 0;
    bool isOn = false;

    while (true)
    {
        // Only pull a new beep command from the queue if the current sequence has finished
        if (remainingRepeats == 0)
        {
            if (xQueueReceive(buzzer, (void *)&buzzerData, 10) == pdTRUE)
            {
                if (buzzerData.hz == 0) buzzerData.hz = 1000;
                if (buzzerData.duration == 0) buzzerData.duration = 500;
                remainingRepeats = buzzerData.repeat + 1;
                nextActionTime = millis();
                isOn = false;
            }
        }

        if (remainingRepeats > 0 && millis() >= nextActionTime)
        {
            if (!isOn)
            {
                setSquareWaveFrequency(buzzerData.hz);
                nextActionTime = millis() + buzzerData.duration;
                isOn = true;
            }
            else
            {
                ledcDetachPin(BUZZER_PIN);
                pinMode(BUZZER_PIN, OUTPUT);
                digitalWrite(BUZZER_PIN, LOW); // Drive pin low to prevent DC static voltage
                nextActionTime = millis() + buzzerData.pause;
                isOn = false;
                remainingRepeats--;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
