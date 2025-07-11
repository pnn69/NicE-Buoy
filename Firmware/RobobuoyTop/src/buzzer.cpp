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
    unsigned long timeStamp;

    while (true)
    {
        if ((xQueueReceive(buzzer, (void *)&buzzerData, 0) == pdTRUE))
        {
            if (buzzerData.hz == 0)
            {
                buzzerData.hz = 1000;
            }
            if (buzzerData.duration == 0)
            {
                buzzerData.duration = 500;
            }
            buzzerData.repeat++;
            timeStamp = millis() + buzzerData.duration;
            while (buzzerData.repeat--)
            {
                setSquareWaveFrequency(buzzerData.hz);
                vTaskDelay(pdMS_TO_TICKS(buzzerData.duration));
                ledcDetachPin(BUZZER_PIN);
                vTaskDelay(pdMS_TO_TICKS(buzzerData.pause));
            }
        }
    }
    vTaskDelay(1);
}
