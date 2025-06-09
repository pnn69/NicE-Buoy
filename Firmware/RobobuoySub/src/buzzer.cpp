#include <Arduino.h>
#include <RoboTone.h>
#include "buzzer.h"
#include "io_sub.h"
const int squareWavePin = 2; // Pin 2 for square wave
const int pwmChannel = 0;    // PWM channel (0-15)
const int pwmResolution = 8; // 8-bit resolution (duty cycle values from 0 to 255)

QueueHandle_t buzzer;
static Buzz buzzerData;

bool initbuzzerqueue()
{
    buzzer = xQueueCreate(10, sizeof(Buzz));
    if (buzzer == NULL)
    {
        printf("Queue buzzer could not be created. %p\\r\n", buzzer);
        return false;
    }
    else
    {
        Serial.println("Queue buzzer created.");
    }
    return true;
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
                timeStamp = millis() + buzzerData.duration;
                while (timeStamp > millis())
                    ;
                ledcDetachPin(BUZZER_PIN);
                timeStamp = millis() + buzzerData.pause;
                while (timeStamp > millis())
                    ;
            }
        }
    }
    vTaskDelay(1);
}
