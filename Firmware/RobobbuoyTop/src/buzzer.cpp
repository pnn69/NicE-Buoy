#include <Arduino.h>
#include "buzzer.h"
#include "io_top.h"
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
    ledcAttachPin(BUZZERPIN, pwmChannel);
    // Set a 50% duty cycle to create a square wave (128 for 50% in 8-bit resolution)
    ledcWrite(pwmChannel, 128);
}

void buzzerTask(void *arg)
{
    unsigned long timeStamp;
    unsigned int repeat, delay, duration;
    unsigned int hz;
    bool buzzOn;

    while (true)
    {
        if (repeat == 0)
        {
            if ((xQueueReceive(buzzer, (void *)&buzzerData, 0) == pdTRUE))
            {
                if (buzzerData.hz > 0)
                {
                    setSquareWaveFrequency(buzzerData.hz);
                    if (buzzerData.duration == 0 || buzzerData.duration > 5000)
                    {
                        duration = 500;
                    }
                    else
                    {
                        duration = buzzerData.duration;
                    }
                    timeStamp = millis() + duration;
                    hz = buzzerData.hz;
                    repeat = buzzerData.repeat;
                    delay = buzzerData.pause;
                    buzzOn = true;
                }
                else
                {
                    ledcDetachPin(BUZZERPIN);
                }
            }
        }
        if (timeStamp < millis() && (repeat > 0 || buzzOn == true))
        {
            if (buzzOn == true)
            {
                ledcDetachPin(BUZZERPIN);
                timeStamp = millis() + delay;
                buzzOn = false;
                if (repeat != 0)
                {
                    repeat--;
                }
            }
            else
            {
                setSquareWaveFrequency(hz);
                timeStamp = millis() + duration;
                buzzOn = true;
            }
            if (repeat == 0)
            {
                ledcDetachPin(BUZZERPIN);
                buzzOn = false;
            }
        }
    }
    vTaskDelay(1);
}
