#include <Arduino.h>
#include <RoboTone.h>
#include "buzzer.h"
#include "io_sub.h"
const int squareWavePin = 2; // Pin 2 for square wave
const int pwmChannel = 4;    // PWM channel (0-15)
const int pwmResolution = 8; // 8-bit resolution (duty cycle values from 0 to 255)

QueueHandle_t buzzer;
static Buzz buzzerData;

/**
 * @brief Initializes the FreeRTOS queue for buzzer commands.
 * 
 * @return true if the queue was successfully created, false otherwise.
 */
bool initbuzzerqueue()
{
    buzzer = xQueueCreate(10, sizeof(Buzz));
    if (buzzer == NULL)
    {
        printf("Queue buzzer could not be created. %p\r\n", buzzer);
        return false;
    }
    else
    {
        Serial.println("Queue buzzer created.");
    }
    return true;
}

/**
 * @brief Configures and starts a square wave on the buzzer pin.
 * Uses the LEDC peripheral to generate a PWM signal with a 50% duty cycle.
 * 
 * @param frequency The desired frequency of the square wave in Hz.
 */
void setSquareWaveFrequency(int frequency)
{
    // Configure PWM channel with the desired frequency
    ledcSetup(pwmChannel, frequency, pwmResolution);
    // Attach the channel to the pin
    ledcAttachPin(BUZZER_PIN, pwmChannel);
    // Set a 50% duty cycle to create a square wave (128 for 50% in 8-bit resolution)
    ledcWrite(pwmChannel, 128);
}

/**
 * @brief FreeRTOS task that manages the buzzer's beeping sequences.
 * 1. Monitors the buzzer queue for new beep commands (frequency, duration, repeat, pause).
 * 2. Cycles through the specified number of repeats.
 * 3. Handles timing for the 'on' duration and 'off' pause using non-blocking delays.
 * 4. Ensures the buzzer pin is driven LOW during pauses to avoid DC offset issues.
 * 
 * @param arg Unused FreeRTOS task argument.
 */
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
// if ((xQueueReceive(buzzer, (void *)&buzzerData, 0) == pdTRUE))
// {
//     if (buzzerData.hz == 0)
//         buzzerData.hz = 1000;

//     if (buzzerData.duration == 0)
//         buzzerData.duration = 500;

//     if (buzzerData.pause == 0)
//         buzzerData.pause = 100;

//     for (int i = 0; i < buzzerData.repeat; i++)
//     {
//         setSquareWaveFrequency(buzzerData.hz); // Start buzzer
//         vTaskDelay(pdMS_TO_TICKS(buzzerData.duration));

//         ledcDetachPin(BUZZER_PIN); // Stop buzzer
//         vTaskDelay(pdMS_TO_TICKS(buzzerData.pause));
//     }
// }
