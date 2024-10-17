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
    buzzer = xQueueCreate(50, sizeof(Buzz));
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

// Input sequence for the first 20 notes of Chopin's "Funeral March" with rounded frequencies
Buzz melody[20] = {
    {117, 500, 100, 0}, // B♭2 (116.54 Hz rounded to 117)
    {117, 500, 100, 0}, // B♭2 (116.54 Hz rounded to 117)
    {117, 500, 100, 0}, // B♭2 (116.54 Hz rounded to 117)
    {98, 500, 100, 0},  // G2 (98.00 Hz rounded to 98)
    {117, 500, 100, 0}, // B♭2 (116.54 Hz rounded to 117)
    {147, 500, 100, 0}, // D3 (146.83 Hz rounded to 147)
    {117, 500, 100, 0}, // B♭2 (116.54 Hz rounded to 117)
    {175, 500, 100, 0}, // F3 (174.61 Hz rounded to 175)
    {117, 500, 100, 0}, // B♭2 (116.54 Hz rounded to 117)
    {117, 500, 100, 0}, // B♭2 (116.54 Hz rounded to 117)
    {117, 500, 100, 0}, // B♭2 (116.54 Hz rounded to 117)
    {98, 500, 100, 0},  // G2 (98.00 Hz rounded to 98)
    {117, 500, 100, 0}, // B♭2 (116.54 Hz rounded to 117)
    {147, 500, 100, 0}, // D3 (146.83 Hz rounded to 147)
    {117, 500, 100, 0}, // B♭2 (116.54 Hz rounded to 117)
    {175, 500, 100, 0}, // F3 (174.61 Hz rounded to 175)
    {117, 500, 100, 0}, // B♭2 (116.54 Hz rounded to 117)
    {185, 500, 100, 0}, // F♯3 (185.00 Hz rounded to 185)
    {117, 500, 100, 0}, // B♭2 (116.54 Hz rounded to 117)
    {175, 500, 100, 0}  // F3 (174.61 Hz rounded to 175)
};
// Input sequence for the first 20 notes of Chopin's "Funeral March"

Buzz tones[20] = {
    {50, 100, 100, 0},   // 0
    {100, 100, 100, 0},  // 1
    {500, 100, 100, 0},  // 2
    {1000, 100, 100, 0}, // 3
    {1500, 100, 100, 0}, // 4
    {2000, 100, 100, 0}, // 5
    {2500, 100, 100, 0}, // 6
    {3000, 100, 100, 0}, // 7
    {117, 100, 100, 0},  // 8
    {117, 100, 100, 0},  // 9
    {117, 100, 100, 0},  // 10
    {98, 100, 100, 0},   // 11
    {117, 100, 100, 0},  // 12
    {147, 100, 100, 0},  // 13
    {117, 100, 100, 0},  // 14
    {175, 100, 100, 0},  // 15
    {117, 100, 100, 0},  // 16
    {185, 100, 100, 0},  // 17
    {117, 100, 100, 0},  // 18
    {175, 100, 100, 0}   // 19
};

void beep(int sound)
{
    Buzz Data;
    if (sound == 1)
    {
        for (int i = 2; i < 5; i++)
        {
            xQueueSend(buzzer, (void *)&tones[i], 10);
        }
    }
    if (sound == 2)
    {
        for (int i = 5; i < 2; i--)
        {
            xQueueSend(buzzer, (void *)&tones[i], 10);
        }
    }
    if (sound == 500)
    {
        xQueueSend(buzzer, (void *)&tones[2], 10);
    }
    if (sound == 1000)
    {
        xQueueSend(buzzer, (void *)&tones[3], 10);
    }
    if (sound == 1500)
    {
        xQueueSend(buzzer, (void *)&tones[4], 10);
    }
    if (sound == -1)
    {
        for (int i = 0; i < 20; i++)
        {
            xQueueSend(buzzer, (void *)&melody[i], 10);
        }
    }
}