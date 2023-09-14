#include <Arduino.h>
#include "buzzer.h"
#include "io.h"
QueueHandle_t Buzzerque; // speed que

MessageBuzz msgBuzz;

void BuzzerTask(void *arg)
{
    Buzzerque = xQueueCreate(10, sizeof(msgBuzz));
    while (1)
    {
        if (xQueueReceive(Buzzerque, (void *)&msgBuzz, 0) == pdTRUE)
        {
            for (int t = 0; t < msgBuzz.repeat; t++)
            {
                digitalWrite(BUZZERPIN, false);
                delay(1);
                digitalWrite(BUZZERPIN, true);
                delay(1);
            }
        }
        vTaskDelay(1);
    }
}