#include <Arduino.h>
#include "buzzer.h"
#include "io.h"
#include "general.h"
QueueHandle_t Buzzerque; // speed que

MessageBuzz msgBuzz;
static unsigned long bstamp;
void BuzzerTask(void *arg)
{
    Buzzerque = xQueueCreate(10, sizeof(msgBuzz));
    while (1)
    {
        if (xQueueReceive(Buzzerque, (void *)&msgBuzz, 0) == pdTRUE)
        {
            do
            {
                bstamp = millis();
                while (bstamp + msgBuzz.time > millis())
                {
                    digitalWrite(BUZZERPIN, BUZZERON);
                    delay(1);
                    digitalWrite(BUZZERPIN, BUZZEROFF);
                    delay(1);
                }
                if(msgBuzz.pauze != 0){
                    vTaskDelay(pdMS_TO_TICKS(msgBuzz.pauze));
                }
            }while(msgBuzz.repeat-- != 0 );
        }
        vTaskDelay(1);
    }
}