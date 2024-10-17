#ifndef BUZZER_H_
#define BUZZER_H_
#include <Arduino.h>

extern QueueHandle_t buzzer;
typedef struct buzz
{
    unsigned int hz;
    unsigned int duration;
    unsigned int repeat;
    unsigned int pause;
} Buzz;
bool initbuzzerqueue(void);
void buzzerTask(void *arg);
void beep(int sound);

#endif /* ESC_H_ */
