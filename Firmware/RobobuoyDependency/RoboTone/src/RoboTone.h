#ifndef ROBOTONE_H
#define ROBOTONE_H
#include <Arduino.h>

typedef struct buzz
{
    unsigned int hz;
    unsigned int duration;
    unsigned int repeat;
    unsigned int pause;
} Buzz;


void beep(int sound , QueueHandle_t buzzer);

#endif
