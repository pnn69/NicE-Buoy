#ifndef LEDS_H_
#define LEDS_H_
#define BRIGHTNES 0
#include <FastLED.h>
#include "../../RobobuoyDependency\RobobuoyMsg.h"

extern QueueHandle_t ledStatus; // speed status bb,sb
extern QueueHandle_t ledUtil;   // Accu status
extern QueueHandle_t ledGps;    // Accu status

struct LedData
{
    CRGB color; // collor
    int blink;  // mode
};

bool initledqueue(void);
void LedTask(void *arg);

#endif /* LEDS_H_ */
