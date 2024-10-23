#ifndef LEDS_H_
#define LEDS_H_
#define BRIGHTNES 0
#include <FastLED.h>
#include "../../RobobuoyDependency\RobobuoyMsg.h"

extern QueueHandle_t ledPwr; // speed status bb,sb
extern QueueHandle_t ledStatus;   // Accu status

typedef struct LedDataStruct
{
    CRGB color;    // collor
    int blink;    // mode
} LedData;

typedef struct LedPwrtruct
{
    CRGB bb;    // collor
    CRGB sb;    // collor
    int blinkBb;    // mode
    int blinkSb;    // mode
} PwrData;

bool initledqueue(void);
void LedTask(void *arg);

#endif /* LEDS_H_ */
