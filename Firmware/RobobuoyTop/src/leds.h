#ifndef LEDS_H_
#define LEDS_H_
#define BRIGHTNES 0
#include <FastLED.h>

extern QueueHandle_t ledStatus; // speed status bb,sb
extern QueueHandle_t ledUtil;   // Accu status
extern QueueHandle_t ledGps;    // Accu status
extern QueueHandle_t ledPwr;    // speed status bb,sb

struct LedData
{
    CRGB color;         // collor
    int blink;          // mode
    int fadeAmount = 5; //
    int brightness = 0;
};

typedef struct LedPwrtruct
{
    CRGB bb;     // collor
    CRGB sb;     // collor
    int blinkBb; // mode
    int blinkSb; // mode
    int ledBb;
    int ledSb;
} PwrData;

bool initledqueue(void);
void LedTask(void *arg);

#endif /* LEDS_H_ */
