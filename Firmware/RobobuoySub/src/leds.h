#ifndef LEDS_H_
#define LEDS_H_
#include <FastLED.h>

extern QueueHandle_t ledsSpeed;   // speed status bb,sb
extern QueueHandle_t ledAccuVoltage; // Accu status

typedef struct Speeddef
{
    int speedbb;
    int speedsb;
} SpeedMessage;

typedef struct Accudef
{
    float accuvoltage;
} AccuMessage;
void initLedTask(void);
void LedTask(void *arg);

#endif /* LEDS_H_ */
