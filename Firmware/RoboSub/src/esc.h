#ifndef ESC_H_
#define ESC_H_
#include "fastled.h"

extern QueueHandle_t escspeed;
typedef struct Message
{
    int speedbb;
    int speedsb;
} Message;

// Nominal ESC stop pulse, and how far either thruster's trim may sit from it. See memEscNeutral().
#define ESC_NEUTRAL_NOMINAL_US 1500
#define ESC_NEUTRAL_MIN_US     1400
#define ESC_NEUTRAL_MAX_US     1600

// Live trim, loaded from NVS at boot. subwifi.cpp writes these from /setparam.
extern int esc_neutral_bb;
extern int esc_neutral_sb;

int escActualPulseBb(void);
int escActualPulseSb(void);

void initescqueue(void);
void startESC(void);
void beepESC(void);
void triggerESC(void);
void EscTask(void *arg);

#endif /* ESC_H_ */
