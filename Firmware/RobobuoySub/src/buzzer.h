#ifndef BUZZER_H_
#define BUZZER_H_
#include <Arduino.h>
#include <RoboTone.h>

extern QueueHandle_t buzzer;
bool initbuzzerqueue(void);
void buzzerTask(void *arg);


#endif /* ESC_H_ */
