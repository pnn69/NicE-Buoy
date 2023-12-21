#ifndef BUZZER_H_
#define BUZZER_H_

extern QueueHandle_t Buzzerque; // speed que

typedef struct BMessage
{
    bool on;
    unsigned int time;
    unsigned int pauze;
    unsigned int repeat;
} MessageBuzz;

void BuzzerTask(void *arg);

#endif /* BUZZER_H_ */
