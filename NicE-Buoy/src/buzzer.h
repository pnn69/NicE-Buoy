#ifndef BUZZER_H_
#define BUZZER_H_

extern QueueHandle_t Buzzerque; // speed que

typedef struct BMessage
{
    bool on;
    int time;
    int pauze;
    int repeat;
} MessageBuzz;

void BuzzerTask(void *arg);

#endif /* BUZZER_H_ */
