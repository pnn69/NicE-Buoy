/*
https://github.com/madhephaestus/ESP32Servo
https://dronebotworkshop.com/esp32-servo/
 */
#include <Arduino.h>
#include <ESP32Servo.h>
#include "esc.h"
#include "io_sub.h"
// cannels for esc

QueueHandle_t escspeed;
#define ESC_ARM_TIME 10
#define ESC_MAX 2000 // 2000
#define ESC_MIN 1000 // 1000

Servo escbb; // create servo object to control a servo
Servo escsb; // create servo object to control a servo
// value = map(value, 0, 180, 1000, 2000);

void triggerESC(void)
{
    unsigned long now;
    Serial.println("Trigger ESC");
    escbb.write(map(5, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    escsb.write(map(5, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    now = millis();
    while (millis() < now + ESC_ARM_TIME)
        ;
    escbb.write(map(0, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'W
    escsb.write(map(0, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    now = millis();
    while (millis() < now + ESC_ARM_TIME)
        ;
    escbb.write(map(-5, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    escsb.write(map(-5, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    now = millis();
    while (millis() < now + ESC_ARM_TIME)
        ;
    escbb.write(map(0, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    escsb.write(map(0, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    now = millis();
    while (millis() < now + ESC_ARM_TIME)
        ;
    escbb.write(map(0, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    escsb.write(map(0, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
}
void beepESC(void)
{
    Serial.println("Beep ESC");
    escbb.setPeriodHertz(0); // standard 400 hz servo
    escsb.setPeriodHertz(0); // standard 400 hz servo
    delay(500);
    escbb.setPeriodHertz(100); // standard 400 hz servo
    escsb.setPeriodHertz(100); // standard 400 hz servo
}

void initescqueue(void)
{
    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    escbb.setPeriodHertz(100);                  // standard 400 hz servo
    escbb.attach(ESC_BB_PIN, ESC_MIN, ESC_MAX); // attaches the servo on pin 18 to the servo object
    escsb.setPeriodHertz(100);                  // standard 400 hz servo
    escsb.attach(ESC_SB_PIN, ESC_MIN, ESC_MAX); // attaches the servo on pin 18 to the servo object
    escspeed = xQueueCreate(10, sizeof(Message));
}

void EscTask(void *arg)
{
    unsigned long sbStamp = 0;
    unsigned long bbStamp = 0;
    int spsb = 0, spbb = 0;
    pinMode(ESC_SB_PWR_PIN, OUTPUT);
    pinMode(ESC_BB_PWR_PIN, OUTPUT);
    escbb.write(map(0, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    escsb.write(map(0, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    delay(500);
    void beepESC();
    Message rcv_msg;
    digitalWrite(ESC_SB_PWR_PIN, HIGH);
    digitalWrite(ESC_BB_PWR_PIN, HIGH);
    sbStamp = millis();
    bbStamp = millis();
    printf("ESC task running!\r\n");
    while (1)
    {
        if (xQueueReceive(escspeed, (void *)&rcv_msg, 0) == pdTRUE)
        {
            spsb = rcv_msg.speedsb;
            spbb = rcv_msg.speedbb;
            escsb.write(map(rcv_msg.speedsb, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
            escbb.write(map(rcv_msg.speedbb, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
        }
        /*
            if the esc is not used for more than 60 seconds turn it off
        */
        if (spsb != 0)
        {
            sbStamp = millis();
            digitalWrite(ESC_SB_PWR_PIN, HIGH);
        }
        if (spbb != 0)
        {
            bbStamp = millis();
            digitalWrite(ESC_BB_PWR_PIN, HIGH);
        }
        if (sbStamp + 1000*60 < millis())
        {
            sbStamp = millis();
            digitalWrite(ESC_SB_PWR_PIN, LOW);
        }
        if (bbStamp + 1000*60 < millis())
        {
            bbStamp = millis();
            digitalWrite(ESC_BB_PWR_PIN, LOW);
        }
        delay(1);
    }
}