/*
https://github.com/madhephaestus/ESP32Servo
https://dronebotworkshop.com/esp32-servo/
 */
#include <Arduino.h>
#include <ESP32Servo.h>
#include "esc.h"
#include "indicator.h"
#include "io.h"
// cannels for esc

QueueHandle_t escspeed;

Servo escbb;      // create servo object to control a servo
Servo escsb;      // create servo object to control a servo
int speed_bb = 0; // variable to store the speed bb
int speed_sb = 0; // variable to store the speed sb

// value = map(value, 0, 180, 1000, 2000);

void InitEsc(void)
{
    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    escbb.setPeriodHertz(50); // standard 50 hz servo
    // escbb.attach(ESC_BB_PIN, 1000, 2000); // attaches the servo on pin 18 to the servo object
    escbb.attach(ESC_BB_PIN, 1050, 2000);  // attaches the servo on pin 18 to the servo object
    escsb.setPeriodHertz(50);             // standard 50 hz servo
    //escsb.attach(ESC_SB_PIN, 1000, 2000); // attaches the servo on pin 18 to the servo object
    escsb.attach(ESC_SB_PIN, 1020, 2000); // attaches the servo on pin 18 to the servo object
}

void EscTask(void *arg)
{
    Message rcv_msg;
    LMessage snd_msg;
    InitEsc();
    escspeed = xQueueCreate(10, sizeof(Message));
    int speedbb = 0, speedsb = 0;
    while (1)
    {
        if (xQueueReceive(escspeed, (void *)&rcv_msg, 0) == pdTRUE)
        {
            speedbb = rcv_msg.speedbb;
            speedsb = rcv_msg.speedsb;
            escbb.write(map(speedbb, -100, 100, 0, 180)); // tell servo to go to position in variable 'pos'
            escsb.write(map(speedsb, -100, 100, 0, 180)); // tell servo to go to position in variable 'pos'
            snd_msg.speedbb = speedbb;
            snd_msg.speedsb = speedsb;
            memcpy(snd_msg.ledstatus, rcv_msg.ledstatus, sizeof(rcv_msg.ledstatus));
            if (xQueueSend(indicatorque, (void *)&snd_msg, 10) != pdTRUE)
            {
                Serial.println("Error sending speed to indicatorque");
            }
            // Serial.printf("esc speed bb: %03d speed sb: %03d\r\n", speedbb, speedsb);
        }
        delay(10);
    }
}