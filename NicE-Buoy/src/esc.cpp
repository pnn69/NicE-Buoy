/*
https://github.com/madhephaestus/ESP32Servo
https://dronebotworkshop.com/esp32-servo/
 */
#include <Arduino.h>
#include <ESP32Servo.h>
#include "esc.h"
// cannels for esc
#define bb 0 // Bakboord
#define sb 1 // Stuurboord

Servo escbb; // create servo object to control a servo
Servo escsb; // create servo object to control a servo

int speed_bb = 0; // variable to store the speed bb
int speed_sb = 0; // variable to store the speed sb

int esc_bb = 13;
int esc_sb = 14;

// value = map(value, 0, 180, 1000, 2000);

void InitEsc(void)
{
    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    escbb.setPeriodHertz(50);         // standard 50 hz servo
    escbb.attach(esc_bb, 1000, 2000); // attaches the servo on pin 18 to the servo object
    escsb.setPeriodHertz(50);         // standard 50 hz servo
    escsb.attach(esc_sb, 1000, 2000); // attaches the servo on pin 18 to the servo object
}

void EscTask(void *arg)
{
    InitEsc();
    int speed = 0;
    bool dir = 0;
    while (1)
    {
        if (dir == 0)
        {
            speed++;
            if (speed >= 100)
            {
                dir = 1;
            }
        }
        else
        {
            speed--;
            if (speed <= 100)
            {
                dir = 0;
            }
        }

        escbb.write(map(speed, -100, 100, 0, 180)); // tell servo to go to position in variable 'pos'
        escsb.write(map(speed, -100, 100, 0, 180)); // tell servo to go to position in variable 'pos'
        Serial.printf("esc speed: %03d%\r\n",speed);
        delay(100);
    }
}