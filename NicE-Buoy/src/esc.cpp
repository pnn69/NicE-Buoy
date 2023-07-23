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

Servo escbb; // create servo object to control a servo
Servo escsb; // create servo object to control a servo
static unsigned long sec01stamp;
static unsigned long sec30000stamp;
static int speedbb = 0, speedsb = 0;
static int speedbbsetpoint = 0, speedsbsetpoint = 0;

// value = map(value, 0, 180, 1000, 2000);

void triggerESC(void)
{
    Serial.println("Trigger ESC");
    escbb.write(map(100, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    escsb.write(map(100, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    delay(10);
    escbb.write(map(-100, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    escsb.write(map(-100, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    delay(10);
    escbb.write(map(0, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
    escsb.write(map(0, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
}

void InitEsc(void)
{
    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    escbb.setPeriodHertz(400);            // standard 400 hz servo
    escbb.attach(ESC_BB_PIN, 1000, 2000); // attaches the servo on pin 18 to the servo object
    escsb.setPeriodHertz(400);            // standard 400 hz servo
    escsb.attach(ESC_SB_PIN, 1000, 2000); // attaches the servo on pin 18 to the servo object
}

void EscTask(void *arg)
{
    Message rcv_msg;
    LMessage snd_msg;
    InitEsc();
    // esc init sequence
    triggerESC();
    escspeed = xQueueCreate(10, sizeof(Message));
    sec01stamp = millis();
    sec30000stamp = millis();
    while (1)
    {
        if (xQueueReceive(escspeed, (void *)&rcv_msg, 0) == pdTRUE)
        {
            speedbbsetpoint = rcv_msg.speedbb;
            speedsbsetpoint = rcv_msg.speedsb;
            snd_msg.speedbb = speedbbsetpoint;
            snd_msg.speedsb = speedsbsetpoint;
            snd_msg.ledstatus1 = rcv_msg.ledstatus1;
            snd_msg.ledstatus2 = rcv_msg.ledstatus2;
            if (xQueueSend(indicatorque, (void *)&snd_msg, 10) != pdTRUE)
            {
                Serial.println("Error sending speed to indicatorque");
            }
            // Serial.printf("esc speed bb: %03d speed sb: %03d\r\n", speedbbsetpoint, speedsbsetpoint);
        }
        // smoodly go to setpoint
        if (millis() - sec01stamp >= 25)
        {
            sec01stamp = millis();
            if (speedbb != speedbbsetpoint)
            {
                if (speedbb < speedbbsetpoint)
                {
                    speedbb++;
                }
                else
                {
                    speedbb--;
                }
                escbb.write(map(speedbb, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
                sec30000stamp = millis();
            }
            if (speedsb != speedsbsetpoint)
            {
                if (speedsb < speedsbsetpoint)
                {
                    speedsb++;
                }
                else
                {
                    speedsb--;
                }
                escsb.write(map(speedsb, -100, 100, 180, 0)); // tell servo to go to position in variable 'pos'
                sec30000stamp = millis();
            }
            // Serial.printf("esc speed bb: %03d speed sb: %03d\r\n", speedbbsetpoint, speedsbsetpoint);
            // Serial.printf("esc speed bb: %03d speed sb: %03d\r\n", speedbb, speedsb);
        }
        if (millis() - sec30000stamp >= 1000 * 60 * 5) // no ESC command for 5 minutes... trigger ESC so prevent from beeping
        {
            sec30000stamp = millis();
            triggerESC();
        }
        delay(1);
    }
}