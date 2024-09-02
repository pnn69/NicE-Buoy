#include <Arduino.h>
#include "freertos/task.h"
#include "io_sub.h"
#include "subwifi.h"
//#include "leds.h"
#include "../../RobobuoyDependency\RobobuoyVersion.h"
#include "../../RobobuoyDependency\RobobuoyMsg.h"
#include "../../RobobuoyDependency\RobobuoyDefaults.h"
#include "../../RobobuoyDependency\RobobuoyCalc.h"
//Speeddef msg_speed;
//Accudef msg_accu;

void setup()
{
    Serial.begin(115200);
    Serial.println(SUBVERSION);
    pinMode(BUZZER_PIN, OUTPUT);
    //initLedTask();
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 2000, NULL, 2, NULL, 0);
 }

void loop(void)
{
    float vbat = 0;
    int speedbb = 50, speedsb = 0;
    while (true)
    {
        // msg_accu.accuvoltage = vbat;
        // msg_speed.speedbb = speedbb;
        // msg_speed.speedsb = speedsb;
        // xQueueSend(ledsSpeed, &msg_speed, 10); // update leds
        // xQueueSend(ledAccuVoltage, (void *)&msg_accu, 10); // update leds
        vTaskDelay(1);
    }
}