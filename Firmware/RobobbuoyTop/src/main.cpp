#include <Arduino.h>
#include "io_top.h"
#include "robolora.h"
#include "../../RobobuoyDependency\RobobuoyVersion.h"
#include "../../RobobuoyDependency\RobobuoyMsg.h"
#include "../../RobobuoyDependency\RobobuoyDefaults.h"
#include "../../RobobuoyDependency\RobobuoyCalc.h"

void setup()
{
    Serial.begin(115200);
    Serial.println(TOPVERSION);
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4024, NULL, 10, NULL, 0);
    //  temp misuage led button ;)
    pinMode(33, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
}

void loop(void)
{
    while (true)
    {
        delay(500);
    }
}