#include <Arduino.h>
#include "io_sub.h"
#include "../../RobobuoyDependency\RobobuoyVersion.h"
#include "../../RobobuoyDependency\RobobuoyMsg.h"
#include "../../RobobuoyDependency\RobobuoyDefaults.h"
#include "../../RobobuoyDependency\RobobuoyCalc.h"

void setup()
{
    Serial.begin(115200);
    Serial.println(SUBVERSION);
    pinMode(BUZZER_PIN, OUTPUT);
}

void loop(void)
{
    while (true)
    {
        delay(500);
    }
}