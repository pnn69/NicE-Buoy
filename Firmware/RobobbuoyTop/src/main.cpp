#include <Arduino.h>
#include "../../RobobuoyDependency\RobobuoyVersion.h"
#include "../../RobobuoyDependency\RobobuoyMsg.h"
#include "../../RobobuoyDependency\RobobuoyDefaults.h"
#include "../../RobobuoyDependency\RobobuoyCalc.h"
#include "io_top.h"
#include "robolora.h"
#include "leds.h"
#include "robowifi.h"
#include "gps.h"

static LedData collorStatus;
static LedData collorUtil;
static LedData collorGps;

void setup()
{
    Serial.begin(115200);
    Serial.println(TOPVERSION);
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4024, NULL, 20, NULL, 0);
    initLedTask();
    xTaskCreatePinnedToCore(LedTask, "LedTask", 1000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(GpsTask, "GpsTask", 2000, NULL, 10, NULL, 1);
    pinMode(BUZZER_PIN, OUTPUT);
    delay(1000);
}

void loop(void)
{
    LedData Gps;
    collorGps.color = CRGB::Black;
    collorGps.blink = 0;
    collorGps.brightness = 200;
    xQueueSend(ledStatus, (void *)&collorGps, 10);
    xQueueSend(ledUtil, (void *)&collorGps, 10);
    xQueueSend(ledGps, (void *)&collorGps, 10);
    Serial.println("Entering main loop!");
    while (true)
    {
        delay(1000);
    }
}