#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "io_top.h"
#include "robolora.h"
#include "../../RobobuoyDependency\RobobuoyDefaults.h"

bool lora = false;
#define RADIO_DIO0_PIN 4
bool InitLora(void)
{
    Serial.println("LoRa setup");
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LoRa_frequency))
    {
        Serial.println("Lora setup Failed!");
        return false;
    }
    Serial.println("Lora setup Succes!");
    lora = true;
    return true;
}

void LoraTask(void *arg)
{
    lora = InitLora();
    while (1)
    {
        delay(100);
    }
}