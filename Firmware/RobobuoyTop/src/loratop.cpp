#include <Arduino.h>
#include <LoRa.h>
#include "loratop.h"
#include "io_top.h"
#include "main.h"

static bool loraOk = false;
static char loraData[MAXSTRINGLENG];
QueueHandle_t loraOut;
QueueHandle_t loraIn;

bool InitLora(void)
{

    loraOk = false;
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    Serial.print("LoRa setup ");
    // LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN); // only polling mode so no irq neede
    if (!LoRa.begin(LoRa_frequency))
    {
        Serial.println("Failed!");
        return false;
    }
    LoRa.enableCrc();
    Serial.println("Succes!");
    loraOk = true;
    return true;
}

// poll wtih in mainloop onReceive(LoRa.parsePacket());

void onReceive(int packetSize)
{
    if (packetSize == 0)
        return; // if there's no packet, return
    // read packet header bytes:
    byte incomingLength = LoRa.read(); // incoming msg length
    String incoming = "";
    while (LoRa.available())
    {
        incoming += (char)LoRa.read();
    }

    if (incomingLength != incoming.length())
    { // check length for error
        Serial.println("error: message length does not match length");
        return; // skip rest of function
    }
    incoming.toCharArray(loraData, incoming.length() + 1);
    xQueueSend(loraIn, (void *)&loraData, 0); // send out trough Lora
    // decode string
}

void sendLora(String loraTransmitt)
{
    LoRa.beginPacket(); // start packet
    LoRa.write(loraTransmitt.length());
    LoRa.print(loraTransmitt);
    LoRa.endPacket(); // finish packet and send it
}

void initloraqueue(void)
{
    loraOut = xQueueCreate(10, sizeof(char[MAXSTRINGLENG]));
    loraIn = xQueueCreate(10, sizeof(char[MAXSTRINGLENG]));
}

void LoraTask(void *arg)
{
    unsigned long transmittReady = 0;
    char dataOut[MAXSTRINGLENG];
    while (1)
    {
        onReceive(LoRa.parsePacket());
        if (transmittReady < millis())
        {
            if (LoRa.beginPacket()) // check if the transmitter is ready
            {
                if (xQueueReceive(loraOut, (void *)&dataOut, 10) == pdTRUE)
                {
                    Serial.println("Lora data recieved from main:" + String(dataOut));
                    //sendLora(dataOut);
                    transmittReady = millis() + 250 + random(0, 50);
                }
            }
        }
        vTaskDelay(1);
    }
}
