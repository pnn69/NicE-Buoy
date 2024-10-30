#include <Arduino.h>
#include <LoRa.h>
#include "lorabase.h"
#include "io_base.h"
#include "main.h"

struct Data
{
    uint64_t id; // Fixed-size array for the MAC address (12 characters + null terminator)
    int msg;     // MSG field
    int ack;     // ACK field
};

static bool loraOk = false;
static char loraData[MAXSTRINGLENG];
static lorabuf loraMsgout = {};
static lorabuf loraMsgin = {};
static lorabuf pendingMsg[10] = {};
static RoboStruct loraStruct;
QueueHandle_t loraOut;
QueueHandle_t loraIn;

struct Data decodeString(String incoming)
{
    struct Data result;
    // Split the incoming string by commas
    int commaIndex = incoming.indexOf(',');
    String hexString = incoming.substring(0, commaIndex);
    // Convert the first part (hexadecimal string) to uint64_t
    result.id = strtoull(hexString.c_str(), NULL, 16);
    // Get the remaining part of the string after the first comma
    incoming = incoming.substring(commaIndex + 1);
    // Parse the next two integer values
    commaIndex = incoming.indexOf(',');
    result.msg = incoming.substring(0, commaIndex).toInt(); // Convert to integer
    incoming = incoming.substring(commaIndex + 1);          // Move to next part
    commaIndex = incoming.indexOf(',');
    result.ack = incoming.substring(0, commaIndex).toInt(); // Convert to integer
    return result;
}

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

bool sendLora(String loraTransmitt)
{
    if (LoRa.beginPacket()) // start packet
    {
        LoRa.write(loraTransmitt.length());
        LoRa.print(loraTransmitt);
        LoRa.endPacket(); // finish packet and send it
        return true;
    }
    return false;
}

void storeAckMsg(lorabuf ackBuffer)
{
    int i = 0;
    while (i < 10)
    {
        if (pendingMsg[i].msg == 0)
        {
            memcpy(&pendingMsg[i].msg, &ackBuffer.msg, sizeof(ackBuffer.msg));
            return;
        }
        i++;
    }
}

void removeAckMsg(int msg)
{
    int i = 0;
    while (i < 10)
    {
        if (pendingMsg[i].msg == msg)
        {
            pendingMsg[i].ack = 0;
            pendingMsg[i].msg = 0;
        }
        i++;
    }
}

struct lorabuf chkAckMsg(void)
{
    struct lorabuf in;
    in.ack = 0;
    in.msg = 0;
    int i = 0;
    while (i < 10)
    {
        if (pendingMsg[i++].msg != 0)
        {
            memcpy(&in, &pendingMsg[i], sizeof(lorabuf));
        }
    }
    return in;
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
    incoming.toCharArray(loraMsgin.data, incoming.length() + 1);
    xQueueSend(loraIn, (void *)&loraMsgin, 10); // send out trough Lora
    sendLora(incoming);
    struct Data piet = decodeString(incoming);
    removeAckMsg(piet.msg);
}


void initloraqueue(void)
{
    loraOut = xQueueCreate(10, sizeof(char[MAXSTRINGLENG]));
    loraIn = xQueueCreate(10, sizeof(lorabuf));
}

void LoraTask(void *arg)
{
    InitLora();
    unsigned long transmittReady = 0;
    unsigned long retransmittReady = 0;
    while (1)
    {
        onReceive(LoRa.parsePacket()); // check if there is new data availeble
        if (transmittReady < millis())
        {
            if (xQueueReceive(loraOut, (void *)&loraMsgout, 10) == pdTRUE)
            {
                Serial.println("Lora OUT:" + String(loraMsgout.data) + "Length:" + String(strlen(loraMsgout.data)));
                while (sendLora(String(loraMsgout.data)) != true)
                {
                    vTaskDelay(pdTICKS_TO_MS(100));
                }
                transmittReady = millis() + 250 + random(0, 50);
                if (loraMsgout.ack == LORAGETACK)
                {
                    storeAckMsg(loraMsgout);            // put data in buffer (will be removed on ack)
                    retransmittReady = millis() + 1000; // give some time for ack
                }
            }
        }
        /* retry one time*/
        if (retransmittReady < millis())
        {
            loraMsgout = chkAckMsg();
            if (loraMsgout.msg != 0)
            {
                while (sendLora(String(loraMsgout.data)) != true)
                {
                    vTaskDelay(pdTICKS_TO_MS(100));
                }
                retransmittReady = millis() + 1000 + random(0, 50);
                removeAckMsg(loraMsgout.msg);
            }
        }
        vTaskDelay(1);
    }
}