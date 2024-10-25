#include <Arduino.h>
#include <LoRa.h>
#include "loratop.h"
#include "io_top.h"
#include "main.h"

static bool loraOk = false;
static char loraData[MAXSTRINGLENG];
static lorabuf loraMsgout = {};
static lorabuf pendingMsg[10] = {};
static RoboStruct loraStruct;
QueueHandle_t loraOut;
QueueHandle_t loraIn;

struct Data
{
    char mac[12]; // Fixed-size array for the MAC address (12 characters + null terminator)
    int msg;      // MSG field
    int ack;      // ACK field
};

// Function to extract the first 3 fields from a char array and return a struct
struct Data extractFields(char *input)
{
    struct Data result;

    // Extract the MAC address (assumed to be the first field in the array, 12 characters long)
    sscanf(input, "%12[^,]", result.mac); // Extract the first 12 characters (MAC address without colons)

    // Extract MSG and ACK (assuming they follow the MAC address)
    sscanf(input + strlen(result.mac) + 1, "%d,%d", &result.msg, &result.ack);

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

void storeAckMsg(lorabuf ackBuffer)
{
    int i = 0;
    while (i < 10)
    {
        if (pendingMsg[i].msg == 0)
        {
            memcpy(&pendingMsg[i].msg, &ackBuffer.msg, sizeof(ackBuffer.msg));
        }
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
    incoming.toCharArray(loraData, incoming.length() + 1);
    xQueueSend(loraIn, (void *)&loraData, 0); // send out trough Lora
    struct Data extractedData = extractFields(loraData);
    removeAckMsg(extractedData.msg);
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