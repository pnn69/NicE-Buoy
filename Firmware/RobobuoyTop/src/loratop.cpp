#include <Arduino.h>
#include <LoRa.h>
#include "loratop.h"
#include "io_top.h"
#include "main.h"

static bool loraOk = false;
static unsigned long my_id = 0;
static char loraData[MAXSTRINGLENG];
static lorabuf loraMsgout = {};
static lorabuf loraMsgin = {};
static lorabuf pendingMsg[10] = {};
static RoboStruct loraStruct;
QueueHandle_t loraOut;
QueueHandle_t loraIn;

//  IDr,IDs,MSG,ACK,<data>
lorabuf decodeString(String incoming)
{
    lorabuf result;
    int commaIndex;
    // Parse mac
    commaIndex = incoming.indexOf(',');
    String hexString = incoming.substring(0, commaIndex);
    result.macIDr = strtoull(hexString.c_str(), NULL, 16);
    // Parse macIn
    incoming = incoming.substring(commaIndex + 1);
    commaIndex = incoming.indexOf(',');
    hexString = incoming.substring(0, commaIndex);
    result.macIDs = strtoull(hexString.c_str(), NULL, 16);
    // Parse msg
    incoming = incoming.substring(commaIndex + 1);
    commaIndex = incoming.indexOf(',');
    result.msg = incoming.substring(0, commaIndex).toInt();
    // Store remaining data as a char array
    incoming.toCharArray(result.data, incoming.length() + 1);
    // Parse ack
    incoming = incoming.substring(commaIndex + 1);
    commaIndex = incoming.indexOf(',');
    result.ack = incoming.substring(0, commaIndex).toInt();
    incoming = incoming.substring(commaIndex + 1);
    return result;
}

bool InitLora(void)
{

    loraOk = false;
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    Serial.print("LoRa setup ");
    // LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN); // only polling mode so no irq needed
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
            if (pendingMsg[i].retry < 0)
            {
                pendingMsg[i].retry--;
            }
            else
            {
                pendingMsg[i].msg = 0;
            }
            return in;
        }
    }
    return in;
}

String removeWhitespace(String str)
{
    String result = "";
    for (int i = 0; i < str.length(); i++)
    {
        if (str.charAt(i) != ' ')
        {
            result += str.charAt(i);
        }
    }
    return result;
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
    lorabuf in = decodeString(incoming);
    //  IDr,IDs,MSG,ACK,<data>
    xQueueSend(loraIn, (void *)&in, 10);          // send to main for prosessing
    if (in.macIDr == buoyId && in.ack == LORAACK) // A message form me so check if its a ACK message
    {
        removeAckMsg(loraMsgin.msg);
    }
}

bool sendLora(String loraTransmitt)
{
    if (LoRa.beginPacket()) // start packet
    {
        loraTransmitt = removeWhitespace(loraTransmitt);
        LoRa.write(loraTransmitt.length());
        LoRa.print(loraTransmitt);
        LoRa.endPacket(); // finish packet and send it
        return true;
    }
    return false;
}

void initloraqueue(void)
{
    loraIn = xQueueCreate(10, sizeof(lorabuf));
    loraOut = xQueueCreate(10, sizeof(lorabuf));
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
                // IDr,IDs,MSG,ACK,<data>
                String loraString = String(loraMsgout.macIDr, HEX) + "," + String(buoyId, HEX) + "," + String(loraMsgout.msg) + "," + String(loraMsgout.ack);
                loraString += removeWhitespace(String(loraMsgout.data));
                Serial.println("Lora OUT:" + loraString + " Length:" + String(strlen(loraMsgout.data)));
                while (sendLora(String(loraString)) != true)
                {
                    vTaskDelay(pdTICKS_TO_MS(50));
                }
                transmittReady = millis() + 250 + random(0, 50);
                if (loraMsgout.ack == LORAGETACK)
                {
                    loraMsgout.retry = 5;
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
            }
        }
        vTaskDelay(1);
    }
}