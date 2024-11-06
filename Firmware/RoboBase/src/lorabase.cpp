#include <Arduino.h>
#include <LoRa.h>
#include "lorabase.h"
#include "io_base.h"
#include "main.h"

static bool loraOk = false;
static char loraData[MAXSTRINGLENG];
static lorabuf loraMsgout = {};
static lorabuf loraMsgin = {};
static lorabuf pendingMsg[10] = {};
static RoboStruct loraStruct;
QueueHandle_t loraOut;
QueueHandle_t loraIn;

//***************************************************************************************************
//  Recieve and decode incomming lora message
//***************************************************************************************************
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
    // Parse ack
    incoming = incoming.substring(commaIndex + 1);
    commaIndex = incoming.indexOf(',');
    result.ack = incoming.substring(0, commaIndex).toInt();
    // Parse msg
    incoming = incoming.substring(commaIndex + 1);
    commaIndex = incoming.indexOf(',');
    // Store remaining data as a char array
    result.msg = incoming.substring(0, commaIndex).toInt();
    incoming.toCharArray(result.data, incoming.length() + 1);
    incoming = incoming.substring(commaIndex + 1);
    commaIndex = incoming.indexOf(',');
    return result;
}

//***************************************************************************************************
//  Lora init
//***************************************************************************************************
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

//***************************************************************************************************
//  Lora data out
//***************************************************************************************************
bool sendLora(String loraTransmitt)
{
    if (LoRa.beginPacket()) // start packet
    {
        LoRa.write(loraTransmitt.length());
        LoRa.print(loraTransmitt);
        LoRa.endPacket(); // finish packet and send it
        Serial.println("Lora out<" + loraTransmitt + ">");
        return true;
    }
    return false;
}

//***************************************************************************************************
//  Store to ack buffer
//***************************************************************************************************
void storeAckMsg(lorabuf ackBuffer)
{
    int i = 0;
    while (i < 10)
    {
        if (pendingMsg[i].msg == 0)
        {
            // printf("storing on pos %d\r\n",i);
            memcpy(&pendingMsg[i], &ackBuffer, sizeof(ackBuffer));
            return;
        }
        i++;
    }
}

//***************************************************************************************************
//  Remove to ack buffer
//***************************************************************************************************
void removeAckMsg(lorabuf ackBuffer)
{
    int i = 0;
    while (i < 10)
    {
        //Serial.println("ackBuffer.macIDr: " + String(ackBuffer.macIDs,HEX) +" pendingMsg[i].macIDs: " + String(pendingMsg[i].macIDr,HEX));
        if (pendingMsg[i].msg == ackBuffer.msg && pendingMsg[i].macIDr == ackBuffer.macIDs)
        {
            pendingMsg[i].ack = 0;
            pendingMsg[i].msg = 0;
            pendingMsg[i].macIDs = 0;
            pendingMsg[i].macIDr = 0;
            pendingMsg[i].retry = 0;
            //printf("message removed for ack on pos:%d\r\n", i);
            return;
        }
        i++;
    }
}

//***************************************************************************************************
//  check ack buffer
//  remove data if more than n times has failed
//***************************************************************************************************
struct lorabuf chkAckMsg(void)
{
    struct lorabuf in;
    in.ack = 0;
    in.msg = 0;
    int i = 0;
    while (i < 10)
    {
        if (pendingMsg[i].msg != 0)
        {
            memcpy(&in, &pendingMsg[i], sizeof(lorabuf));
            pendingMsg[i].retry--;
            if (pendingMsg[i].retry == 0)
            {
                pendingMsg[i].msg = 0;
                pendingMsg[i].ack = 0;
                pendingMsg[i].macIDs = 0;
                pendingMsg[i].macIDr = 0;
            }
            //printf(" message ack on pos:%d rettrys left %d   msg:%d\r\n", i, pendingMsg[i].retry, pendingMsg[i].msg);
            return in;
        }
        i++;
    }
    return in;
}

//***************************************************************************************************
//  Recieve and decode incomming lora message
//***************************************************************************************************
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
    Serial.println("Lora in <" + incoming + ">");
    lorabuf in = decodeString(incoming);
    if (in.macIDr == buoyId && in.ack == LORAACK) // A message form me so check if its a ACK message
    {
        removeAckMsg(in);
        // printf("Lora Ack recieved msg:%d cleared\r\n", in.msg);
        return;
    }
    if (in.macIDr == buoyId || in.macIDr == BUOYIDALL) // A message form me
    {
        if (in.ack == LORAGETACK)
        {
            // IDr,IDs,ACK,MSG
            loraMsgout.macIDr = in.macIDs;
            loraMsgout.macIDs = buoyId;
            loraMsgout.ack = LORAACK;
            String out = String(in.msg);
            out.toCharArray(loraMsgout.data, out.length() + 1);
            xQueueSend(loraOut, (void *)&in, 10); // send ACK out
        }
        xQueueSend(loraIn, (void *)&in, 10); // send to main
    }
    else
    {
        Serial.println("LoraData not for me: " + String(in.data) + " macIDr:" + String(in.macIDr, HEX) + " macIDs:" + String(in.macIDs, HEX));
    }
}

//***************************************************************************************************
//  Remove white space
//***************************************************************************************************
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

//***************************************************************************************************
//  Init lora queue
//***************************************************************************************************
void initloraqueue(void)
{
    loraOut = xQueueCreate(10, sizeof(char[MAXSTRINGLENG]));
    loraIn = xQueueCreate(10, sizeof(lorabuf));
    InitLora();
}

//***************************************************************************************************
//  Lora task
//***************************************************************************************************
void LoraTask(void *arg)
{
    unsigned long transmittReady = 0;
    unsigned long retransmittReady = 0;
    while (1)
    {
        onReceive(LoRa.parsePacket()); // check if there is new data availeble
        if (transmittReady < millis())
        {
            if (xQueueReceive(loraOut, (void *)&loraMsgout, 10) == pdTRUE)
            {
                // IDr,IDs,ACK,MSG,<data>
                String loraString = String(loraMsgout.macIDr, HEX) + "," + String(buoyId, HEX) + "," + String(loraMsgout.ack);
                loraString += "," + removeWhitespace(String(loraMsgout.data));
                while (sendLora(loraString) != true)
                {
                    vTaskDelay(pdTICKS_TO_MS(50));
                }
                transmittReady = millis() + 150 + random(0, 50);
                if (loraMsgout.ack == LORAGETACK)
                {
                    loraMsgout.retry = 5;
                    storeAckMsg(loraMsgout);                            // put data in buffer (will be removed on ack)
                    retransmittReady = millis() + 900 + random(0, 150); // give some time for ack
                }
            }
        }
        /* retry one time*/
        if (retransmittReady < millis())
        {
            loraMsgout = chkAckMsg();
            if (loraMsgout.msg != 0)
            {
                String loraString = String(loraMsgout.macIDr, HEX) + "," + String(buoyId, HEX) + "," + String(loraMsgout.ack);
                loraString += "," + removeWhitespace(String(loraMsgout.data));
                while (sendLora(loraString) != true)
                {
                    vTaskDelay(pdTICKS_TO_MS(50));
                }
                retransmittReady = millis() + 900 + random(0, 150);
            }
        }
        vTaskDelay(1);
    }
}