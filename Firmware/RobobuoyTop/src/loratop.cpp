#include <Arduino.h>
#include <LoRa.h>
#include "loratop.h"
#include "io_top.h"
#include "main.h"
#include "topwifi.h"

static const char *TAG = "lora.cpp";
static bool loraOk = false;
static unsigned long my_id = 0;
static char loraData[MAXSTRINGLENG];
static RoboStruct loraMsgout = {};
static RoboStruct loraMsgin = {};
static RoboStruct pendingMsg[10] = {};
static RoboStruct loraStruct;
static unsigned long transmittReady = 0;
QueueHandle_t loraOut;
QueueHandle_t loraIn;
static unsigned long buoyId = 0;

//***************************************************************************************************
//  Lora init
//***************************************************************************************************
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
    buoyId = espMac();
    return true;
}

//***************************************************************************************************
//  initialise lora queue
//***************************************************************************************************
void initloraqueue(void)
{
    loraIn = xQueueCreate(10, sizeof(RoboStruct));
    loraOut = xQueueCreate(10, sizeof(RoboStruct));
    InitLora();
    Serial.print("#BuoyId=");
    Serial.println(espMac(), HEX);
}

//***************************************************************************************************
//  Store to ack buffer
//***************************************************************************************************
void storeAckMsg(RoboStruct ackBuffer)
{
    int i = 0;
    while (i < 10)
    {
        if (pendingMsg[i].cmd == 0)
        {
            memcpy(&pendingMsg[i], &ackBuffer, sizeof(ackBuffer));
            Serial.println("message stored on pos:" + String(i) + " rettrys left:" + ackBuffer.retry + " for:" + String(ackBuffer.IDr, HEX) + " msg:" + ackBuffer.cmd);
            return;
        }
        i++;
    }
}

//***************************************************************************************************
//  Remove to ack buffer
//***************************************************************************************************
void removeAckMsg(RoboStruct ackBuffer)
{
    Serial.println("looking for msg:" + String(ackBuffer.cmd) + "Id:" + String(ackBuffer.IDs, HEX));
    // printf("looking for msg%d of id %lld\r\n", ackBuffer.cmd, ackBuffer.macIDs);
    int i = 0;
    while (i < 10)
    {
        // Serial.println("ackBuffer.macIDr: " + String(ackBuffer.macIDs,HEX) +" pendingMsg[i].macIDs: " + String(pendingMsg[i].macIDr,HEX));
        if (pendingMsg[i].cmd == ackBuffer.cmd && pendingMsg[i].IDr == ackBuffer.IDs)
        {
            Serial.println("message removed pos:" + String(i) + " rettrys left:" + pendingMsg[i].retry + " for:" + String(pendingMsg[i].IDr, HEX) + " msg:" + pendingMsg[i].cmd);
            pendingMsg[i].ack = 0;
            pendingMsg[i].cmd = 0;
            pendingMsg[i].IDs = 0;
            pendingMsg[i].IDr = 0;
            pendingMsg[i].retry = 0;
            //            printf("message removed for ack on pos:%d\r\n", i);
            return;
        }
        i++;
    }
}

//***************************************************************************************************
//  check ack buffer
//  remove data if more than n times has failed
//***************************************************************************************************
RoboStruct chkAckMsg()
{
    RoboStruct in;
    in.ack = 0;
    in.cmd = 0;
    int i = 0;
    while (i < 10)
    {
        if (pendingMsg[i].cmd != 0)
        {
            memcpy(&in, &pendingMsg[i], sizeof(RoboStruct));
            pendingMsg[i].retry--;
            if (pendingMsg[i].retry == 0)
            {
                pendingMsg[i].cmd = 0;
                pendingMsg[i].ack = 0;
                pendingMsg[i].IDs = 0;
                pendingMsg[i].IDr = 0;
            }
            // printf("message ack on pos:%d rettrys left %d to:%lld  msg:%d\r\n", i, pendingMsg[i].retry, pendingMsg[i].macIDr, pendingMsg[i].cmd);
            Serial.println("message ack on pos:" + String(i) + " rettrys left:" + pendingMsg[i].retry + " for:" + String(pendingMsg[i].IDr, HEX) + " msg:" + pendingMsg[i].cmd);
            return in;
        }
        i++;
    }
    return in;
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
        Serial.println("#error: message length does not match length");
        return; // skip rest of function
    }
    Serial.println("#Lora_i <" + incoming + ">");
    RoboStruct in = rfDeCode(incoming);
    if (in.IDr == buoyId && in.ack == LORAACK) // A message form me so check if its a ACK message
    {
        removeAckMsg(in);
        printf("#Lora Ack recieved buffer cleared\r\n");
        return;
    }
    if (in.IDr == buoyId || in.IDr == BUOYIDALL) // A message form me so check if its a ACK message
    {
        if (in.ack == LORAGETACK) // on ack request send ack back
        {
            // IDr,IDs,ACK,MSG
            loraMsgout.IDr = in.IDs;
            loraMsgout.IDs = buoyId;
            loraMsgout.cmd = in.cmd;
            loraMsgout.ack = LORAACK;
            xQueueSend(loraOut, (void *)&loraMsgout, 10); // send ACK out
        }
        xQueueSend(loraIn, (void *)&in, 10); // send to main
    }
}

//***************************************************************************************************
//  Lora data out
//***************************************************************************************************
bool sendLora(String loraTransmitt)
{
    if (LoRa.beginPacket()) // start packet
    {
        loraTransmitt = removeWhitespace(loraTransmitt);
        LoRa.write(loraTransmitt.length());
        LoRa.print(loraTransmitt);
        LoRa.endPacket(); // finish packet and send it
        Serial.println("#Lora_o <" + loraTransmitt + ">");
        return true;
    }
    return false;
}

//***************************************************************************************************
//  Lora task
//***************************************************************************************************
void LoraTask(void *arg)
{
    unsigned long retransmittReady = 0;
    delay(500);
    // while (xQueueReceive(loraOut, (void *)&loraMsgout, 10) == pdTRUE)
    // {
    //     delay(10);
    // }
    while (1)
    {
        onReceive(LoRa.parsePacket()); // check if there is new data availeble
        if (transmittReady < millis())
        {
            if (xQueueReceive(loraOut, (void *)&loraMsgout, 10) == pdTRUE)
            {
                // IDr,IDs,ACK,MSG,<data>
                loraMsgout.IDs = buoyId;
                String loraString = rfCode(loraMsgout);
                while (sendLora(String(loraString)) != true)
                {
                    vTaskDelay(pdTICKS_TO_MS(50));
                }
                transmittReady = millis() + 150;
                if (loraMsgout.ack == LORAGETACK)
                {
                    loraMsgout.retry = 5;
                    storeAckMsg(loraMsgout);                            // put data in buffer (will be removed on ack)
                    retransmittReady = millis() + 900 + random(0, 150); // give some time for ack
                    transmittReady = millis() + 250;
                }
            }
        }
        /* retry one time*/
        if (retransmittReady < millis())
        {
            loraMsgout = chkAckMsg();
            if (loraMsgout.cmd != 0)
            {
                String loraString = rfCode(loraMsgout);
                while (sendLora(loraString) != true)
                {
                    vTaskDelay(pdTICKS_TO_MS(50));
                }
                retransmittReady = millis() + 1500 + random(0, 150);
            }
        }
        vTaskDelay(1);
    }
}