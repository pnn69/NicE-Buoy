/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
*/
#include <Arduino.h>
#include <SPI.h> // include libraries
#include <LoRa.h>
#include "io.h"
#include "LiLlora.h"
#include <RoboCompute.h>
#include "sercom.h"
#include "controlwifi.h"

int counter = 0;

const int csPin = 18;    // LoRa radio chip select
const int resetPin = 23; // LoRa radio reset
const int irqPin = 26;   // change for your board; must be a hardware interrupt pin

static unsigned long lasttransmission = 0;
static unsigned long transmittReady = 0;

static RoboStruct loraMsgout = {};
static RoboStruct loraMsser = {};
RoboStruct pendingMsg[10] = {};
char buffer[128]; // or however long your messages are

String outgoing;          // outgoing message
byte msgCount = 0;        // count of outgoing messages
byte localAddress = 0xFE; // address of this device
byte destination = 0x01;  // destination to send to
QueueHandle_t loraOut;
QueueHandle_t loraIn;
QueueHandle_t loraToMain;

static unsigned long buoyId = 0;
// struct loraDataType loraIn;
// struct loraDataType loraOut;

//***************************************************************************************************
//  Lora init
//***************************************************************************************************
bool InitLora(void)
{
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    Serial.print("LoRa setup ");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LoRa_frequency))
    {
        Serial.println(" failed!");
        return false;
    }
    LoRa.enableCrc();
    Serial.println(" Succes!");
    return true;
}

//***************************************************************************************************
//  initialise lora queue
//***************************************************************************************************
void initloraqueue(void)
{
    loraIn = xQueueCreate(10, sizeof(RoboStruct));
    loraOut = xQueueCreate(10, sizeof(RoboStruct));
    loraToMain = xQueueCreate(10, sizeof(RoboStruct));
    InitLora();
    buoyId = espMac();
    Serial.print("#BuoyId=");
    Serial.println(buoyId, HEX);
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
            pendingMsg[i] = ackBuffer;
            // Serial.println("message stored on pos:" + String(i) + " rettrys left:" + ackBuffer.retry + " for:" + String(ackBuffer.IDr, HEX) + " msg:" + ackBuffer.cmd);
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
    // Serial.println("looking for msg:" + String(ackBuffer.cmd) + " Id:" + String(ackBuffer.IDs, HEX));
    int i = 0;
    while (i < 10)
    {
        // Serial.println("Found:" + String(pendingMsg[i].cmd) + " Id:" + String(pendingMsg[i].IDr, HEX));
        if (pendingMsg[i].cmd == ackBuffer.cmd && pendingMsg[i].IDr == ackBuffer.IDs)
        {
            pendingMsg[i].ack = 0;
            pendingMsg[i].cmd = 0;
            pendingMsg[i].IDs = 0;
            pendingMsg[i].IDr = 0;
            pendingMsg[i].retry = 0;
            return;
        }
        i++;
    }
}

//***************************************************************************************************
//  check ack buffer
//  remove data if more than n times has failed
//***************************************************************************************************
RoboStruct chkAckMsg(void)
{
    RoboStruct in;
    in.ack = 0;
    in.cmd = 0;
    int i = 0;
    while (i < 10)
    {
        if (pendingMsg[i].cmd != 0)
        {
            in = pendingMsg[i];
            pendingMsg[i].retry--;
            if (pendingMsg[i].retry == 0)
            {
                pendingMsg[i].cmd = 0;
                pendingMsg[i].ack = 0;
                pendingMsg[i].IDs = 0;
                pendingMsg[i].IDr = 0;
            }
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
        return;                        // if there's no packet, return
    byte incomingLength = LoRa.read(); // incoming msg length
    String incoming = "";
    digitalWrite(LED_PIN, HIGH); // turn on led
    while (LoRa.available())
    {
        incoming += (char)LoRa.read();
    }
    digitalWrite(LED_PIN, LOW); // turn off led
    if (incomingLength != incoming.length())
    { // check length for error
        Serial.println("#error: message length does not match length");
        return; // skip rest of function
    }
    Serial.println(incoming);
    RoboStruct in;
    rfDeCode(incoming, &in);
    if ((in.IDr == buoyId || in.IDr == 0x99) && in.ack == ACK) // A message form me so check if its a ACK message
    {
        removeAckMsg(in);
        // printf("#Lora Ack recieved buffer cleared\r\n");
        return;
    }
    if (in.IDr == buoyId || in.IDr == BUOYIDALL || in.IDr == 0x99) // A message form me so check if its a ACK message
    {
        if (in.ack == GETACK) // on ack request send ack back
        {
            // IDr,IDs,ACK,MSG
            loraMsgout.IDr = in.IDs;
            loraMsgout.IDs = buoyId;
            loraMsgout.cmd = in.cmd;
            loraMsgout.ack = ACK;
            xQueueSend(loraOut, (void *)&loraMsgout, 10); // send ACK out
        }
    }
    xQueueSend(loraToMain, (void *)&in, 10); // send to main
}

//***************************************************************************************************
//  Lora data out
//***************************************************************************************************
bool sendLora(String loraTransmitt)
{
    if (transmittReady < millis())
    {
        if (LoRa.beginPacket()) // start packet
        {
            digitalWrite(LED_PIN, HIGH); // turn on led
            LoRa.write(loraTransmitt.length());
            LoRa.print(loraTransmitt);
            
            if (LoRa.endPacket() == 1) // finish packet and send it (returns 1 on success)
            {
                digitalWrite(LED_PIN, LOW); // turn off led
                printf("#####################Lora sent: %s\r\n", loraTransmitt.c_str());
                transmittReady = millis() + 10;
                return true; // return true if success
            }
            else
            {
                digitalWrite(LED_PIN, LOW); // turn off led
                Serial.println("#Error: LoRa.endPacket() failed during transmission.");
            }
        }
        else
        {
            Serial.println("#Error: LoRa.beginPacket() failed.");
        }
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
    while (1)
    {
        onReceive(LoRa.parsePacket()); // check if there is new data availeble

        // data to send from main
        static RoboStruct txMsg;
        if (xQueueReceive(loraOut, (void *)&txMsg, 1) == pdTRUE)
        {
            txMsg.IDs = espMac();
            String loraString = rfCode(&txMsg);

            int attempts = 0;
            const int maxAttempts = 3;
            bool sent = false;
            while (attempts < maxAttempts)
            {
                if (sendLora(String(loraString)))
                {
                    sent = true;
                    break;
                }
                attempts++;
                for (int d = 0; d < 15; d++) // 15 * 10ms = 150ms
                {
                    onReceive(LoRa.parsePacket()); // keep receiving while waiting
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }

            if (!sent)
            {
                Serial.println("#Error: Failed to transmit LoRa packet after 3 attempts. Transceiver may be locked up.");
                Serial.println("#Attempting LoRa transceiver self-healing...");
                if (InitLora())
                {
                    Serial.println("#LoRa self-healing successful!");
                }
                else
                {
                    Serial.println("#LoRa self-healing failed!");
                }
            }

            Serial.println(loraString);
            if (txMsg.ack == GETACK || txMsg.ack == 3) // 3 is Python's GETACK
            {
                txMsg.retry = 5;
                storeAckMsg(txMsg);                                 // put data in buffer (will be removed on ack)
                retransmittReady = millis() + 500 + random(0, 150); // give some time for ack
            }
        }

        /* retry one time*/
        if (retransmittReady < millis())
        {
            static RoboStruct retryMsg;
            retryMsg = chkAckMsg();
            if (retryMsg.cmd != 0)
            {
                String loraString = rfCode(&retryMsg);
                int attempts = 0;
                const int maxAttempts = 3;
                bool sent = false;
                while (attempts < maxAttempts)
                {
                    if (sendLora(loraString))
                    {
                        sent = true;
                        break;
                    }
                    attempts++;
                    for (int d = 0; d < 5; d++) // 5 * 10ms = 50ms
                    {
                        onReceive(LoRa.parsePacket()); // keep receiving while waiting
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                }

                if (!sent)
                {
                    Serial.println("#Error: Failed to retransmit ACK-pending LoRa packet. Transceiver may be locked up.");
                    Serial.println("#Attempting LoRa transceiver self-healing...");
                    if (InitLora())
                    {
                        Serial.println("#LoRa self-healing successful!");
                    }
                    else
                    {
                        Serial.println("#LoRa self-healing failed!");
                    }
                }
                retransmittReady = millis() + 900 + random(0, 150);
            }
            else
            {
                // No pending messages to retry, check again in 500ms
                retransmittReady = millis() + 500;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Yield CPU to other tasks and prevent starvation
    }
}
