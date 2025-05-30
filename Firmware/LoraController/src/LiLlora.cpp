/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
*/
#include <Arduino.h>
#include <SPI.h> // include libraries
#include <LoRa.h>
#include "io.h"
#include "LiLlora.h"
#include "RoboCompute.h"
#include "oled_ssd1306.h"
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
QueueHandle_t loraOutSerial; // Queue for serial output

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
    loraOutSerial = xQueueCreate(10, sizeof(String));
    loraToMain = xQueueCreate(10, sizeof(RoboStruct));
    InitLora();
    Serial.print("#BuoyId=");
    Serial.println(espMac(), HEX);
    buoyId = espMac();
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
            //Serial.println("message stored on pos:" + String(i) + " rettrys left:" + ackBuffer.retry + " for:" + String(ackBuffer.IDr, HEX) + " msg:" + ackBuffer.cmd);
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
    //Serial.println("looking for msg:" + String(ackBuffer.cmd) + " Id:" + String(ackBuffer.IDs, HEX));
    int i = 0;
    while (i < 10)
    {
        //Serial.println("Found:" + String(pendingMsg[i].cmd) + " Id:" + String(pendingMsg[i].IDr, HEX));
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
    digitalWrite(LED_PIN, LOW); // turn on led
    if (incomingLength != incoming.length())
    { // check length for error
        Serial.println("#error: message length does not match length");
        return; // skip rest of function
    }
    RoboStruct in = rfDeCode(incoming);
    if ((in.IDr == buoyId || in.IDr == 0x99) && in.ack == LORAACK) // A message form me so check if its a ACK message
    {
        removeAckMsg(in);
        //printf("#Lora Ack recieved buffer cleared\r\n");
        return;
    }
    Serial.println(incoming);
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
    }
    xQueueSend(loraToMain, (void *)&in, 10); // send to main
}

//***************************************************************************************************
//  Lora data out
//***************************************************************************************************
bool sendLora(String loraTransmitt)
{
    if (transmittReady < millis())
        if (LoRa.beginPacket()) // start packet
        {
            digitalWrite(LED_PIN, HIGH); // turn on led
            LoRa.write(loraTransmitt.length());
            LoRa.print(loraTransmitt);
            LoRa.endPacket(); // finish packet and send it
            digitalWrite(LED_PIN, LOW); // turn on led
            transmittReady = millis() + 10;
            return true; // return true if success
        }
    return false;
}

//***************************************************************************************************
//  Lora task
//***************************************************************************************************
void LoraTask(void *arg)
{
    unsigned long retransmittReady = 0;
    String serDataOut = "";
    delay(500);
    while (1)
    {
        onReceive(LoRa.parsePacket()); // check if there is new data availeble
        // data to send from serial
        if (xQueueReceive(loraOutSerial, (void *)&serDataOut, 0) == pdTRUE) // send data from serial to lora
        {
            while (sendLora(serDataOut) != true)
            {
                vTaskDelay(pdTICKS_TO_MS(50));
            }
            RoboStruct loraMsgin = rfDeCode(serDataOut);
            if (loraMsgin.ack == LORAGETACK)
            {
                loraMsgin.retry = 5;
                storeAckMsg(loraMsgin);                              // put data in buffer (will be removed on ack)
                retransmittReady = millis() + 500 + random(0, 150); // give some time for ack
            }
        }
        // data to send from main
        if (xQueueReceive(loraOut, (void *)&loraMsgout, 10) == pdTRUE)
        {
            loraMsgout.IDs = espMac();
            String loraString = rfCode(loraMsgout);
            while (sendLora(String(loraString)) != true)
            {
                vTaskDelay(pdTICKS_TO_MS(150));
            }
            Serial.println(loraString);
            if (loraMsgout.ack == LORAGETACK)
            {
                loraMsgout.retry = 5;
                storeAckMsg(loraMsgout);                             // put data in buffer (will be removed on ack)
                retransmittReady = millis() + 500 + random(0, 150); // give some time for ack
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
                retransmittReady = millis() + 900 + random(0, 150);
            }
        }
    }
    vTaskDelay(1);
}
