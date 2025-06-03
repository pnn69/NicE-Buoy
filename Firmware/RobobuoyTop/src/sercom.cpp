#include "sercom.h"
#include "io_top.h"
#include "main.h"
#include "topwifi.h"
#include <HardwareSerial.h>

QueueHandle_t serOut;
QueueHandle_t serIn;
static unsigned long mac;
static RoboStruct serDataOut;
static RoboStruct serDataIn;
RoboStruct pendingMsgSer[10] = {};
static unsigned long lastSerMsg;
static unsigned long retransmittReady = 0;

void initserqueue(void)
{
    serOut = xQueueCreate(10, sizeof(RoboStruct));
    serIn = xQueueCreate(10, sizeof(RoboStruct));
    mac = espMac();
}

void PowerOnSub(void)
{
    pinMode(COM_PIN_TX, OUTPUT);
    digitalWrite(COM_PIN_TX, 1);
    delay(500);
    Serial1.begin(460800, SERIAL_8N1, COM_PIN_RX, COM_PIN_TX, true); // RX on GPIO 32, TX on GPIO 32 (Only one wire)
    delay(1500);
}

//***************************************************************************************************
//  Store to ack buffer
//***************************************************************************************************
void storeAckMsgSer(RoboStruct ackBuffer)
{
    int i = 0;
    while (i < 10)
    {
        if (pendingMsgSer[i].cmd == 0)
        {
            memcpy(&pendingMsgSer[i], &ackBuffer, sizeof(ackBuffer));
            pendingMsgSer[i] = ackBuffer;
            // Serial.println("message stored on pos:" + String(i) + " rettrys left:" + ackBuffer.retry + " for:" + String(ackBuffer.IDr, HEX) + " msg:" + ackBuffer.cmd);
            return;
        }
        i++;
    }
}

//***************************************************************************************************
//  Remove to ack buffer
//***************************************************************************************************
void removeAckMsgSer(RoboStruct ackBuffer)
{
    // Serial.println("looking for msg:" + String(ackBuffer.cmd) + " Id:" + String(ackBuffer.IDs, HEX));
    int i = 0;
    while (i < 10)
    {
        // Serial.println("Found:" + String(pendingMsgSer[i].cmd) + " Id:" + String(pendingMsgSer[i].IDr, HEX));
        if (pendingMsgSer[i].cmd == ackBuffer.cmd)
        {
            pendingMsgSer[i].ack = 0;
            pendingMsgSer[i].cmd = 0;
            pendingMsgSer[i].IDs = 0;
            pendingMsgSer[i].IDr = 0;
            pendingMsgSer[i].retry = 0;
            return;
        }
        i++;
    }
}

//***************************************************************************************************
//  check ack buffer
//  remove data if more than n times has failed
//***************************************************************************************************
RoboStruct chkAckMsgSer(void)
{
    RoboStruct in;
    in.ack = 0;
    in.cmd = 0;
    int i = 0;
    while (i < 10)
    {
        if (pendingMsgSer[i].cmd != 0)
        {
            in = pendingMsgSer[i];
            pendingMsgSer[i].retry--;
            if (pendingMsgSer[i].retry == 0)
            {
                pendingMsgSer[i].cmd = 0;
                pendingMsgSer[i].ack = 0;
                pendingMsgSer[i].IDs = 0;
                pendingMsgSer[i].IDr = 0;
            }
            return in;
        }
        i++;
    }
    return in;
}

void SercomTask(void *arg)
{
    char buff[50];
    pinMode(COM_PIN_TX, OUTPUT);
    digitalWrite(COM_PIN_TX, 1);
    delay(2000);
    Serial1.begin(460800, SERIAL_8N1, COM_PIN_RX, COM_PIN_TX, true); // RX on GPIO 32, TX on GPIO 32 (Only one wire)
    // Serial1.begin(230400, SERIAL_8N1, COM_PIN_RX, COM_PIN_TX, true); // RX on GPIO 32, TX on GPIO 32 (Only one wire)
    while (1)
    {
        // //***************************************************************************************************
        // // Recieving data from pc
        // //***************************************************************************************************
        if (Serial.available()) // recieve data form top
        {
            String serStringIn = "";
            while (Serial.available())
            {
                serStringIn += (char)Serial.read();
            }
            RoboStruct serDataIn = rfDeCode(serStringIn);
            if (serDataIn.IDs != -1)
            {
                xQueueSend(serIn, (void *)&serDataIn, 10); // notify main there is new data
                lastSerMsg = millis();
            }
        }
        //***************************************************************************************************
        // Recieving data from sub
        //***************************************************************************************************
        if (Serial1.available()) // recieve data form top
        {
            String serStringIn = "";
            while (Serial1.available())
            {
                serStringIn += (char)Serial1.read();
            }
            RoboStruct serDataIn = rfDeCode(serStringIn);
            if (serDataIn.IDs != -1)
            {
                xQueueSend(serIn, (void *)&serDataIn, 10); // notify main there is new data
                lastSerMsg = millis();
            }
            if (serDataIn.ack == LORAACK) // A message form me so check if its a ACK message
            {
                printf("ack recieved removing cmd\r\n");
                removeAckMsgSer(serDataIn);
            }
            if (serDataIn.ack == LORAGETACK) // on ack request send ack back
            {
                // IDr,IDs,ACK,MSG
                serDataIn.IDr = serDataIn.IDs;
                serDataIn.IDs = mac;
                serDataIn.ack = LORAACK;
                xQueueSend(serOut, (void *)&serDataIn, 10); // send ACK out
            }
        }
        //***************************************************************************************************
        // Sending data to sub
        //***************************************************************************************************
        if (xQueueReceive(serOut, (void *)&serDataOut, 0) == pdTRUE) // send data to bottom
        {
            String out = rfCode(serDataOut);
            Serial1.println(out);
            delay(2);
            while (Serial1.available())
            {
                Serial1.read();
            }
            if (serDataOut.ack == LORAGETACK)
            {
                serDataOut.retry = 5;
                storeAckMsgSer(serDataOut);                         // put data in buffer (will be removed on ack)
                retransmittReady = millis() + 500 + random(0, 150); // give some time for ack
            }
        }

        //***************************************************************************************************
        //  retry one time
        //***************************************************************************************************
        if (retransmittReady < millis())
        {
            serDataOut = chkAckMsgSer();
            if (serDataOut.cmd != 0)
            {
                String loraString = rfCode(serDataOut);
                Serial1.println(loraString);
                Serial.print("Resending: ");
                Serial.println(loraString);
                delay(2);
                while (Serial1.available())
                {
                    Serial1.read();
                }
                retransmittReady = millis() + random(300, 750);
            }
        }
        delay(1);
    }
}
