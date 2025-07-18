#include "sercom.h"
#include "io_sub.h"
#include "main.h"
#include "subwifi.h"
#include <HardwareSerial.h>

QueueHandle_t serOut;
QueueHandle_t serIn;
static unsigned long mac;

static RoboStruct serDataOut;
static RoboStruct serDataIn;
RoboStruct pendingMsg[10] = {};
static unsigned long lastSerMsg;
static unsigned long retransmittReady = 0;

void initserqueue(void)
{
    serOut = xQueueCreate(10, sizeof(RoboStruct));
    serIn = xQueueCreate(10, sizeof(RoboStruct));
    mac = espMac();
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
        if (pendingMsg[i].cmd == ackBuffer.cmd)
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
void SercomTask(void *arg)
{
    unsigned long lastRx = millis();
    mac = espMac();
    delay(2000);
    // Serial1.begin(BAUDRATE, SERIAL_8N1, COM_PIN_RX, COM_PIN_TX, LEVEL); // Half-duplex on same pin
    Serial1.begin(230400, SERIAL_8N1, COM_PIN_RX, COM_PIN_TX, LEVEL); // Half-duplex on same pin
    while (1)
    {
        //***************************************************************************************************
        // Recieving data from PC
        //***************************************************************************************************
        if (Serial.available()) // recieve data form top
        {
            String serStringIn = "";
            while (Serial.available())
            {
                serStringIn += (char)Serial.read();
            }
            RoboStruct serDataIn;
            rfDeCode(serStringIn, &serDataIn);
            if (serDataIn.IDs != -1 && serDataIn.IDs != mac) // ignore own messages
            {
                xQueueSend(serIn, (void *)&serDataIn, 10); // notify main there is new data
                lastSerMsg = millis();
            }
        }
        //***************************************************************************************************
        // Recieving data from Top
        //***************************************************************************************************
        if (Serial1.available()) // recieve data form top
        {
            String serStringIn = "";
            while (Serial1.available())
            {
                serStringIn += (char)Serial1.read();
            }
            RoboStruct serDataIn;
            rfDeCode(serStringIn, &serDataIn);
            if (serDataIn.IDs != -1 && serDataIn.IDs != mac) // ignore own messages
            {
                //Serial.print("RxSub " + serStringIn);
                lastRx = millis();
                if (serDataIn.ack == LORAACK) // A message form me so check if its a ACK message
                {
                    printf("ack recieved removing cmd\r\n");
                    removeAckMsg(serDataIn);
                }
                else
                {
                    xQueueSend(serIn, (void *)&serDataIn, 10); // notify main there is new data
                    lastSerMsg = millis();
                }
            }
            if (serDataIn.ack == LORAGETACK) // on ack request send ack back
            {
                // IDr,IDs,ACK,MSG
                serDataIn.IDr = serDataIn.IDs;
                serDataIn.IDs = mac;
                serDataIn.ack = LORAACK;
                xQueueSend(serOut, (void *)&serDataIn, 10); // send ACK out
                printf("sending ack\r\n");
            }
        }
        //***************************************************************************************************
        // Sending data to Top
        //***************************************************************************************************
        if (xQueueReceive(serOut, (void *)&serDataOut, 0) == pdTRUE) // send data to bottom
        {
            while (lastRx + 20 > millis())
                ;
            serDataOut.IDs = mac; // set my ID
            String out = rfCode(&serDataOut);
            Serial1.println(out);
            if (serDataOut.ack == LORAGETACK)
            {
                serDataOut.retry = 5;
                storeAckMsg(serDataOut);                            // put data in buffer (will be removed on ack)
                retransmittReady = millis() + 750 + random(0, 150); // give some time for ack
            }
        }

        //***************************************************************************************************
        //  retry one time
        //***************************************************************************************************
        if (retransmittReady < millis())
        {
            serDataOut = chkAckMsg();
            if (serDataOut.cmd != 0)
            {
                while (lastRx + 20 > millis())
                    ;
                String out = rfCode(&serDataOut);
                Serial1.println(out);
                retransmittReady = millis() + random(1200, 750);
            }
        }
        delay(1);
    }
}
