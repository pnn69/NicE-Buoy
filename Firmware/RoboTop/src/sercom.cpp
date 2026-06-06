#include "sercom.h"
#include "io_top.h"
#include "main.h"
#include "topwifi.h"
#include <HardwareSerial.h>
#include "robotone.h"
#include "buzzer.h"
#include <AsyncUDP.h>

QueueHandle_t serOut;
QueueHandle_t serIn;
static RoboStruct serDataOut;
static RoboStruct serDataIn;
RoboStruct pendingMsg[10] = {};
static unsigned long lastSerMsg = 0;
static unsigned long retransmittReady = 0;
static unsigned long mac;

/**
 * @brief Initializes the FreeRTOS queues for serial communication.
 */
void initserqueue(void)
{
    serOut = xQueueCreate(10, sizeof(RoboStruct));
    serIn = xQueueCreate(10, sizeof(RoboStruct));
    mac = espMac();
}

//***************************************************************************************************
//  Store to ack buffer
//***************************************************************************************************
void SerstoreAckMsg(RoboStruct ackBuffer)
{
    for (int i = 0; i < 10; i++)
    {
        if (pendingMsg[i].cmd == ackBuffer.cmd && pendingMsg[i].IDr == ackBuffer.IDr)
        {
            pendingMsg[i] = ackBuffer;
            pendingMsg[i].retry = 5;
            // printf("ACK_STORE: Updated msg cmd=%d for IDr=%X at pos=%d\r\n", ackBuffer.cmd, ackBuffer.IDr, i);
            return;
        }
    }

    for (int i = 0; i < 10; i++)
    {
        if (pendingMsg[i].cmd == 0)
        {
            pendingMsg[i] = ackBuffer;
            // printf("ACK_STORE: Stored msg cmd=%d for IDr=%X at pos=%d\r\n", ackBuffer.cmd, ackBuffer.IDr, i);
            return;
        }
    }
    // printf("ACK_STORE: ERROR - Buffer full!\r\n");
}

//***************************************************************************************************
//  Remove to ack buffer
//***************************************************************************************************
void SerremoveAckMsg(RoboStruct ackBuffer)
{
    bool found = false;
    for (int i = 0; i < 10; i++)
    {
        if (pendingMsg[i].cmd != 0 && pendingMsg[i].cmd == ackBuffer.cmd)
        {
            // printf("ACK_REMOVE: Removed msg cmd=%d from pos=%d\r\n", pendingMsg[i].cmd, i);
            pendingMsg[i].ack = 0;
            pendingMsg[i].cmd = 0;
            pendingMsg[i].IDs = 0;
            pendingMsg[i].IDr = 0;
            pendingMsg[i].retry = 0;
            found = true;
        }
    }
    if (!found) {
        // printf("ACK_REMOVE: No pending msg found for cmd=%d\r\n", ackBuffer.cmd);
    }
}

//***************************************************************************************************
//  check ack buffer
//***************************************************************************************************
RoboStruct SerchkAckMsg(void)
{
    RoboStruct in;
    in.ack = 0;
    in.cmd = 0;
    for (int i = 0; i < 10; i++)
    {
        if (pendingMsg[i].cmd != 0)
        {
            in = pendingMsg[i];
            pendingMsg[i].retry--;
            if (pendingMsg[i].retry == 0)
            {
                // printf("ACK_RETRY: Failed after max retries cmd=%d\r\n", pendingMsg[i].cmd);
                pendingMsg[i].cmd = 0;
                pendingMsg[i].ack = 0;
                pendingMsg[i].IDs = 0;
                pendingMsg[i].IDr = 0;
            } else {
                // printf("ACK_RETRY: Resending cmd=%d to IDr=%X, retries left=%d\r\n", in.cmd, in.IDr, pendingMsg[i].retry);
            }
            return in;
        }
    }
    return in;
}

/**
 * @brief FreeRTOS task handling all half-duplex serial communication.
 */
void SercomTask(void *arg)
{
    unsigned long lastRx = millis();
    mac = espMac();
    delay(2000);
    beep(5, buzzer);
    Serial1.begin(BAUDRATE, SERIAL_8N1, COM_PIN_RX, COM_PIN_TX, LEVEL);
    Serial1.setTimeout(100);
    Serial.setTimeout(100);
    while (1)
    {
        if (Serial.available())
        {
            String serStringIn = Serial.readStringUntil('\n');
            serStringIn.trim();
            if (serStringIn.length() > 0)
            {
                RoboStruct serDataIn;
                rfDeCode(serStringIn, &serDataIn);
                if (serDataIn.IDs != -1 && serDataIn.IDs != mac && serDataIn.IDs != 0x99)
                {
                    printf("SER_PC_IN CMD=%d\n", serDataIn.cmd); 
                    xQueueSend(serIn, (void *)&serDataIn, 10);
                }
            }
        }

        if (Serial1.available())
        {
            String serStringIn = Serial1.readStringUntil('\n');
            serStringIn.trim();
            if (serStringIn.length() > 0)
            {
                RoboStruct serDataIn;
                rfDeCode(serStringIn, &serDataIn);
                
                // Prevent processing our own echoed transmissions on the half-duplex line.
                // An echo has IDs matching our Top MAC and an ack type of GET, GETACK, or SET.
                bool is_echo = (serDataIn.IDs == mac && (serDataIn.ack == GET || serDataIn.ack == GETACK || serDataIn.ack == SET));
                
                if (serDataIn.IDs != -1 && serDataIn.IDs != 0x99 && !is_echo)
                {
                    lastRx = millis();
                    if (serDataIn.ack == ACK)
                    {
                        // printf("SER_SUB_ACK received cmd=%d from IDs=%X\r\n", serDataIn.cmd, serDataIn.IDs);
                        SerremoveAckMsg(serDataIn);
                    }
                    else
                    {
                        // printf("SER_SUB_IN CMD=%d from IDs=%X\n", serDataIn.cmd, serDataIn.IDs);
                        xQueueSend(serIn, (void *)&serDataIn, 10);
                        lastSerMsg = millis();

                        if (serDataIn.ack == GETACK)
                        {
                            serDataIn.IDr = serDataIn.IDs;
                            serDataIn.IDs = mac;
                            serDataIn.ack = ACK;
                            xQueueSend(serOut, (void *)&serDataIn, 10);
                            // printf("SER_SUB_ACK_SEND cmd=%d to IDr=%X\r\n", serDataIn.cmd, serDataIn.IDr);
                        }
                    }
                }
            }
        }

        if (xQueueReceive(serOut, (void *)&serDataOut, 0) == pdTRUE)
        {
            if (serDataOut.cmd == WAKEUP)
            {
                Serial.println("Waking up Sub...");
                Serial1.end();
                pinMode(COM_PIN_TX, OUTPUT);
                digitalWrite(COM_PIN_TX, HIGH);
                vTaskDelay(pdMS_TO_TICKS(500));
                digitalWrite(COM_PIN_TX, LOW);
                Serial1.begin(BAUDRATE, SERIAL_8N1, COM_PIN_RX, COM_PIN_TX, LEVEL);
            }
            else
            {
                while (lastRx + 20 > millis())
                    vTaskDelay(pdMS_TO_TICKS(1));

                serDataOut.IDs = mac;
                
                String out = rfCode(&serDataOut);
                Serial1.println(out);
                // printf("SER_SUB_OUT>%s<\r\n", out.c_str());

                if (serDataOut.ack == GETACK)
                {
                    serDataOut.retry = 5;
                    SerstoreAckMsg(serDataOut);
                    retransmittReady = millis() + 1000;
                }
            }
        }

        if (retransmittReady != 0 && retransmittReady < millis())
        {
            serDataOut = SerchkAckMsg();
            if (serDataOut.cmd != 0)
            {
                while (lastRx + 20 > millis())
                    delay(1);
                String out = rfCode(&serDataOut);
                Serial1.println(out);
                // printf("SER_SUB_RETRY>%s<\r\n", out.c_str());
                retransmittReady = millis() + 1000;
            } else {
                retransmittReady = 0;
            }
        }
        delay(1);
    }
}
