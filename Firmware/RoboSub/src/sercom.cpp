#include "sercom.h"
#include "io_sub.h"
#include "main.h"
#include "subwifi.h"
#include <HardwareSerial.h>
#include "compass.h"
#include "datastorage.h"

QueueHandle_t serOut;
QueueHandle_t serIn;
static RoboStruct serDataOut;
static RoboStruct serDataIn;
RoboStruct pendingMsg[10] = {};
static unsigned long lastSerMsg;
static unsigned long retransmittReady = 0;
static unsigned long mac;

void initserqueue(void)
{
    serOut = xQueueCreate(10, sizeof(RoboStruct));
    serIn = xQueueCreate(10, sizeof(RoboStruct));
    mac = espMac();
}

void printMatrix()
{
    extern RoboStruct mainData;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Serial.print(mainData.magSoft[i][j], 4);
            Serial.print(j < 2 ? ", " : "\n");
        }
    }
}

void storeAckMsg(RoboStruct ackBuffer)
{
    for (int i = 0; i < 10; i++)
    {
        if (pendingMsg[i].cmd == ackBuffer.cmd && pendingMsg[i].IDr == ackBuffer.IDr)
        {
            pendingMsg[i] = ackBuffer;
            pendingMsg[i].retry = 5;
            printf("ACK_STORE: Updated msg cmd=%d for IDr=%X at pos=%d\r\n", ackBuffer.cmd, ackBuffer.IDr, i);
            return;
        }
    }

    for (int i = 0; i < 10; i++)
    {
        if (pendingMsg[i].cmd == 0)
        {
            pendingMsg[i] = ackBuffer;
            printf("ACK_STORE: Stored msg cmd=%d for IDr=%X at pos=%d\r\n", ackBuffer.cmd, ackBuffer.IDr, i);
            return;
        }
    }
    printf("ACK_STORE: ERROR - Buffer full!\r\n");
}

void removeAckMsg(RoboStruct ackBuffer)
{
    bool found = false;
    for (int i = 0; i < 10; i++)
    {
        if (pendingMsg[i].cmd != 0 && pendingMsg[i].cmd == ackBuffer.cmd)
        {
            printf("ACK_REMOVE: Removed msg cmd=%d from pos=%d\r\n", pendingMsg[i].cmd, i);
            pendingMsg[i].ack = 0;
            pendingMsg[i].cmd = 0;
            pendingMsg[i].IDs = 0;
            pendingMsg[i].IDr = 0;
            pendingMsg[i].retry = 0;
            found = true;
        }
    }
    if (!found) {
        printf("ACK_REMOVE: No pending msg found for cmd=%d\r\n", ackBuffer.cmd);
    }
}

RoboStruct chkAckMsg(void)
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
                printf("ACK_RETRY: Failed after max retries cmd=%d\r\n", pendingMsg[i].cmd);
                pendingMsg[i].cmd = 0;
                pendingMsg[i].ack = 0;
                pendingMsg[i].IDs = 0;
                pendingMsg[i].IDr = 0;
            } else {
                printf("ACK_RETRY: Resending cmd=%d to IDr=%X, retries left=%d\r\n", in.cmd, in.IDr, pendingMsg[i].retry);
            }
            return in;
        }
    }
    return in;
}

void SercomTask(void *arg)
{
    unsigned long lastRx = millis();
    mac = espMac();
    delay(2000);
    Serial1.begin(BAUDRATE, SERIAL_8N1, COM_PIN_RX, COM_PIN_TX, LEVEL);
    Serial1.setTimeout(100);
    Serial.setTimeout(100);
    while (1)
    {
        if (Serial.available())
        {
            String serStringIn = Serial.readStringUntil('\n');
            serStringIn.trim();
            char buffer[100];
            if (serStringIn.startsWith("SoftI:"))
            {
                serStringIn.remove(0, 6);
                serStringIn.toCharArray(buffer, sizeof(buffer));
                int index = 0;
                char *token = strtok(buffer, ",");
                while (token != NULL && index < 9)
                {
                    int row = index / 3;
                    int col = index % 3;
                    serDataIn.magSoft[row][col] = atof(token);
                    token = strtok(NULL, ",");
                    index++;
                }
                if (index == 9)
                {
                    softIron(&serDataIn, SET);
                    InitCompass();
                }
            }
            else if (serStringIn.startsWith("HardI:"))
            {
                serStringIn.remove(0, 6);
                serStringIn.toCharArray(buffer, sizeof(buffer));
                int index = 0;
                char *token = strtok(buffer, ",");
                while (token != NULL && index < 3)
                {
                    serDataIn.magHard[index] = atof(token);
                    token = strtok(NULL, ",");
                    index++;
                }
                if (index == 3)
                {
                    hardIron(&serDataIn, SET);
                    InitCompass();
                }
            }
            else
            {
                RoboStruct serDataIn;
                rfDeCode(serStringIn, &serDataIn);
                if (serDataIn.IDs != -1 && serDataIn.IDs != mac)
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
                if (serDataIn.IDs != -1 && serDataIn.IDs != mac)
                {
                    lastRx = millis();
                    if (serDataIn.ack == LORAACK)
                    {
                        printf("SER_TOP_ACK received cmd=%d from IDs=%X\r\n", serDataIn.cmd, serDataIn.IDs);
                        removeAckMsg(serDataIn);
                    }
                    else
                    {
                        printf("SER_TOP_IN CMD=%d from IDs=%X\n", serDataIn.cmd, serDataIn.IDs);
                        xQueueSend(serIn, (void *)&serDataIn, 10);
                        lastSerMsg = millis();
                        
                        if (serDataIn.ack == LORAGETACK)
                        {
                            serDataIn.IDr = serDataIn.IDs;
                            serDataIn.IDs = mac;
                            serDataIn.ack = LORAACK;
                            xQueueSend(serOut, (void *)&serDataIn, 10);
                            printf("SER_TOP_ACK_SEND cmd=%d to IDr=%X\r\n", serDataIn.cmd, serDataIn.IDr);
                        }
                    }
                }
            }
        }

        if (xQueueReceive(serOut, (void *)&serDataOut, 0) == pdTRUE)
        {
            while (lastRx + 20 > millis())
                vTaskDelay(pdMS_TO_TICKS(1));
            if (serDataOut.IDs == 0) serDataOut.IDs = mac;
            String out = rfCode(&serDataOut);
            Serial1.println(out);
            printf("SER_TOP_OUT>%s<\r\n", out.c_str());
            if (serDataOut.ack == LORAGETACK)
            {
                serDataOut.retry = 5;
                storeAckMsg(serDataOut);
                retransmittReady = millis() + 1000;
            }
        }

        if (retransmittReady != 0 && retransmittReady < millis())
        {
            serDataOut = chkAckMsg();
            if (serDataOut.cmd != 0)
            {
                while (lastRx + 20 > millis())
                    vTaskDelay(pdMS_TO_TICKS(1));
                String out = rfCode(&serDataOut);
                Serial1.println(out);
                printf("SER_TOP_RETRY>%s<\r\n", out.c_str());
                retransmittReady = millis() + 1000;
            } else {
                retransmittReady = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
