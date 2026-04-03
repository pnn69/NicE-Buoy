#include "sercom.h"
#include "io_top.h"
#include "main.h"
#include "topwifi.h"
#include <HardwareSerial.h>
#include "robotone.h"
#include "buzzer.h"

QueueHandle_t serOut;
QueueHandle_t serIn;
static unsigned long mac;

static RoboStruct serDataOut;
static RoboStruct serDataIn;
RoboStruct pendingMsg[10] = {};
static unsigned long lastSerMsg = 0;
static unsigned long retransmittReady = 0;
static bool serialTimeout = true;

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
/**
 * @brief Stores a message in the pending queue awaiting an acknowledgment over serial.
 * 
 * @param ackBuffer The message structure to store.
 */
void SerstoreAckMsg(RoboStruct ackBuffer)
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
/**
 * @brief Removes a message from the pending queue when its ACK is received over serial.
 * 
 * @param ackBuffer The acknowledgment message containing the matching cmd.
 */
void SerremoveAckMsg(RoboStruct ackBuffer)
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
/**
 * @brief Checks the pending message queue for timeouts and decrements retries over serial.
 * 
 * @return A RoboStruct containing a message that needs retransmission, or an empty
 * struct if nothing needs retransmitting.
 */
RoboStruct SerchkAckMsg(void)
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

/**
 * @brief FreeRTOS task handling all half-duplex serial communication.
 * 
 * Manages communication with the Sub unit via Serial1 and the PC via Serial.
 * Parses incoming strings, decodes them into RoboStructs, and routes them to 
 * the appropriate queues. Also handles retransmissions for unacknowledged messages.
 * 
 * @param arg Task arguments (unused).
 */
void SercomTask(void *arg)
{
    unsigned long lastRx = millis();
    mac = espMac();
    delay(2000);
    beep(5, buzzer);
    // Serial1.begin(BAUDRATE, SERIAL_8N1, COM_PIN_RX, COM_PIN_TX, LEVEL); // Half-duplex on same pin
    Serial1.begin(230400, SERIAL_8N1, COM_PIN_RX, COM_PIN_TX, LEVEL); // Half-duplex on same pin
    Serial1.setTimeout(100);
    Serial.setTimeout(100);
    while (1)
    {
        //***************************************************************************************************
        // Recieving data from PC
        //***************************************************************************************************
        if (Serial.available()) // recieve data form pc
        {
            String serStringIn = "";
            serStringIn = Serial.readStringUntil('\n');
            serStringIn.trim(); // Remove whitespace and \r\n
            char buffer[100];   // Safe copy buffer
            // SoftI:S11,S12,S13,S21,S22,S23,S31,S32,S33
            // HardI:H1,H2,H3
            // Example: SoftI:0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9
            // Example: HardI:-30,-45,37
            serStringIn.trim(); // Remove leading/trailing whitespace
            //Serial.print("RxPC: " +serStringIn + "\r\n");
            if (serStringIn.startsWith("SoftI:"))
            {
                serStringIn.remove(0, 6);                        // Remove prefix
                serStringIn.toCharArray(buffer, sizeof(buffer)); // Copy to char array
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
                if (index == 9) // 9 values for soft iron correction
                {
                    // tranfer to sub
                    serDataIn.cmd = SOFTIRONFACTORS;
                    serDataIn.ack = LORAGETACK;
                    xQueueSend(serOut, (void *)&serDataIn, 10); // send out trough serial port
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
                if (index == 3) // 3 values for hard iron correction
                {
                    // tranfer to sub
                    serDataIn.cmd = HARDIRONFACTORS;
                    serDataIn.ack = LORAGETACK;
                    xQueueSend(serOut, (void *)&serDataIn, 10); // send out trough serial port
                }
            }
            else
            {
                RoboStruct serDataIn;
                rfDeCode(serStringIn, &serDataIn);
                mac = espMac();
                if (serDataIn.IDs != -1 && serDataIn.IDs != mac) // ignore own messages
                {
                    xQueueSend(serIn, (void *)&serDataIn, 10); // notify main there is new data
                    lastSerMsg = millis();
                }
            }
        }
        //***************************************************************************************************
        // Recieving data from Sub
        //***************************************************************************************************
        if (Serial1.available()) // recieve data form sub
        {
            String serStringIn = Serial1.readStringUntil('\n');
            serStringIn.trim();
            if (serStringIn.length() > 0)
            {
                RoboStruct serDataIn;
                rfDeCode(serStringIn, &serDataIn);
                if (serDataIn.IDs != -1 && serDataIn.IDs != mac) // ignore own messages
                {
                    lastRx = millis();
                    if (serDataIn.ack == LORAACK) // A message form me so check if its a ACK message
                    {
                        printf("ack recieved removing cmd\r\n");
                        SerremoveAckMsg(serDataIn);
                    }
                    else
                    {
                        xQueueSend(serIn, (void *)&serDataIn, 10); // notify main there is new data
                        lastSerMsg = millis();
                    }
                }
                if (serDataIn.ack == LORAGETACK) // on ack request send ack back
                {
                    serDataIn.IDr = serDataIn.IDs;
                    serDataIn.IDs = mac;
                    serDataIn.ack = LORAACK;
                    xQueueSend(serOut, (void *)&serDataIn, 10); // send ACK out
                    printf("sending ack\r\n");
                }
            }
        }
        //***************************************************************************************************
        // Sending data to Sub
        //***************************************************************************************************
        if (xQueueReceive(serOut, (void *)&serDataOut, 0) == pdTRUE) // send data to bottom
        {
            if (serDataOut.cmd == WAKEUP)
            {
                Serial.println("Waking up Sub...");
                Serial1.end();
                pinMode(COM_PIN_TX, OUTPUT);
                // Based on LEVEL=true (inverted), Active state is HIGH
                digitalWrite(COM_PIN_TX, HIGH);
                vTaskDelay(pdMS_TO_TICKS(500));
                digitalWrite(COM_PIN_TX, LOW);
                Serial1.begin(230400, SERIAL_8N1, COM_PIN_RX, COM_PIN_TX, LEVEL);
            }
            else
            {
                while (lastRx + 20 > millis())
                    vTaskDelay(pdMS_TO_TICKS(1));
                
                if (serDataOut.IDs == 0)
                {
                    serDataOut.IDs = mac;
                }
                String out = rfCode(&serDataOut);
                Serial1.println(out);
                if (serDataOut.ack == LORAGETACK)
                {
                    serDataOut.retry = 5;
                    SerstoreAckMsg(serDataOut);                         // put data in buffer (will be removed on ack)
                    retransmittReady = millis() + 750 + random(0, 150); // give some time for ack
                }
            }
        }

        //***************************************************************************************************
        //  retry one time
        //***************************************************************************************************
        if (retransmittReady < millis())
        {
            serDataOut = SerchkAckMsg();
            if (serDataOut.cmd != 0)
            {
                while (lastRx + 20 > millis())
                    delay(1);
                String out = rfCode(&serDataOut);
                Serial1.println(out);
                retransmittReady = millis() + random(750, 1200);
            }
        }
        delay(1);
    }
}