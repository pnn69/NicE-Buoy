#include "sercom.h"
#include "io_top.h"
#include "main.h"
#include <HardwareSerial.h>

QueueHandle_t serOut;
QueueHandle_t serIn;

static RoboStruct serDataOut;
static RoboStruct serDataIn;
static unsigned long lastSerMsg;

void initserqueue(void)
{
    serOut = xQueueCreate(10, sizeof(RoboStruct));
    serIn = xQueueCreate(10, sizeof(RoboStruct));
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
            else
            {
                Serial.println("crc error: " + serStringIn);
            }
        }
        if (xQueueReceive(serOut, (void *)&serDataOut, 0) == pdTRUE) // send data to bottom
        {
            String out = rfCode(serDataOut);
            Serial1.println(out);
            delay(2);
            while (Serial1.available())
            {
                Serial1.read();
            }
        }
        delay(1);
    }
}
