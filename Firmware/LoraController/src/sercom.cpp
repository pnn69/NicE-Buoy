#include "sercom.h"
#include "main.h"
#include <HardwareSerial.h>
#include "LoRa.h"
#include "oled_ssd1306.h"
#include "LiLlora.h"

QueueHandle_t serOut;
QueueHandle_t serIn;

static RoboStruct serDataOut;
static RoboStruct serDataIn;
static unsigned long lastSerMsg;
static String serStringOut = "";
void initserqueue(void)
{
    serOut = xQueueCreate(10, sizeof(RoboStruct));
    serIn = xQueueCreate(10, sizeof(String));
}

void SercomTask(void *arg)
{
    char buff[50];
    String serStringIn = "";
    while (1)
    {
        if (Serial.available()) // recieve data form top
        {
            serStringIn = "";
            while (Serial.available())
            {
                serStringIn += (char)Serial.read();
            }
            xQueueSend(loraOutSerial, (void *)&serStringIn, 10); // send to lora
            RoboStruct serDataIn;
            rfDeCode(serStringIn,&serDataIn);
            if (serDataIn.IDs != -1)
            {
                xQueueSend(serIn, (void *)&serDataIn, 10); // notify main there is new data
                lastSerMsg = millis();
            }
        }
        delay(1);
    }
}
