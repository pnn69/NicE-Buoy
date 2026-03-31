#include "sercom.h"
#include "main.h"
#include <HardwareSerial.h>
#include "LoRa.h"
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
    while (1)
    {
        if (Serial.available()) // receive data from PC/RoboControl
        {
            String serStringIn = Serial.readStringUntil('\n');
            serStringIn.trim(); // Clean up any trailing whitespace
            
            if (serStringIn.length() > 0)
            {
                // Decode the string immediately to avoid passing String objects via Queue
                RoboStruct serDataIn;
                rfDeCode(serStringIn, &serDataIn);
                
                if (serDataIn.IDs != -1)
                {
                    // Route to LoRa transmitter queue
                    xQueueSend(loraOut, (void *)&serDataIn, 10);
                    
                    // Also route to main task for local monitoring/display
                    xQueueSend(serIn, (void *)&serDataIn, 10);
                    lastSerMsg = millis();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
