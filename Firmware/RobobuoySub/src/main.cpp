#include <Arduino.h>
#include "main.h"
#include "freertos/task.h"
#include "io_sub.h"
#include "subwifi.h"
#include "datastorage.h"
#include "leds.h"
#include "esc.h"
#include "compass.h"
static int8_t buoyId;
#define HOST_NAME "RoboBuoySub"
static UdpData mainUdpOutMsg;

void setup()
{
    Serial.begin(115200);
    delay(100);
    printf("\r\nSetup running!\r\n");
    printf("Robobuoy Sub Version: %0.1f\r\n", SUBVERSION);
    pinMode(BUZZER_PIN, OUTPUT);
    initMemory();
    // buoyId = 2;
    memBuoyId(&buoyId, true);
    Serial.print("Buoy ID: ");
    Serial.println(buoyId);
    initledqueue();
    initescqueue();
    initwifiqueue();
    xTaskCreatePinnedToCore(EscTask, "EscTask", 2400, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    InitCompass();
}

void loop(void)
{
    unsigned long nextSamp = millis();
    float vbat = 0;
    int speedbb = 50, speedsb = 0;
    while (true)
    {
        if (nextSamp < millis())
        {
            nextSamp = 500 + millis();
            float dir = GetHeading();
            if (udpOut != NULL && uxQueueSpacesAvailable(udpOut) > 0)
            {
                sprintf(mainUdpOutMsg.msg, "$HDG,%05.1f,20*", dir);
                mainUdpOutMsg.port = 1001;
                xQueueSend(udpOut, (void *)&mainUdpOutMsg, 10); // update WiFi
            }
            printf("Heding= %03.2f\r\n", dir);
        }
        vTaskDelay(1);
    }
}