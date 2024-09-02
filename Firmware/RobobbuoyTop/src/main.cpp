#include <Arduino.h>
#include "../../RobobuoyDependency\RobobuoyVersion.h"
#include "../../RobobuoyDependency\RobobuoyMsg.h"
#include "../../RobobuoyDependency\RobobuoyDefaults.h"
#include "../../RobobuoyDependency\RobobuoyCalc.h"
#include "io_top.h"
#include "robolora.h"
#include "leds.h"
#include "robowifi.h"
#include "gps.h"
#include "datastorage.h"

static LedData mainCollorStatus;
static LedData mainCollorUtil;
static LedData mainCollorGps;
static UdpData mainUdpOutMsg;
static GpsDataType gpsStatus;
int8_t buoyId;
unsigned long udpTimer = millis();
int pingConter = 0;

void setup()
{
    Serial.begin(115200);
    delay(100);
    printf("\r\nSetup running!\r\n");
    printf("Robobuoy Top Version: %0.1f\r\n",TOPVERSION);
    initMemory();
    /* run once
    buoyId = 4;
    memBuoyId(&buoyId, false);
    */

    initledqueue();
    initwifiqueue();
    initgpsqueue();
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4024, NULL, 20, NULL, 1);
    xTaskCreatePinnedToCore(GpsTask, "GpsTask", 2000, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8000, NULL, 10, NULL, 1);
    memBuoyId(&buoyId, true);
    Serial.print("Buoy ID: ");
    Serial.println(buoyId);
    pinMode(BUZZER_PIN, OUTPUT);
    Serial.println("Main task running!");
    mainCollorUtil.color = CRGB::Black;
    mainCollorUtil.blink = false;
    mainCollorStatus.color = CRGB ::Black;
    mainCollorStatus.blink = false;
    xQueueSend(ledUtil, (void *)&mainCollorUtil, 10);     // update util led
    xQueueSend(ledStatus, (void *)&mainCollorStatus, 10); // update util led
}

void loop(void)
{
    sprintf(mainUdpOutMsg.msg, "Hello world");
    mainUdpOutMsg.port = 1001;
    xQueueSend(udpOut, (void *)&mainUdpOutMsg, 10); // update WiFi
    while (true)
    {
        if (udpTimer < millis())
        {
            udpTimer = millis() + 1000;
            if (udpOut != NULL && uxQueueSpacesAvailable(udpOut) > 0)
            {
                sprintf(mainUdpOutMsg.msg, "ping %d",pingConter++);
                mainUdpOutMsg.port = 1001;
                mainCollorStatus.color = CRGB ::Red;
                mainCollorStatus.blink = false;
                xQueueSend(udpOut, (void *)&mainUdpOutMsg, 10);       // update WiFi
                xQueueSend(ledStatus, (void *)&mainCollorStatus, 10); // update util led
            }
            else
            {
                // printf("ping not in buffer!\r\n");
            }
        }
        if (xQueueReceive(gpsQue, (void *)&gpsStatus, 0) == pdTRUE) // New gps data
        {
        }
    }
    delay(100);
}
