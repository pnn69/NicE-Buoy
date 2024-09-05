#include <Arduino.h>
#include "../../RobobuoyDependency\RobobuoyVersion.h"
#include "../../RobobuoyDependency\RobobuoyMsg.h"
#include "../../RobobuoyDependency\RobobuoyDefaults.h"
#include "../../RobobuoyDependency\RobobuoyCalc.h"
#include "io_top.h"
#include "robolora.h"
#include "leds.h"
#include "topwifi.h"
#include "gps.h"
#include "datastorage.h"

static LedData mainCollorStatus;
static LedData mainCollorUtil;
static LedData mainCollorGps;
static UdpData mainUdpOutMsg;
static GpsDataType gpsStatus;
int8_t buoyId;
unsigned long udpTimer = millis();

void setup()
{
    Serial.begin(115200);
    delay(100);
    printf("\r\nSetup running!\r\n");
    printf("Robobuoy Top Version: %0.1f\r\n", TOPVERSION);
    initMemory();
    // buoyId = 1;
    // memBuoyId(&buoyId, false);

    initledqueue();
    initwifiqueue();
    initgpsqueue();
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4024, NULL, 20, NULL, 1);
    xTaskCreatePinnedToCore(GpsTask, "GpsTask", 2000, NULL, configMAX_PRIORITIES - 8, NULL, 1);
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 2000, NULL, configMAX_PRIORITIES - 5, NULL, 0);
    memBuoyId(&buoyId, true);
    Serial.print("Buoy ID: ");
    Serial.println(buoyId);
    pinMode(BUZZER_PIN, OUTPUT);
    Serial.println("Main task running!");
    mainCollorStatus.color = CRGB ::OrangeRed;
    mainCollorStatus.blink = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&mainCollorStatus, 10); // update util led
}

void loop(void)
{
    sprintf(mainUdpOutMsg.msg, "Hello world");
    mainUdpOutMsg.port = 0;
    xQueueSend(udpOut, (void *)&mainUdpOutMsg, 10); // update WiFi
    while (true)
    {
        if (udpTimer < millis())
        {
            udpTimer = millis() + 500;
            if (udpOut != NULL && uxQueueSpacesAvailable(udpOut) > 0)
            {
                sprintf(mainUdpOutMsg.msg, "ping");
                mainUdpOutMsg.port = 1001;
                xQueueSend(udpOut, (void *)&mainUdpOutMsg, 10); // update WiFi
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
    delay(1);
}
