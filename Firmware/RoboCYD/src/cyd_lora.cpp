#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "cyd_lora.h"
#include "cyd_display.h"
#include "buoy_data.h"
#include "cyd_wifi.h"
#include "boot_screen.h"
#include "RGBledDriver.h" // Include LED driver header for blink capabilities!

#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS 5
#define LORA_RST 27
#define LORA_DIO0 35

#define LORA_BAND 433E6

SPIClass loraSpi(VSPI);

// The radio is drained by its own task, not from loop().
//
// loop() also runs OTA, the web server and every screen repaint; a full redraw pushes a lot of
// pixels over SPI and takes far longer than one packet is on the air. The Tops send their idle
// telemetry as BUOYPOS immediately followed by TOPDATA, so the radio has to be read twice within
// ~130 ms - and the draw loop simply was not coming back that fast. The second frame of every
// pair was overwritten before it was ever read, which showed up at the display as 12 BUOYPOS and
// 0 TOPDATA per minute from a buoy that was sending them one for one.
//
// The task does nothing but move bytes off the radio into a queue. Parsing stays on the main
// loop, so buoys[] keeps a single writer from this path and the Arduino Strings inside it are
// never reallocated under the draw code's feet.
#define LORA_RX_MAXLEN 256
#define LORA_RX_QUEUE_DEPTH 8

typedef struct
{
    int rssi;
    char msg[LORA_RX_MAXLEN];
} LoraRxFrame;

static QueueHandle_t loraRxQueue = NULL;
// Guards every SX127x access. The receive task polls continuously while send_lora_packet() is
// called from the touch handlers on the main loop, and they share one SPI bus and one radio.
static SemaphoreHandle_t loraMutex = NULL;

static void LoraRxTask(void *arg)
{
    for (;;)
    {
        LoraRxFrame frame;
        bool got = false;

        if (xSemaphoreTake(loraMutex, portMAX_DELAY) == pdTRUE)
        {
            if (LoRa.parsePacket() > 0)
            {
                (void)LoRa.read(); // length prefix byte (custom RoboLora protocol)
                int n = 0;
                while (LoRa.available() && n < LORA_RX_MAXLEN - 1)
                {
                    frame.msg[n++] = (char)LoRa.read();
                }
                frame.msg[n] = '\0';
                frame.rssi = LoRa.packetRssi();
                got = (n > 0);
            }
            xSemaphoreGive(loraMutex);
        }

        // Dropped rather than blocked if the main loop has fallen behind: holding the radio
        // hostage waiting for queue space would lose the next frame off the air, which is the
        // exact failure this task exists to remove.
        if (got && xQueueSend(loraRxQueue, &frame, 0) != pdTRUE)
        {
            Serial.println("LoRa RX queue full - frame dropped");
        }

        vTaskDelay(1);
    }
}

void init_lora()
{
    Serial.println("Initializing LoRa on SD-Card SPI Pins...");

    loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LoRa.setSPI(loraSpi);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(LORA_BAND))
    {
        Serial.println("Starting LoRa failed!");
        // Reported through the splash rather than drawn here: bottom-centre is where the splash
        // puts its own indicators, so the old drawString landed straight on top of them.
        boot_screen_lora(false);
        boot_screen_status("LORA FAILED");
    }
    else
    {
        Serial.println("LoRa Init Success! Enabling Hardware CRC...");
        LoRa.enableCrc(); // Enable receiver hardware CRC checking to match RoboLora

        loraMutex = xSemaphoreCreateMutex();
        loraRxQueue = xQueueCreate(LORA_RX_QUEUE_DEPTH, sizeof(LoraRxFrame));
        // Core 0, away from the Arduino loop task on core 1, and above it in priority so a busy
        // repaint can never stall the radio. It sleeps on vTaskDelay between polls, so the cost
        // of the higher priority is nil.
        xTaskCreatePinnedToCore(LoraRxTask, "LoraRxTask", 8192, NULL, 3, NULL, 0);
        Serial.println("LoRa receive task started");

        boot_screen_lora(true);
    }
}

// Drain whatever LoraRxTask has captured since the last pass. Called from loop(); how late it
// runs no longer costs packets, because the radio was already emptied by the task. Loops rather
// than handling one per call so a slow repaint catches up in a single pass instead of letting
// the queue back up.
void check_lora_packets()
{
    if (loraRxQueue == NULL)
        return;

    LoraRxFrame frame;
    while (xQueueReceive(loraRxQueue, &frame, 0) == pdTRUE)
    {
        // Active RF Blinker indicator: Blink GREEN on reception
        ChangeRGBColor(RGB_COLOR_2); // Green

        String message = String(frame.msg);
        Serial.print("Received LoRa packet: ");
        Serial.println(message);

        // Parse incoming packet and update our buoy data structure
        parse_buoy_packet(message, "LoRa", frame.rssi);

        // Broadcast over WebSockets so the webpage shows real-time LoRa telemetry!
        broadcast_websocket_lora(message, frame.rssi);

        if (selected_buoy_idx == -1)
        {
            ChangeRGBColor(RGB_COLOR_2); // Back to Green status (Main Menu)
        }
        else
        {
            ChangeRGBColor(RGB_COLOR_3); // Back to Blue status (Nav / Setup)
        }
    }
}

void send_lora_packet(const String &message)
{
    Serial.print("Sending LoRa packet: ");
    Serial.println(message);

    // Turn the on-screen LoRa indicator dot RED for the duration of the transmit blink
    last_lora_tx_ms = millis();

    // Active RF Blinker indicator: Blink RED on transmission
    ChangeRGBColor(RGB_COLOR_1); // Red

    // Under the mutex: the receive task is polling the same radio over the same SPI bus, and
    // interleaving a parsePacket() with a half-written packet corrupts both.
    if (loraMutex)
        xSemaphoreTake(loraMutex, portMAX_DELAY);
    LoRa.beginPacket();
    LoRa.write(message.length()); // Send the length byte first, as expected by RoboLora receivers!
    LoRa.print(message);
    LoRa.endPacket();
    if (loraMutex)
        xSemaphoreGive(loraMutex);

    // Outside the mutex - this is only the LED flash, and holding the radio through it would
    // block the receive task for 40 ms, reintroducing the very stall this design removed.
    delay(40);
    if (selected_buoy_idx == -1)
    {
        ChangeRGBColor(RGB_COLOR_2); // Back to Green status (Main Menu)
    }
    else
    {
        ChangeRGBColor(RGB_COLOR_3); // Back to Blue status (Nav / Setup)
    }
}
