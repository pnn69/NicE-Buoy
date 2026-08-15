#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "cyd_lora.h"
#include "cyd_display.h"
#include "buoy_data.h"
#include "cyd_wifi.h"
#include "RGBledDriver.h" // Include LED driver header for blink capabilities!

#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS 5
#define LORA_RST 27
#define LORA_DIO0 35

#define LORA_BAND 433E6

SPIClass loraSpi(VSPI);

void init_lora() {
    Serial.println("Initializing LoRa on SD-Card SPI Pins...");
    
    loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LoRa.setSPI(loraSpi);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(LORA_BAND)) {
        Serial.println("Starting LoRa failed!");
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.setTextSize(1);
        tft.setTextDatum(BC_DATUM);
        tft.drawString("LoRa Init Failed!", tft.width() / 2, tft.height() - 10);
    } else {
        Serial.println("LoRa Init Success! Enabling Hardware CRC...");
        LoRa.enableCrc(); // Enable receiver hardware CRC checking to match RoboLora
        
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextSize(1);
        tft.setTextDatum(BC_DATUM);
        tft.drawString("LoRa Active (433MHz)", tft.width() / 2, tft.height() - 10);
    }
}

void check_lora_packets() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // Active RF Blinker indicator: Blink GREEN on reception
        ChangeRGBColor(RGB_COLOR_2); // Green
        
        // Read the length prefix byte first (custom RoboLora protocol)
        byte incomingLength = LoRa.read();
        
        String message = "";
        while (LoRa.available()) {
            message += (char)LoRa.read();
        }
        Serial.print("Received LoRa packet (length: ");
        Serial.print(incomingLength);
        Serial.print("): ");
        Serial.println(message);
        
        // Fetch active packet RSSI directly from the Ra-02 LoRa radio
        int rssi = LoRa.packetRssi();

        // Parse incoming packet and update our buoy data structure
        parse_buoy_packet(message, "LoRa", rssi);

        // Broadcast over WebSockets so the webpage shows real-time LoRa telemetry!
        broadcast_websocket_lora(message, rssi);
        
        // Brief delay to make the Green flash visually noticeable, then restore status LED
        delay(40);
        if (selected_buoy_idx == -1) {
            ChangeRGBColor(RGB_COLOR_2); // Back to Green status (Main Menu)
        } else {
            ChangeRGBColor(RGB_COLOR_3); // Back to Blue status (Nav / Setup)
        }
    }
}

void send_lora_packet(const String &message) {
    Serial.print("Sending LoRa packet: ");
    Serial.println(message);
    
    // Active RF Blinker indicator: Blink RED on transmission
    ChangeRGBColor(RGB_COLOR_1); // Red
    
    LoRa.beginPacket();
    LoRa.write(message.length()); // Send the length byte first, as expected by RoboLora receivers!
    LoRa.print(message);
    LoRa.endPacket();
    
    // Brief delay to make the Red flash visually noticeable, then restore status LED
    delay(40);
    if (selected_buoy_idx == -1) {
        ChangeRGBColor(RGB_COLOR_2); // Back to Green status (Main Menu)
    } else {
        ChangeRGBColor(RGB_COLOR_3); // Back to Blue status (Nav / Setup)
    }
}
