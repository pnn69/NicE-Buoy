# RoboLora (Remote Controller Firmware)

The `RoboLora` firmware powers the handheld remote controller (based on ESP32 Pico32/LilyGo LoRa boards) used to manually monitor and steer the NicE-Buoys.

## Key Features
- **LoRa Telemetry:** Uses the SX1278 (or similar) LoRa module to communicate with the `RoboTop` units over long distances.
- **OLED Display:** Provides a real-time SSD1306 OLED interface displaying buoy status, target distance, speed, and battery levels.
- **Physical Controls:** Reads physical switches and potentiometers (via ADC) to allow manual steering (`REMOTE` mode) or state switching (e.g., to `LOCK` or `IDLE`).
- **Multi-Buoy Support:** Can cycle through and control up to 3 different buoys by managing target MAC/IDs (`IDr` / `IDs`).