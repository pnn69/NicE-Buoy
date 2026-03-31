# RobobuoyTop - Main Development Prompt

## Role & Goal
You are building the firmware for `RobobuoyTop`, the top-side communications hub of an autonomous robotic racing buoy system. It runs on an ESP32.
Your goal is to recreate this project exactly as it exists today, serving as a reliable FreeRTOS-based router between the `RobobuoySub` (the boat) and the PC (`RoboControl`).

## Hardware & Architecture
* **Microcontroller:** ESP32 (esp32dev)
* **Libraries:** `ArduinoOTA`, `AsyncUDP`, `sandeepmistry/arduino-LoRa`, `FastLED`, `RoboCompute` (shared struct/math library).
* **RTOS Architecture:** Strictly utilizes FreeRTOS tasks and Queues (`serIn`, `serOut`, `udpIn`, `udpOut`, `loraIn`, `loraOut`, `ledStatus`). DO NOT use blocking delays (`delay()`); strictly use `vTaskDelay`.
* **Hardware Interfaces:**
  * `Serial1`: Connected to the `RobobuoySub` (half/full duplex communication).
  * `SPI (LoRa)`: SX1278 module on 433MHz.
  * `WiFi/UDP`: Hosts an AP (`BUOY_<mac>`) or connects to `NicE_WiFi`. Broadcasts/listens on UDP port `1001`.
  * `FastLED`: WS2812 status LEDs to indicate WiFi, LoRa, GPS, and Sub connection status.
  * `Buzzer`: Piezo buzzer for audio feedback.

## Core Responsibilities & Logic
1. **Routing:**
   * Read UDP/LoRa -> Decode via `RoboCompute` -> Queue to `serOut` (send to Sub via Serial1).
   * Read Serial1 from Sub -> Decode via `RoboCompute` -> Queue to `udpOut` and `loraOut` (send to PC).
2. **Addressing (The `SETUPDATA` Bugfix):**
   * The `SETUPDATA` (CMD 83) response from the Sub MUST be broadcasted with `IDr = BUOYIDALL` (which is `1`). Ensure you force `IDr = 1` before pushing the response to UDP and LoRa.
   * *Anti-Echo filtering:* Because `Serial1` might be half-duplex and echo its own transmission, `RobobuoyTop` must examine incoming `SETUPDATA` packets on Serial1. If `ACK == LORAGET` or `LORAGETACK`, it is an echo from the PC -> forward it to the Sub. If `ACK == LORAINF` (or others), it is a real response from the Sub -> forward to UDP/LoRa to reach the PC.
3. **Zero Compression Handling:**
   * Packets use a custom NMEA style: `$IDr,IDs,ACK,CMD,STATUS,val1,val2...*CRC`.
   * `RoboCompute` compresses zero values (`0`, `0.00`) into empty fields (`,,`) to save bandwidth. `RobobuoyTop` must decode and re-encode faithfully without breaking this protocol.
4. **Over-The-Air (OTA):**
   * Set the OTA hostname to `Top_<MAC_ADDRESS>` so the PC can discover it via mDNS.

## Constraints
* Ensure stability of the RTOS queues. Never block a task.
* Maintain the `RoboStruct` arrays (`buoyPara[3]`) to track the fleet state.
* The Top buoy does NOT run PID loops or drive motors; it strictly routes telemetry and commands.