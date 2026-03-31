# LoraController - Main Development Prompt

## Role & Goal
You are building the firmware for `LoraController`, a transparent USB-to-LoRa bridge for the Robobuoy system. It runs on an ESP32 Pico Kit.
Your goal is to recreate this project exactly as it exists today. The device sits next to the user's PC, receives commands via USB Serial from the `RoboControl` Python app, and transmits them over the air via a LoRa transceiver to `RobobuoyTop`. It also listens for incoming LoRa telemetry and forwards it back to the PC.

## Hardware & Architecture
* **Microcontroller:** ESP32 Pico Kit (`pico32`).
* **Libraries:** `sandeepmistry/arduino-LoRa`, `FastLED`, `RoboCompute`.
* **Hardware Interfaces:**
  * `USB Serial`: Main communication line with the PC (115200 baud).
  * `SPI (LoRa)`: SX1278 module on 433MHz.
  * `OLED Display`: I2C (optional status display).

## Core Responsibilities & Logic
1. **Bridging Logic:**
   * **Serial -> LoRa:** Read NMEA string from USB Serial (`$IDr,IDs...*CRC`). Decode it into a `RoboStruct` using `RoboCompute`, then re-encode it using `rfCode()`, and push to `loraOut` queue to send.
   * **LoRa -> Serial:** Read packet from SPI LoRa. Decode into a `RoboStruct` using `RoboCompute`, re-encode using `rfCode()`, and print to USB Serial.
2. **Zero Compression Handling:**
   * Packets use a custom NMEA style: `$IDr,IDs,ACK,CMD,STATUS,val1,val2...*CRC`.
   * `RoboCompute` compresses zero values (`0`, `0.00`) into empty fields (`,,`) to save bandwidth over the slow LoRa link. 
   * It is strictly required that the `LoraController` correctly formats integers (e.g. `minOfsetDist`, `gpsDir`, `maxSpeed`) when re-encoding. Do NOT use `String(val, 0)` for integer values, as it breaks the base-10 formatting and corrupts the zero-compression logic.

## Constraints
* Use FreeRTOS tasks to keep Serial reading and LoRa reading completely non-blocking.
* The controller must seamlessly handle all NMEA command variants from `RoboCompute` (`SETUPDATA`, `DIRDIST`, `TOPDATA`, `SUBDATA`, etc.) because it must parse and re-encode every single message.
* Keep it purely as a relay. It does not store fleet state, calculate paths, or modify commands; it simply unpacks and repacks to ensure protocol integrity before changing physical layers.