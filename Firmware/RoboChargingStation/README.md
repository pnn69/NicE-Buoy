# RoboChargingStation (Base Station Firmware)

The `RoboChargingStation` is intended to manage the physical docking and charging infrastructure for the NicE-Buoys.

> ## ⚠️ Status: scaffolding only — no firmware yet
>
> This directory currently contains **only `platformio.ini` and this README**. There is no `src/`,
> no `main.cpp` and nothing to build. Everything below describes the intended design, not working
> behaviour. Treat it as a specification.

## Intended features
- **Communications:** monitor buoy status, location and power levels over LoRa/WiFi.
- **Hardware integration:** drive relays and an MCP23017 I/O expander to engage the docking
  mechanism or charging lines once a buoy is correctly aligned with the dock.
- **State monitoring:** watch the `DOCKING` → `DOCKED` transition and trigger the physical systems
  from it.

## What the project file already declares

`platformio.ini` targets `esp32dev` and pulls in a fairly complete dependency set, which is a
reasonable guide to the intended hardware:

| Library | Suggests |
|---|---|
| `Adafruit_LSM303AGR_Mag`, `Adafruit_LSM303_Accel` | magnetometer + accelerometer (buoy alignment sensing) |
| `Adafruit-MCP23017` | I/O expander for relays |
| `Adafruit_SSD1306`, `Adafruit-GFX` | local OLED status display |
| `arduino-LoRa` | the fleet radio link |
| `TinyGPSPlus` | position |
| `ESP32Servo`, `FastLED` | actuator and status LEDs |
| `ArduinoOTA` | field updates |

Upload is preconfigured as `upload_protocol = espota` to `192.168.4.1` — that is the default ESP32
softAP address, i.e. flashing while connected to the station's **own** access point.

## When it is implemented

It should follow the same conventions as the rest of the fleet — see the
[firmware README](../README.md):

- WiFi priority `NicE_WiFi` → `Robo_WiFi` → its own AP, with `geenanker` as the AP password.
- mDNS via `ArduinoOTA.begin()` only; never a second `MDNS.begin()`.
- Protocol frames on UDP `1001`; a plain-text debug log on UDP `1002` is available for free by
  reusing `udplog.cpp` from `RoboTop`.
- Broadcast beacons must use `ack = INF`. A broadcast asking for an ACK jams a LoRa retry slot for
  five retransmits.
