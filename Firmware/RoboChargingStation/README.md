# RoboChargingStation (Base Station Firmware)

The `RoboChargingStation` manages the physical docking and charging infrastructure for the NicE-Buoys.

## Key Features
- **Communications:** Monitors buoy status, location, and power levels over LoRa/WiFi.
- **Hardware Integration:** Manages relays and I/O expanders (e.g., MCP23017) to activate docking mechanisms or charging lines when a buoy correctly aligns with the dock.
- **State Monitoring:** Tracks transitions from `DOCKING` to `DOCKED` to trigger physical systems autonomously.