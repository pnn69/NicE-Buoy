# RoboTop (Surface Unit Firmware)

The `RoboTop` firmware runs on the surface ESP32 unit of the NicE-Buoy system. It acts as the central communications and routing hub for the buoy.

## Key Features
- **Communications Hub:** Bridges LoRa, WiFi (UDP & Web Server), and a direct Serial link to the `RoboSub` unit.
- **Web Interface:** Hosts a captive portal / Web GUI (`index.html` via SPIFFS) for monitoring telemetry (battery, heading, GPS) and issuing commands (LOCK, DOCK, setup PIDs).
- **GPS Navigation:** Integrates a GPS module to determine global position and calculate distance/heading to target waypoints.
- **User Feedback:** Controls top-mounted LEDs and a buzzer for visual and auditory status indicators (e.g., locking, docking, error states).
- **Routing Logic:** Intelligently forwards NMEA-style `$CMD` packets from remote controllers (Python UI or LoRa Handset) down to the `RoboSub`.