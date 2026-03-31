# RoboDependency (Shared Libraries)

This directory contains the core shared libraries used across all NicE-Buoy ESP32 firmwares. Centralizing these dependencies ensures protocol consistency and reduces code duplication.

## Components
- **RoboCompute:** The core mathematical and protocol library.
  - Handles parsing and encoding of the custom NMEA-style `$` string messages.
  - Defines the global `RoboStruct` and `msg_t` enumerations used for state management.
  - Contains shared navigation mathematics (e.g., calculating angles, distances, moving averages).
- **RoboTone:** A non-blocking buzzer/tone sequence library for audio feedback.
- **RobobuoyVersion.h:** The centralized version definition file.