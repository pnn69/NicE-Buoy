# RoboBuoy Firmware & Dashboard - Major Update v4.0.0

## 🚀 Overview
Version 4.0.0 represents a significant milestone in the development of the NicE-Buoy system. This release focuses on architectural stability, network efficiency, and a completely overhauled diagnostic dashboard. The communication protocol has been hardened to support reliable fleet coordination with minimized bandwidth overhead.

---

## 🖥️ RoboMonitor (Python Dashboard)
The diagnostic monitor has been completely refactored for better visibility and robustness.

### 1. Quad-Window Telemetry
- **Separated Data Streams**: The single raw data log has been replaced with four dedicated, color-coded windows:
    - **UDP IN (Lime)**: Incoming telemetry from buoys.
    - **UDP OUT (Cyan)**: Outgoing commands sent via WiFi.
    - **LoRa IN (Yellow)**: Incoming data from the LoRa controller.
    - **LoRa OUT (Orange)**: Outgoing LoRa commands.
- **Enhanced Readability**: Application width increased to **1400px** with horizontal resizing enabled to accommodate long protocol strings.

### 2. Protocol Hardening
- **Sender-Based Identification**: Corrected buoy mapping to use the **Sender ID** (`fields[1]`) from the `$Target,Sender...` protocol.
- **Strict ACK Validation**: The parser now only processes primary data (TOPDATA/SETUPDATA) if the ACK field is exactly `6` (`LORAINF`). This prevents echoed requests from overwriting real data with zeros.
- **Hex-Only Filtering**: Added safety checks to ignore non-hex IDs, preventing simulator artifacts or serial noise from corrupting the buoy database.

---

## 🛰️ RoboTop Firmware
The surface module's communication and bridging logic have been redesigned from the ground up.

### 1. Smart Interface Bridging
- **Collision Prevention**: Implemented a bridging logic that seamlessly routes messages between LoRa and UDP only when necessary.
- **Echo Suppression**: RoboTop now identifies if a message is intended for itself (or broadcast) and handles it locally. If intended for a different buoy, it bridges it to the alternate interface *without* echoing it back to the source.
- **Broadcast Support**: Added explicit support for legacy broadcast ID `0` alongside `BUOYIDALL`.

### 2. Setup Data Integrity
- **Instant Cache Response**: RoboTop now responds to `SETUPDATA` requests immediately using its local cache. This makes the dashboard Setup window feel nearly instantaneous.
- **Data Protection**: Fixed a critical bug where incoming setup requests would zero-out local parameters during the handshake.

---

## 📚 RoboDependency Library
The core shared library has been optimized for low-bandwidth environments.

### 1. Bandwidth Optimization
- **Dynamic Field Truncation**: The `rfCode` function now skips long strings of empty/zeroed data fields for simple **GET** and **ACK** packets.
- **Transitional State Support**: Added missing handlers for `IDELING`, `DOCKING`, and `RESET` commands to prevent "Unknown CMD" errors across the fleet.

### 2. Diagnostic Improvements
- **Correct Hex Formatting**: Fixed `printf` corruption where ESP32 MAC IDs were appearing as `lX`. They now show as clean, 8-digit uppercase hex strings (e.g., `B7A5B578`).

---

## 🛠️ Files Modified
- `RoboPythonDisplay/RoboControl.py` (Full Rewrite)
- `RoboTop/src/main.cpp`
- `RoboTop/src/loratop.cpp`
- `RoboDependency/RoboCompute/src/RoboCompute.cpp`
- `RoboDependency/RoboCompute/src/RoboCompute.h`
- `RoboSub/src/main.cpp`
- `RoboSub/src/pidrudspeed.cpp`
- `RoboSub/src/subwifi.cpp`

---

**Release Tag: v4.0.0**
**Date: June 3, 2026**
