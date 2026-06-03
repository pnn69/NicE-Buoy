# RoboBuoy Firmware Update Report - June 3, 2026

## Overview
Significant updates have been made to both the RoboSub and RoboTop firmware to improve stability, control performance, and user interface responsiveness.

## RoboSub (Sub-surface Module)

### 1. Control Systems (PID & Filtering)
- **Heading Filter Tuning**: The low-pass filter alpha for heading error was increased from 0.40 to 0.80. This drastically reduces phase lag, resulting in faster response and significantly reduced overshoot during course corrections.
- **Improved Anti-Windup**: Simplified the PID anti-windup logic. The Integral term is now gracefully disabled when the heading error exceeds 15 degrees, preventing "windup" without causing abrupt transitions in control output.

### 2. WiFi & Telemetry
- **Stack Stability**: Increased the `WiFiTask` stack size from 8KB to 16KB to prevent stack overflow issues during heavy network traffic.
- **Parameter Sync Revision**: Implemented a global parameter revision system (`global_params_rev`). The web UI now only polls for full parameter updates when a change is detected on the device, reducing network overhead and improving UI responsiveness.
- **Robust Connectivity**: Enhanced the WiFi connection logic with explicit serial logging and automatic restart on persistent connection failure.
- **Added safety**: Implemented a minimum safety limit of 1.5m for the "Locking Range" (holdrad) to ensure it stays outside the pivot range plus a buffer.

### 3. Web Interface (index.html)
- **UI Clarity**: Renamed "Piv" to "Pivot" and "Rad" to "Lock" for better user understanding.
- **Dynamic Updates**: The interface now reacts to the revision system, updating field values automatically if changed by another controller (e.g., Lora).
- **Calibration Feedback**: Improved the BNO055 calibration status display logic.

## RoboTop (Surface Module)

### 1. GPS & Navigation Safety
- **Zero-Coordinate Protection**: Added comprehensive checks to ensure `RouteToPoint` is only called with valid GPS coordinates. This prevents "jumping" and invalid distance/direction calculations when GPS lock is lost or dock positions are uninitialized.
- **Default Docking**: Added a default fallback dock position (52.29296796, 4.93254612) if the onboard memory is empty, ensuring the system has a valid target on first boot.
- **Enhanced Logging**: Added detailed serial warnings when dock positions are missing or coordinates are invalid.

## Files Modified:
- `RoboSub/data/index.html`
- `RoboSub/src/main.cpp`
- `RoboSub/src/pidrudspeed.cpp`
- `RoboSub/src/subwifi.cpp`
- `RoboTop/src/main.cpp`
