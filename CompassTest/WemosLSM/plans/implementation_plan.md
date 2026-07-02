# Project Design Document & Implementation Plan: WemosLSM

This document details the architecture and implementation plan for building and deploying the firmware and web companion app for the **Heltec Wireless Tracker** and **LSM303AGR** compass.

## 1. Objectives & Scope
The goal is to develop a highly polished, fully functional Arduino-based firmware that:
1. Calculates real-time **tilt-compensated heading** using accelerometer and magnetometer data from the LSM303AGR.
2. Supports **Hard-Iron** and **Soft-Iron Calibration** to eliminate magnetic distortions.
3. Hosts a high-performance **ESPAsyncWebServer** using **WebSockets** for real-time (30fps) client-server telemetry.
4. Renders **two comparative windroses** (Hard-Iron only vs. Hard + Soft-Iron) and a **dynamic 3D visualizer** (representing the physical tracker's movement in real-time) using Three.js on the frontend.
5. Performs the complex magnetometer calibration fitting calculations directly on the client browser, sending computed calibration matrices/offsets back to the ESP32 to be stored in non-volatile memory (**Preferences**).
6. Runs on the **Heltec Wireless Tracker (ESP32-S3)** with ST7735 TFT display and supports Over-The-Air (OTA) updates.

---

## 2. System Architecture

```
                       +---------------------------------------+
                       |       Heltec Wireless Tracker         |
                       |       (ESP32-S3 / ST7735 TFT)         |
                       +-------------------+-------------------+
                                           |
                    +----------------------+----------------------+
                    |                      |                      |
            I2C Bus |                      | WebSockets / HTTP    | Wi-Fi (LAN)
                    v                      v                      v
         +----------+----------+  +--------+---------+   +--------+--------+
         | LSM303AGR Sensor    |  | ESPAsyncWebSrv   |   | ArduinoOTA      |
         | Accel + Magnetometer|  | (SPIFFS / LittleFS)| | Over-The-Air    |
         +---------------------+  +--------+---------+   +-----------------+
                                           |
                                           v  WebSocket (Telemetry & Commands)
                                  +--------+---------+
                                  | Client Browser   |
                                  | Dashboard (HTML5)|
                                  +--------+---------+
                                           |
                                           |--- Windrose A (Hard-Iron only)
                                           |--- Windrose B (Hard & Soft-Iron)
                                           |--- Three.js 3D Orientation Cube
                                           |--- Calibration Math (JS matrix fit)
```

### 2.1. Hardware & Firmware Layers (C++)
- **PlatformIO + Arduino ESP32 Framework:** Standard toolchain configuration.
- **Sensor Drivers:** 
  - `Adafruit_LSM303_Accel` and `Adafruit_LSM303AGR_Mag` for raw values.
- **Display Driver:** `Adafruit_ST7735` to show current status/IP address.
- **Network Stack:**
  - `WiFi` for connecting to SSID: `NicE_WiFi`.
  - `ArduinoOTA` for over-the-air binary uploading.
  - `ESPAsyncWebServer` and `AsyncTCP` for high-throughput WebSocket transmission.
- **NVM Storage:**
  - `Preferences` (ESP32 native NVM API) to store/retrieve 3D hard-iron offsets ($V_{hi}$) and a $3\times3$ soft-iron compensation matrix ($W_{si}$).

### 2.2. Web Dashboard & Visualization Layers (HTML/CSS/JS)
- **HTML/CSS (SPIFFS/LittleFS):** Static assets stored locally on the ESP32, loaded once, and cached.
- **WebSockets:** Establishes a raw connection for transferring real-time packet data (JSON format) containing:
  - Raw Accelerometer ($x, y, z$)
  - Raw Magnetometer ($x, y, z$)
  - Calibrated Magnetometer ($x, y, z$)
  - Heading (Hard-Iron only), Heading (Hard + Soft), Roll, Pitch
- **UI Render Engine:**
  - **Three.js** (loaded via CDN or minified and stored locally) to render a 3D board/cube inside a bounding wireframe.
  - **HTML Canvas** to render the custom Dual Windroses.
- **Calibration Engine (Client-side JS):**
  - When the user clicks **Start Calibration**, the browser accumulates raw magnetometer points.
  - Mathematical fitting (such as least-squares ellipsoid fitting) is computed in JS. This solves for both the hard-iron offset vector ($3\times1$) and the soft-iron matrix ($3\times3$).
  - Once complete, clicking **Store Calibration** sends these computed coefficients back to the ESP32 via WebSocket.
  - The ESP32 writes them to `Preferences` and immediately applies them.

---

## 3. Project File Structure

```
WemosLSM/
├── platformio.ini         # PlatformIO Project Configuration
├── LSM_arduino.md         # Technical Specification (Markdown)
├── plans/
│   └── implementation_plan.md # This document
├── data/                  # Web Server Files (SPIFFS / LittleFS)
│   ├── index.html         # Dynamic windrose dashboard & 3D Three.js renderer
│   ├── calibrate.html     # Web calibration interface with calibration algorithm
│   └── style.css          # Visual styling for the dashboard
└── src/                   # C++ Source Code
    ├── main.cpp           # Main entrypoint, setup, loop
    ├── DisplayController.h # Display initialization and text drawing
    ├── SensorFusion.h     # Compass reading, tilt compensation, math calculations
    └── WebPortal.h        # WiFi, WebServer, WebSockets, OTA and NVM management
```

---

## 4. Implementation Steps

### Phase 1: PlatformIO Setup & Configuration
- Create `platformio.ini` with correct ESP32-S3 board configurations (specifically for the Heltec Wireless Tracker).
- Add external dependencies (`Adafruit LSM303DLHC`, `Adafruit BusIO`, `Adafruit GFX`, `Adafruit ST7735`, `ESPAsyncWebServer`, `AsyncTCP`).
- Configure partitions to support both SPIFFS/LittleFS and OTA.

### Phase 2: Sensor and Display Control (C++)
- Implement initialization routines for the ST7735 display (enable power via `VEXT_PIN` (GPIO 3) and backlight via `BL_PIN` (GPIO 21)).
- Implement `SensorFusion` to read raw LSM303AGR accelerometer and magnetometer data.
- Write standard Hard-Iron and Soft-Iron calibration logic:
  $$\vec{M}_{calibrated} = W_{si} \times (\vec{M}_{raw} - \vec{V}_{hi})$$
- Apply tilt compensation to calculate roll, pitch, and yaw:
  - $\text{Roll} = \arctan2(Ay, Az)$
  - $\text{Pitch} = \arctan2(-Ax, \sqrt{Ay^2 + Az^2})$
  - Compasses yaw (heading) calculated using horizontal magnetometer projections.

### Phase 3: Web Server & Network Interface (C++)
- Set up WiFi connection with `NicE_WiFi` / `!Ni1001100110`.
- Initialize `ArduinoOTA` for OTA uploads.
- Implement SPIFFS/LittleFS filesystem serving `index.html` and other assets.
- Set up WebSocket endpoint (`/ws`) to stream JSON telemetry data at ~30Hz.
- Implement WebSocket commands:
  - `start_cal`: Starts sending raw uncalibrated values to client.
  - `save_cal`: Receives calibration coefficients from client and writes them to ESP32 Preferences NVM.

### Phase 4: Frontend Development (HTML/CSS/JS)
- Develop `index.html` with:
  - Two high-performance HTML5 Canvas Windroses.
  - A Three.js 3D viewport containing a 3D box representing the physical tracker.
  - An easy-to-use menu to navigate to `calibrate.html`.
- Develop `calibrate.html` with:
  - An algorithm to collect magnetic field points and compute a 3D ellipsoid fit.
  - Visual feedbacks of calibration progress.
  - A button to submit the calculated factors back to the device.

---

## 5. Testing & Verification Strategy
1. **Compilation Validation:** Ensure PlatformIO successfully compiles the C++ code without any warnings or type safety violations.
2. **Local Hardware Verification (First Upload):**
   - Flash via USB interface.
   - Monitor Serial output to verify connection to Wi-Fi.
   - Verify TFT display lights up and shows IP address.
3. **Sensor Calibration Validation:**
   - Verify raw magnetic data matches expected patterns (circular/ellipsoidal orbit in 3D).
   - Test calibration fit accuracy: Windrose B should match magnetic north perfectly as the tracker is rotated, while Windrose A shows distortion errors.
4. **OTA Verification:**
   - Modify the TFT displayed version.
   - Upload via OTA to verify the PlatformIO OTA toolchain works seamlessly.
