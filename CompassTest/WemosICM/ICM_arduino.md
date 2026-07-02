# Heltec Wireless Tracker & LSM303AGR Compass Project Specification

## 1. Project Overview
This specification outlines the architectural and functional requirements for an Arduino-based firmware application targeting the **Heltec Wireless Tracker** (ESP32-S3 with onboard 0.96" TFT display, GPS, and LoRa). The application integrates an **LSM303AGR** sensor (accelerometer and magnetometer) to function as a tilt-compensated compass. It hosts a dynamic web portal from internal SPIFFS storage, streams real-time $100\text{Hz}$ telemetry over WebSockets, and supports Over-the-Air (OTA) firmware updates.

---

## 2. Hardware Configuration & Pinout

### 2.1. Power and General Control
- **LED Pin:** `18` (Onboard LED indicator, flashes as a loop heartbeat)
- **VEXT Power Enable:** `3` (Set `HIGH` to enable power rails for the TFT display, GNSS, and peripheral devices)

### 2.2. Display Configuration (ST7735 Mini 160x80)
The built-in display is a **0.96-inch 160x80 IPS TFT screen** driven by the `Adafruit_ST7735` library using **Hardware SPI** (running at up to 40MHz for flicker-free rendering) configured in landscape mode.
- **Backlight Pin (BL_PIN):** `21` (Set `HIGH` to enable display backlight)

| Signal Name | Pin Number | Description |
| :--- | :--- | :--- |
| `TFT_CS` | `38` | Chip Select |
| `TFT_RST` | `39` | Reset |
| `TFT_DC` | `40` | Data / Command Selection |
| `TFT_SCLK` | `41` | SPI Clock (Hardware SPI SCLK) |
| `TFT_MOSI` | `42` | SPI MOSI (Hardware SPI MOSI) |

**Instantiation & Initialization Example:**
```cpp
#include <SPI.h>
#include <Adafruit_ST7735.h>

// Initialize using the 3-argument Hardware SPI constructor
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void setupDisplay() {
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, HIGH); // Enable display power
    delay(100);
    pinMode(BL_PIN, OUTPUT);
    digitalWrite(BL_PIN, HIGH);   // Turn on backlight
    
    // Assign Hardware SPI pins on the ESP32-S3 and initialize
    SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
    tft.initR(INITR_MINI160x80); // Initialize for 160x80 screen size
    tft.setRotation(1);          // Set rotation to landscape
}
```

### 2.3. ICM-20948 I2C Sensor Pins
The LSM303AGR sensor is connected via the standard ESP32 I2C bus configured for Fast-Mode ($400\text{kHz}$).

| Signal Name | Pin Number | Description |
| :--- | :--- | :--- |
| `I2C_SDA` | `4` | I2C Data Line |
| `I2C_SCL` | `5` | I2C Clock Line |

---

## 3. Core Functional Requirements

### 3.1. Network & OTA Updates
* **Wi-Fi Connectivity:**
  * **SSID:** `NicE_WiFi`
  * **Password:** `!Ni1001100110`
* **Over-the-Air (OTA) Updates:**
  * Implement standard `ArduinoOTA` wireless update services.
  * Provide visual status progress bars on the ST7735 display during OTA transfers.

### 3.2. Compass & Sensor Fusion (ICM-20948)
* **Hardware Output Data Rate (ODR) Configuration:**
  * Explicitly reconfigure the physical sensor registers on startup to eliminate hardware undersampling and capture high-speed motion:
    * Set the **ICM-20948 Magnetometer** ODR to **100Hz** using `setDataRate(LIS2MDL_RATE_100_HZ)`.
    * Set the **ICM-20948 Accelerometer** ODR to **200Hz** by writing `0x67` directly to register `CTRL_REG1_A` (address `0x20`) on I2C address `0x19`.
* **Physical Coordinate Axes Alignment:**
  * Transform raw magnetometer axes to align with the accelerometer's coordinate frame prior to performing tilt compensation:
    * $X_{aligned} = -Y_{raw\_lsm}$
    * $Y_{aligned} = X_{raw\_lsm}$
    * $Z_{aligned} = Z_{raw\_lsm}$
* **Tilt-Compensated Heading Calculation:**
  * Calculate Roll ($\phi$) and Pitch ($\theta$) from the accelerometer:
    * $\text{Roll} = \arctan2(Ay, Az)$
    * $\text{Pitch} = \arctan2(-Ax, \sqrt{Ay^2 + Az^2})$
  * Rotate the aligned magnetic vector into the horizontal plane:
    * $X_h = M_{x\_aligned} \cos(\text{Pitch}) + M_{y\_aligned} \sin(\text{Roll})\sin(\text{Pitch}) + M_{z\_aligned} \cos(\text{Roll})\sin(\text{Pitch})$
    * $Y_h = M_{y\_aligned} \cos(\text{Roll}) - M_{z\_aligned} \sin(\text{Roll})$
  * Compute Heading = $\arctan2(Y_h, X_h) \times \frac{180}{\pi}$ (with $0^\circ$ to $360^\circ$ normalization, checking for standard positive rotation).

### 3.3. Calibration & Non-Volatile Storage (NVM)
* **Calibration Algorithms:**
  * **Hard-Iron Calibration:** Corrects for constant magnetic offsets ($V_{hx}, V_{hy}, V_{hz}$).
  * **Soft-Iron Calibration:** Corrects for magnetic field deformation and axis scaling distortions ($S_x, S_y, S_z$).
* **Non-Volatile Memory (NVM):**
  * Load and persist hard-iron offsets and soft-iron scale factors dynamically from NVM (`Preferences` namespace `"calibration"`) to protect settings across reboots.

### 3.4. Dynamic Web Server & Multi-Core Architecture (SPIFFS Hosted)
To guarantee a solid, jitter-free $100\text{Hz}$ telemetry stream, the firmware must utilize the ESP32's dual-core architecture:
1. **Core 0 Sensor Thread (FreeRTOS Task):**
   * A dedicated, high-priority FreeRTOS task pinned to **Core 0** that performs jitter-free $100\text{Hz}$ (every 10ms) I2C hardware polling, axes alignment, and tilt compensation.
   * Telemetry is written to a shared memory structure protected by a hardware spinlock (`portENTER_CRITICAL` / `portEXIT_CRITICAL`).
2. **Core 1 Application Thread (Main Loop):**
   * Dedicated to network requests, `ArduinoOTA`, Web Server, and TFT screen drawing.
   * Extracts telemetry safely from Core 0 and dispatches it immediately to WebSocket clients.
3. **Active Network Congestion Control:**
   * To prevent client disconnection loops, the WebSockets dispatcher must implement an active **frame-skipping queue check**. A telemetry packet is only sent to a client if its pending queue length is empty or has space: `client.queueLen() < 2`. If Congested, that specific frame is skipped.
4. **Dynamic Screen Throttling:**
   * Drawing text on local screens takes time. To maximize network throughput during active calibration, the loop must dynamically throttle the ST7735 display's refresh interval from **$300\text{ms}$ down to $2000\text{ms}$** when the `start_cal` signal is active.

---

## 4. Frontend Specifications (HTML/CSS/JS in SPIFFS)

Static assets are stored on internal SPIFFS, loaded once, and cached.

### 4.1. Comparative SVG Windroses
The interface displays two side-by-side, high-contrast inline SVG compass windroses:
- **Windrose A:** Displays tilt-compensated heading with **Hard-Iron only** calibration.
- **Windrose B:** Displays tilt-compensated heading with **Hard + Soft-Iron** calibration.
- **Visual styling:** Dark theme circular dials with cardinal labels (N in red, E, S, W in slate), cyan dashed inner rings, and a 3D split-shaded needle (red/slate) with a gold cap.
- **Shortest-Path Rotation:** Needle rotation animated via CSS `transform` using shortest-path angular math to prevent flipping or spinning when crossing the $0^\circ / 360^\circ$ boundary.

### 4.2. Real-Time 3D Orientation Visualizer
- Renders an interactive 3D model representing the physical tracker board rotating inside a wireframe bounding grid.
- **Reference Frame:** A fixed circular ground compass with 'N', 'S', 'E', and 'W' sprite markers is rendered on the floor to provide a spatial frame of reference.
- **Standard Euler Rotations:** Rotations are mapped strictly to standard Yaw-Pitch-Roll (Euler 'YXZ') flight-dynamics coordinates to eliminate mathematical gimbal locks.

### 4.3. Interactive 3D Calibration Portal
- **Real-Time Scatter Plot:** Displays live coordinate points as they are streamed over WebSockets, drawn instantly as 2D projections (X-Y and Y-Z planes) on a dark canvas using fast hardware-accelerated `fillRect()` methods.
- **Mathematical Solver (Client-side JS):** Collects raw points and performs mathematical center offsets and scale normalization to solve for hard-iron and soft-iron parameters.
- **Storage Hook:** Once computed, clicking "Store Calibration" transmits parameters back to the ESP32 over WebSockets to be written to NVM.
