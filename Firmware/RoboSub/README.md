# RoboSub — Autonomous Marine Buoy Subunit Firmware

The **`RoboSub`** firmware runs on the submerged ESP32 microcontroller of the NicE-Buoy Autonomous Marine System. Acting as the low-level physical execution and navigation engine, it processes raw sensor data, executes autonomous PID navigation loops, and drives the thrusters via Electronic Speed Controllers (ESCs).

---

## 🏗️ Subsystem Architecture & FreeRTOS Flow

RoboSub utilizes FreeRTOS to manage real-time tasks across the dual cores of the ESP32:
*   **Core 1 (High-Priority Navigation & Sensors)**: Processes timing-critical I2C sensor fusion (`CompassTask`), 100Hz PID locomotion loops (`EscTask`), and high-speed RS-485 serial bridge decoding (`SercomTask`). This guarantees isolated, high-precision timing away from Wi-Fi overhead.
*   **Core 0 (Communications & Visuals)**: Manages Wi-Fi networking client hooks (`WiFiTask`), addressable LED status animations (`LedTask`), and status buzzer audio signals (`buzzerTask`).

```
                      ┌────────────────────────────────────────┐
                      │             RoboTop (UART)             │
                      └───────────────────┬────────────────────┘
                                          │ Half-Duplex RS-485
                                          ▼
                      ┌────────────────────────────────────────┐
                      │               SercomTask               │
                      │       - Decodes Command Frames         │
                      │       - Single-wire Echo Filter        │
                      └───────────────────┬────────────────────┘
                                          │
                  ┌───────────────────────┼────────────────────────┐
                  │ Queues / Mutex        │ Queues / Mutex         │ Queues
                  ▼                       ▼                        ▼
┌────────────────────────────────┐ ┌────────────────────────┐ ┌────────────────────────┐
│           CompassTask          │ │        PIDTask         │ │        LedTask         │
│  - BNO055 I2C Fusion (0x28/29) │ │  - Speed PID (100Hz)   │ │  - Status NeoPixels    │
│  - NVS Profile Injection       │ │  - Rudder PID (100Hz)  │ │  - Battery Telemetry   │
│  - 30-Min Auto-Save Guard      │ │  - Differential Mix    │ └────────────────────────┘
└────────────────────────────────┘ └──────────┬─────────────┘
                                              │ PWM Duty Cycles
                                              ▼
                                   ┌────────────────────┐
                                   │      EscTask       │
                                   │  - Port Motor (BB) │
                                   │  - Stbd Motor (SB) │
                                   └────────────────────┘
```

---

## ⚡ Core Modules & Functionality

### 1. Locomotion & Thruster Mixing (`esc.cpp` / `esc.h`)
RoboSub translates abstract navigation instructions into physical locomotion using differential thrust mixing:
*   **Hardware Interface**: Controls two brushless thrusters—**Port (BB)** and **Starboard (SB)**—via electronic speed controllers connected to hardware PWM channels utilizing the `ESP32Servo` library.
*   **Arming Sequence**: Executes a strict ESC arming cycle at boot (writing a safety-neutral 1500µs pulse-width) to protect hardware and ensure pilot safety.
*   **Differential Thrust Mixer**: Combines the forward speed demand ($PID_{\text{speed}}$) and steering demand ($PID_{\text{rudder}}$) to drive both motors independently:
    $$Thrust_{\text{Port}} = Speed + Steering$$
    $$Thrust_{\text{Starboard}} = Speed - Steering$$

### 2. Intelligent Compass Fusion (`compass.cpp` / `compass.h`)
RoboSub utilizes an **Adafruit BNO055** 9-degrees-of-freedom intelligent sensor fusion coprocessor to track exact orientation and heading.

*   **I2C Auto-Discovery**: Automatically probes I2C addresses `0x28` and `0x29` at boot, re-instantiating the driver object on the active address dynamically.
*   **Atomic Profile Injection & Verification**: To bypass the extensive manual calibration sequence (figures-of-eight and tilting) required by the sensor at boot, RoboSub implements atomic register injection:
    *   Reads a stored 22-byte calibration matrix from the ESP32 NVS (Flash).
    *   Forces the BNO055 into `CONFIG` mode, burst-writes all 22 registers (comprising accelerometer, gyroscope, and magnetometer offsets + radius vectors), and switches back to `NDOF` fusion mode.
    *   Immediately reads back the active registers and validates them byte-for-byte against the NVS data. If the verification matches, it emits a confirmation beep and displays `"Profile Loaded OK"`.
*   **Conditional Auto-Save with 30-Minute Sustained Countdown**: To ensure the stored NVS calibration is of the highest quality (representing a broad matrix of operating angles in the water), RoboSub features a highly robust, conditional save guard:
    *   **Startup Bypass**: If a valid profile is successfully loaded from NVS during boot, the auto-save loop is **completely disabled**. This protects your verified long-term calibration from being overwritten by a localized startup state.
    *   **Conditional Countdown**: If the NVS is empty at boot, the system waits until the sensor hits peak calibration accuracy (`M:3` and `G:3`). Once reached, it begins a **30-minute countdown timer**.
    *   **Automatic Resilience**: If the calibration drops below peak accuracy before the 30-minute mark (e.g., due to local magnetic interference), the countdown is immediately reset to `0` and will restart only when peak calibration is re-attained.
    *   **Manual Override**: The pilot can trigger a manual save command (`cmd == 34`) at any time through the web interface to instantly write the current active calibration offsets to NVS.
*   **Stuck-Sensor Watchdog**: Monitors the heading data continuously. If the telemetry remains frozen for 5 consecutive minutes (indicating an I2C hang or internal sensor crash), the system automatically performs a full sensor re-initialization sequence and restores the profile.
*   **Highly Stable Averaging Filter**: Raised `NUM_DIRECTIONS` (the size of the circular averaging buffer) from `5` to `25` inside `compass.cpp`. This increases history filtering, yielding extremely smooth, stable heading updates and dampening localized magnetic anomalies caused by high-power thruster surges.

### 3. Dual-Loop PID Navigation (`pidrudspeed.cpp` / `pidrudspeed.h`)
Autonomy is governed by independent, high-speed PID loops running at **100Hz** inside the `PIDTask`:
*   **Speed PID**: Computes the required thrust based on the remaining distance to the target waypoint.
*   **Rudder PID**: Calculates the required heading correction to align the buoy's current heading with the target direction.
*   **Navigation Modes**:
    *   `IDLE`: Thrust output disabled; motors neutral.
    *   `MANUAL`: Bypasses PID control to allow direct manual piloting over LoRa or WiFi.
    *   `LOCKED`: Autonomous position-holding (GPS Anchor) using localized coordinate boundaries.
    *   `DOCKING`: Automated waypoint navigation to guide the buoy to its charging station or home harbor.

### 4. UART Serial Interface & RS-485 Sercom (`sercom.cpp` / `sercom.h`)
RoboSub communicates with the `RoboTop` over a single-wire half-duplex UART interface.

*   **Half-Duplex Echo Filtering**: Because the RX and TX lines are tied together, the transmitting ESP32 receives its own transmissions. `SercomTask` filters out these loopback echoes by comparing the sender MAC address against its own address, discarding self-transmitted frames automatically.
*   **Implicit Acknowledgments**: Every periodic `INF` telemetry response from RoboSub serves as an implicit acknowledgment for commands, which reduces communication overhead and prevents packet congestion.

---

## 🎛️ Peripheral Controllers

*   **Battery Telemetry (`adc.cpp`)**: Reads battery voltages via ESP32 ADC pins, applies smoothing filters to eliminate voltage sags from thruster surges, and logs the cell power telemetry.
*   **Sounder Feedback (`buzzer.cpp`)**: Drives a piezo sounder, emitting audible frequencies and beep sequences to signal boot status, profile loading status, and save actions.
*   **Visual Status Matrices (`leds.cpp` / `leds.h`)**: Displays system states, communication connectivity, and battery low warnings via an addressable WS2812B RGB NeoPixel array.

---

## 🛠️ Building & Flashing

RoboSub is built and managed using the **PlatformIO** ecosystem:

1.  **Configuration (`platformio.ini`)**: Defines build flags, libraries (`ESP32Servo`, `FastLED`, `Adafruit_BNO055`), and targets the `robo-esp-v3` environment.
2.  **Compilation**:
    ```powershell
    C:\Users\Peter.d.Nijs\.platformio\penv\Scripts\platformio.exe run
    ```
3.  **OTA Upload**:
    The subsystem supports Over-The-Air updates using PlatformIO's OTA upload protocol:
    ```powershell
    C:\Users\Peter.d.Nijs\.platformio\penv\Scripts\platformio.exe run -t upload
    ```
