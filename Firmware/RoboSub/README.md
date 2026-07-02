# RoboSub — Autonomous Marine Buoy Subunit Firmware

The **`RoboSub`** firmware runs on the submerged ESP32 microcontroller of the NicE-Buoy Autonomous Marine System. Acting as the low-level physical execution and navigation engine, it processes raw sensor data, executes autonomous PID navigation loops, and drives the thrusters via Electronic Speed Controllers (ESCs).

---

## 🏗️ Subsystem Architecture & FreeRTOS Flow

RoboSub utilizes FreeRTOS to manage real-time tasks across the dual cores of the ESP32:
*   **Core 1 (High-Priority Navigation & Sensors)**: Processes timing-critical I2C sensor fusion (`CompassTask`), 100Hz PID locomotion loops (`EscTask`), and high-speed RS-485 serial bridge decoding (`SercomTask`). This guarantees isolated, high-precision timing away from Wi-Fi overhead.
*   **Core 0 (Communications & Visuals)**: Manages Wi-Fi networking client hooks (`WiFiTask`), addressable LED status animations (`LedTask`), and status buzzer audio signals (`buzzerTask`).

```
┌────────────────────────────────────────────────────────────────────────┐
│                                RoboSub                                 │
├───────────────────────────────────┬────────────────────────────────────┤
│              Core 0               │               Core 1               │
├───────────────────────────────────┼────────────────────────────────────┤
│ - WiFiTask (Local AP Web Server)  │ - CompassTask (ICM-20948 Fusion)   │
│ - LedTask (NeoPixel Animations)   │ - EscTask (100Hz PID Loco Loops)   │
│ - buzzerTask (Audio Sounder Beeps)│ - SercomTask (RS-485 Serial Bridge)│
└───────────────────────────────────┴────────────────────────────────────┘
```

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
│  - ICM-20948 I2C Fusion (69/68)│ │  - Speed PID (100Hz)   │ │  - Status NeoPixels    │
│  - Madgwick AHRS Filter (100Hz)│ │  - Rudder PID (100Hz)  │ │  - Battery Telemetry   │
│  - 3D Copilot Web Calibration  │ │  - Differential Mix    │ └────────────────────────┘
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

### 2. High-Performance ICM-20948 Compass & Madgwick Fusion (`compass.cpp` / `compass.h`)
RoboSub utilizes a high-performance **ICM-20948 9-DOF IMU** coupled with a mathematical **Madgwick AHRS sensor fusion filter** to compute highly precise, drift-free magnetic headings.

*   **I2C Auto-Discovery**: Automatically probes active I2C addresses `0x69` and `0x68` at boot, dynamically instantiating the driver object on the discovered port.
*   **Zero-Rate Gyro Bias Calibration**: Runs an automated 200-sample gyroscope calibration routine at boot to calculate static offsets, ensuring gyro drift is eliminated.
*   **Advanced Madgwick AHRS Fusion (100Hz)**: Feeds filtered, low-passed accelerometer, gyroscope, and aligned calibrated magnetometer readings into a 100Hz Madgwick AHRS algorithm to track roll, pitch, and yaw.
*   **Tilt-Compensated Magnetometer Math**: Employs geometrical tilt-compensation algorithms utilizing roll and pitch vectors to guarantee correct magnetic headings even when the buoy experiences severe angular tilting in turbulent seas.
*   **NVS Persistence & Hard/Soft Iron Scaling**: Loads custom 3D calibration matrices (Hard Iron: `hi_x`, `hi_y`, `hi_z`; Soft Iron: `si_x`, `si_y`, `si_z`) from persistent Preferences NVS (Flash) to scale and offset raw magnetic distortion.
*   **3D Copilot Interactive Calibration Web Dashboard**:
    *   Integrates an interactive **3D Compass Calibration Tool** served via HTTP from the ESP32.
    *   Features real-time 3D plotting and feedback to help operators execute figure-eight and pitch/roll movements (Z-span checking) in the field.
    *   Validates point count (minimum 500 points) and multi-axis coverage before letting users commit the calculated parameters directly to NVS, playing an audio cue upon successful calibration storage.
*   **Highly Stable Averaging Filter**: Runs heading outputs through a low-pass Exponential Moving Average (EMA) filter and caches them into a 25-step circular averaging buffer (`NUM_DIRECTIONS 25`), smoothing out MEMS noise and momentary magnetic anomalies caused by high-power thruster surges.

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

## ⚙️ Persistent Configuration Parameters (Preferences NVS)
RoboSub utilizes the ESP32's **Preferences NVS (Non-Volatile Storage)** to store and load operational parameters across power-cycles. This ensures all controller configurations remain persistent:

### 1. Autopilot PID Parameters
*   **Rudder / Steering PID (`Kpr`, `Kir`, `Kdr`)**:
    *   `Kpr` (Default: `1.0`): Proportional coefficient for steering correction.
    *   `Kir` (Default: `0.0`): Integral coefficient.
    *   `Kdr` (Default: `0.0`): Derivative coefficient (steering damping).
*   **Speed / Approach PID (`Kps`, `Kis`, `Kds`)**:
    *   `Kps` (Default: `1.0`): Proportional coefficient for waypoint distance approach.
    *   `Kis` (Default: `0.0`): Integral coefficient.
    *   `Kds` (Default: `0.0`): Derivative coefficient.

### 2. Differential Locomotion Limits & Motor Flags
*   **Thrust Boundaries (`maxSpeed`, `minSpeed`, `pivotSpeed`)**:
    *   `maxSpeed` (Default: `75`): Maximum forward speed percentage allowed under autonomous control.
    *   `minSpeed` (Default: `-75`): Maximum reverse speed percentage allowed under autonomous control (NVS initializer writes `-75`).
    *   `pivotSpeed` (Default: `0.2`): Proportional motor speed coefficient during in-place pivoting.
*   **Thruster Correction Flags (`revBB`, `revSB`, `swap_BB_SB`)**:
    *   `revBB` (Default: `false`): Boolean flag to invert the rotation direction of the Port (BB) thruster.
    *   `revSB` (Default: `false`): Boolean flag to invert the rotation direction of the Starboard (SB) thruster.
    *   `swap_BB_SB` (Default: `false`): Boolean flag to swap Port and Starboard output control channels (remedies physical ESC wire cross-overs).

### 3. Compass Calibration & Correction Math
*   **Declination & Mounting Alignment (`declination`, `compassOffset`)**:
    *   `declination` (Default: `0.0`): Standard geographic magnetic declination offset.
    *   `compassOffset` (Default: `0.0`): Mounting angular offset calibration (set by "Set as North") to align the physical thruster output with magnetic North.
*   **Hard & Soft Iron 3D Calibration Matrices (`magHard`, `magSoft`)**:
    *   `magHard[3]` (NVS Keys: `mH00` to `mH02`): 3-axis offset vector used to correct Hard Iron distortions.
    *   `magSoft[3][3]` (NVS Keys: `mS00` to `mS22`): 3x3 scaling matrix used to normalize Soft Iron spatial distortions, generated by the 3D Copilot calibration tool.

### 4. Adaptive Boundary Keep Limits
*   **Hold / Anchor Radius (`holdRad`)**:
    *   `holdRad` (Default: `2.0`): Specifies the circular anchor boundary (in meters) for autonomous station keeping. If the buoy sails inside this radius, thrust is cut to neutral to save battery.

---

## 🛠️ Building & Flashing

RoboSub is built and managed using the **PlatformIO** ecosystem:

1.  **Configuration (`platformio.ini`)**: Defines build flags, libraries (`ESP32Servo`, `FastLED`, `ICM-20948`, `Madgwick`), and targets the `robo-esp-v3` environment.
2.  **Compilation**:
    ```powershell
    C:\Users\Peter.d.Nijs\.platformio\penv\Scripts\platformio.exe run
    ```
3.  **OTA Upload**:
    The subsystem supports Over-The-Air updates using PlatformIO's OTA upload protocol:
    ```powershell
    C:\Users\Peter.d.Nijs\.platformio\penv\Scripts\platformio.exe run -t upload
    ```
