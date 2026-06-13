# NicE-Buoy Autonomous Marine System Firmware

Welcome to the master repository for the **NicE-Buoy Autonomous Marine System**. This codebase powers a highly reliable, dual-processor autonomous marine buoy designed for GPS lock-to-position, remote configuration, automated sailing, and live telemetry tracking over LoRa and WiFi (UDP).

---

## 📌 Project Overview

NicE-Buoy is an industrial-grade marine robotic system consisting of multiple hardware and software components. It utilizes a **dual-ESP32 architecture** split into a **Top Unit (RoboTop)** and a **Sub Unit (RoboSub)**, communicating over a half-duplex serial interface. This physical decoupling isolates high-frequency navigation, radio communication, and web server tasks (Top) from raw thruster driving, compass polling, and high-frequency PID motor loops (Sub).

---

## 🏗️ System Architecture

```
                    ┌──────────────────────────────────────────────┐
                    │            RoboPythonDisplay (PC GUI)        │
                    │         - Multi-buoy Tkinter Dashboard        │
                    └──────────────┬───────────────────▲───────────┘
                                   │ UDP               │ LoRa (via COM)
                                   ▼                   ▼
┌──────────────────────────────────┴───────────────────┴──────────────────────────────────┐
│                                       RoboTop                                           │
│  - GPS Polling (NMEA Parser)              - Wi-Fi Access Point & Station Web-Dashboard  │
│  - LoRa RF Communication Task (LoraTask)  - UDP Broadcast Server (1001)                 │
│  - Buzzer & RGB LED Indicators            - Serial Bridge over Half-Duplex RS-485       │
└──────────────────────────────────────────┬──────────────────────────────────────────────┘
                                           │ UART Half-Duplex Serial
                                           ▼
┌──────────────────────────────────────────┴──────────────────────────────────────────────┐
│                                       RoboSub                                           │
│  - Dual ESC Thruster Drivers (BB & SB)   - Compass Sensor Interface (HMC5883L/QMC5883L) │
│  - Heading & Speed PID Loops (100Hz)     - Watchdog & I2C Sensor Re-initialization      │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

The codebase is structured into the following decoupled sub-projects:

1.  **`RoboTop` (Top Unit)**:
    *   **Processor**: ESP32 (running FreeRTOS).
    *   **Responsibilities**: Acquires GPS coordinates, hosts the web-based HTML5 control dashboard, broadcasts status telemetry packets via UDP and LoRa, and delegates motor execution commands to the Sub Unit.
    *   **Key Tasks**: `GpsTask` (NMEA parsing & outlier filtering), `LoraTask` (RF telemetry and acknowledgments), `WiFiTask` (Async web servers and UDP socket), and `SercomTask` (serial bridge).
2.  **`RoboSub` (Sub Unit)**:
    *   **Processor**: ESP32 (running FreeRTOS).
    *   **Responsibilities**: Directly interfaces with physical actuators (stern and bow thrusters via electronic speed controllers) and reads sensor telemetry (I2C magnetic compass).
    *   **Key Tasks**: `PIDTask` (high-speed heading/speed calculations), `CompassTask` (stuck-sensor watchdog and self-healing), and `SerialTask` (command execution).
3.  **`RoboDependency` (`RoboCompute` Library)**:
    *   **Responsibilities**: Common shared library loaded by both Top and Sub units.
    *   **Features**: Holds navigation math algorithms (`RouteToPoint` shortest-path, distance, and angle calculation), the unified `RoboStruct` communication state structure, formatting libraries (`formatFloat`), and the core telemetry string encoder/decoder (`rfCode` and `rfDeCode`).
4.  **`RoboLora` (LoRa Gateway / Controller)**:
    *   **Responsibilities**: Serves as a standalone USB-to-LoRa serial gateway. Relays packets between the PC monitor software and the buoys over long-range RF channels.
5.  **`RoboPythonDisplay` (Python GUI Monitor)**:
    *   **Responsibilities**: A desktop control dashboard (`RoboControl.py`) built with Tkinter for live telemetry tracking of up to 3 buoys.
    *   **Features**: Real-time windroses, thruster speed indicators (BB/SB), battery voltages, live socket listeners, and interactive PID parameter setting menus.
6.  **`RoboChargingStation`**:
    *   **Responsibilities**: Solar monitoring and charging dock automated control firmware.

---

## 📡 Communication Protocol & Frame Format

The system communicates using a lightweight, proprietary, ASCII-based comma-separated protocol designed to optimize RF bandwidth while remaining human-readable.

### Frame Anatomy
```
$TargetID,SenderID,AckType,CommandType,Status,Param1,Param2,...,ParamN*CRC\n
```

*   **`$`**: Starting sentinel character.
*   **`TargetID`**: Address of the recipient (e.g. `99` for PC, `0` or `1` for Broadcast, specific hex MAC ID).
*   **`SenderID`**: Address of the transmitting station.
*   **`AckType`**: Packet type classification:
    *   `1 (GET)`: Request parameter values.
    *   `2 (SET)`: Modify parameter values.
    *   `3 (GETACK)`: Request command execution with receipt verification.
    *   `4 (ACK)`: Receipt confirmation.
    *   `6 (INF)`: Telemetry status update.
*   **`CommandType`**: Integer mapping to system command (e.g., `51` for `TOPDATA`, `83` for `SETUPDATA`, `47` for `DIRDIST`).
*   **`Status`**: Current state machine status of the buoy (`IDLE`, `LOCKED`, `DOCKING`, `CALIBRATION`, etc.).
*   **`Params`**: Dynamic floating-point or integer parameters depending on the command.
*   **`*`**: End-of-payload delimiter.
*   **`CRC`**: 2-character hexadecimal CRC calculated by XOR'ing all characters between `$` and `*`.

---

## 🚀 Advanced Telemetry Optimizations

### 1. Bandwidth Compression
To maximize LoRa airtime efficiency and range, telemetry strings undergo two compression steps in `RoboCompute.cpp`:
*   **Float Trailing Stripping**: The `formatFloat()` function trims redundant trailing zeros and decimal points from float values (e.g. `12.5000` becomes `12.5`, `18.000` becomes `18`).
*   **Zero Compression**: A post-processing encoder in `RoboCode()` searches for consecutive commas representing zeros and compresses them to empty strings (e.g., `,0,0,0,0,` is packed to `,,,,,`). This dramatically reduces LoRa packet sizes.

### 2. Half-Duplex Wire Echo Filtering
In the RS-485 single-wire physical interface connecting Top and Sub, the TX and RX lines are tied together. This causes the transmitting ESP32 to receive its own transmissions immediately.
*   The `SercomTask` implements **Echo Filtering** by checking if the incoming packet sender ID matches its own MAC address and filtering out self-echoed `GET`, `GETACK`, or `SET` packets.
*   It implements **Implicit ACKs**: any valid `INF` (Information) response from the Sub implicitly signals that it processed the command, allowing the Top's retry queue to clear without extra overhead.

### 3. Hardware Self-Healing Watchdogs
Marine environments are hostile to electronics. The system is designed to recover autonomously from failures:
*   **LoRa Transceiver Watchdog**: If a LoRa packet transmission fails consistently over a 500ms window, the SPI channel or state machine is assumed locked. The system calls `InitLora()` to force a full hardware re-init.
*   **Compass Watchdog**: If the I2C compass readings remain completely static for over 5 minutes while thruster power is applied, the system declares `COMPASS STUCK - REINIT` and forces sensor reinitialization.
*   **Task Stacking & Watchdogs**: All FreeRTOS tasks contain safety-critical yielding (`vTaskDelay(1)`) on their pinned cores to prevent thread starvation and hardware watchdog resets.

---

## 🧭 In-Field Navigation Calibration State Machines

NicE-Buoy supports advanced calibration procedures directly in the water:

1.  **In-Field Compass Spin Calibration**:
    *   Puts the buoy into self-guided continuous rotation while measuring magnetic minimums/maximums to calculate hard-iron offsets.
2.  **In-Field Offset Alignment**:
    *   Sails the buoy on a linear trajectory (e.g., South/180°) using pure magnetic compass readings.
    *   Tracks the actual track trajectory using high-accuracy GPS coordinates.
    *   Calculates the angular difference between GPS heading and compass heading, updating the stored compass offset in memory to align physical thruster output with magnetic North.

---

## 🛠️ Developer Guide

### Prerequisites
*   **Firmware**: [VS Code](https://code.visualstudio.com/) + [PlatformIO IDE](https://platformio.org/).
*   **Python Monitor**: Python 3.10+ with `pyserial` and `tkinter`.

### Compilation & Flash Commands
Navigate to the desired subdirectory (`RoboTop`, `RoboSub`, `RoboLora`) and run PlatformIO CLI commands:

```bash
# Build the project
pio run

# Upload via USB Serial
pio run -t upload

# Monitor Serial Output
pio device monitor
```

### Over-The-Air (OTA) Updates
The Top and Sub units support over-the-air firmware updates using ESP32's built-in OTA:
1.  Connect your development machine to the buoy's local WiFi Access Point.
2.  Query active OTA-enabled ports:
    ```bash
    pio device list --mdns
    ```
3.  Upload using the discovered IP:
    ```bash
    pio run -t upload --upload-port <DISCOVERED_IP>
    ```

### Running the Python GUI Dashboard
Install dependencies and run:
```bash
pip install pyserial
python RoboPythonDisplay/RoboControl.py
```

---

## 📜 License & Contributions
This project is proprietary and confidential. For contributions, pull requests, or feature requests, contact **Peter de Nijs** (`pnn69pnn@gmail.com`).
