# NicE-Buoy Autonomous Marine System Firmware

Welcome to the master repository for the **NicE-Buoy Autonomous Marine System**. This codebase powers a highly reliable, dual-processor autonomous marine buoy designed for GPS lock-to-position, remote configuration, automated sailing, and live telemetry tracking over LoRa and WiFi (UDP).

---

## 📌 Project Overview

NicE-Buoy is an industrial-grade marine robotic system consisting of multiple hardware and software components. Each buoy uses a **dual-ESP32 architecture** split into a **Top Unit (RoboTop)** and a **Sub Unit (RoboSub)**, connected by a single-wire half-duplex serial link. This physical decoupling isolates high-frequency navigation, radio communication, and web server tasks (Top) from raw thruster driving, compass polling, and the PID motor loops (Sub).

Alongside the buoys sit two shore-side controllers: **RoboCYD**, a touchscreen unit that doubles as the field WiFi access point, and **RoboLora**, a USB/WiFi LoRa gateway.

---

## 🏗️ System Architecture

```
        ┌───────────────────────────┐        ┌───────────────────────────────┐
        │  RoboPythonDisplay (PC)   │        │  RoboCYD (touchscreen)        │
        │  Tkinter dashboard        │        │  - IS the field AP Robo_WiFi  │
        └────────────┬──────────────┘        │  - LoRa RX task + web UI      │
                     │ UDP / LoRa            └───────────────┬───────────────┘
                     │                                       │
        ┌────────────▼──────────────┐                        │
        │  RoboLora (gateway)       │   UDP 1001 (protocol)  │
        │  USB + WiFi <-> LoRa      │   UDP 1002 (debug log) │
        └────────────┬──────────────┘                        │
                     └───────────────┬───────────────────────┘
                                     │  LoRa 868 MHz  +  WiFi/UDP
┌────────────────────────────────────▼────────────────────────────────────────┐
│                                  RoboTop                                    │
│  - GPS polling (TinyGPS++, 0,0 placeholder rejected)                        │
│  - LoraTask: RF telemetry, ACK retry table (10 slots)                       │
│  - WiFiTask: web dashboard, UDP broadcast, OTA, captive portal, mDNS        │
│  - SercomTask: bridge to the Sub          - Button, buzzer, RGB LEDs        │
└────────────────────────────────────┬────────────────────────────────────────┘
                                     │  single-wire half-duplex UART
                                     │  Top TX 22 / RX 21  <->  Sub RX 5 / TX 18
┌────────────────────────────────────▼────────────────────────────────────────┐
│                                  RoboSub                                    │
│  - Dual ESC thrusters (BB & SB)     - ICM-20948 9-DoF compass + fusion      │
│  - Rudder & speed PID loops         - Owns ALL tuning in NVS                │
│  - Its own web dashboard (24 endpoints) and OTA                            │
└─────────────────────────────────────────────────────────────────────────────┘
```

The codebase is structured into the following decoupled sub-projects:

1.  **`RoboTop` (Top Unit)** — ESP32, FreeRTOS. Acquires GPS, hosts the HTML5 control dashboard, broadcasts telemetry over UDP and LoRa, and delegates motor execution to the Sub.
    Tasks: `GpsTask`, `LoraTask`, `WiFiTask`, `SerialTask` (SercomTask), `buttonTask`, `LedTask`, `buzzTask`.
2.  **`RoboSub` (Sub Unit)** — ESP32, FreeRTOS. Drives the thrusters, reads the compass, runs the PID loops, and is the **authoritative owner of all tuning parameters** (see below).
    Tasks: `CompassTask`, `EscTask`, `SerialTask`, `WiFiTask`, `LedTask`, `buzzTask`. The rudder and speed PIDs live in `pidrudspeed.cpp`.
3.  **`RoboCYD` (Touchscreen Controller)** — ESP32 + 320x240 TFT with resistive touch. Displays the fleet, sends commands, and **raises the field access point `Robo_WiFi`** when no home network is present. Has a dedicated LoRa receive task and serves both a dashboard and a standalone fullscreen map.
4.  **`RoboDependency` (`RoboCompute` + `RoboTone` libraries)** — shared code linked by Top and Sub. Navigation maths (`RouteToPoint`), the unified `RoboStruct`, `formatFloat`, and the telemetry encoder/decoder (`rfCode` / `rfDeCode`).
5.  **`RoboLora` (LoRa Gateway)** — USB-and-WiFi to LoRa gateway with its own web UI. Relays operator commands to the buoys; stamps its outgoing frames with sender ID `0x99`.
6.  **`RoboPythonDisplay` (Python GUI Monitor)** — Tkinter desktop dashboard (`RoboControl.py`) for up to 3 buoys: windroses, thruster indicators, battery voltages, PID menus.
7.  **`RoboChargingStation`** — solar monitoring and charging dock firmware.

### Who owns what

Worth stating plainly, because it drives a lot of the design:

*   **The Sub owns the tuning.** PID gains, compass offset, `holdRad`, thruster inversion and swap all live in the Sub's NVS. The Top holds a RAM copy only, fetched via `SETUPDATA` when the serial link comes up, and shown on the dashboards until the Sub reports again.
*   **`compassOffset`, `revBB`, `revSB` and `swap_BB_SB` are hull-specific** — they describe the physical boat, not the electronics. Move a Sub to another hull and these travel with it and will be wrong until re-entered.
*   **The Sub has no LoRa.** Its only path to the outside world is the serial line to its Top.

---

## 🌐 Network Layout

Every node follows the same priority list:

```
NicE_WiFi  ->  Robo_WiFi  ->  its own AP
```

*   **At home** everything joins `NicE_WiFi`.
*   **In the field**, when no `NicE_WiFi` is in reach, the **CYD becomes `Robo_WiFi`** (password `geenanker`, `192.168.1.1/24`, 8 client slots) and the Tops and RoboLora join it.
*   **A node that finds neither** raises its own AP — `TOP_<id>`, `SUB_<id>`, `LORA_<id>`, password `geenanker`, on `192.168.4.1` — and keeps hunting underneath it. Joining that AP opens the dashboard automatically via a captive portal.
*   **The Sub deliberately skips `Robo_WiFi`**, to keep the CYD's client slots for the Tops and a phone. It costs nothing: the Sub never transmits on UDP anyway, and all of its telemetry reaches you folded into the Top's `TOPDATA`.

**mDNS** gives every node a stable name that works in both modes — `top-<id>.local`, `sub-<id>.local`, `robocyd.local`, `lora-<id>.local`. ArduinoOTA registers the responder; nothing else may call `MDNS.begin()`, as a second responder has crashed this hardware before.

**UDP ports:** `1001` carries the protocol; `1002` carries a plain-text debug log (see the RoboTop README).

---

## 📡 Communication Protocol & Frame Format

The system communicates using a lightweight, proprietary, ASCII-based comma-separated protocol designed to optimize RF bandwidth while remaining human-readable.

### Frame Anatomy
```
$TargetID,SenderID,AckType,CommandType,Status,Param1,Param2,...,ParamN*CRC\n
```

*   **`$`**: Starting sentinel character.
*   **`TargetID`**: Address of the recipient. **All IDs are hexadecimal.** A buoy's ID is the low four bytes of its MAC (e.g. `b7a5b578`). `1` (`BUOYIDALL`) is the broadcast address; `0` is treated as a legacy broadcast.
*   **`SenderID`**: Address of the transmitting station. `99` (i.e. `0x99`) is the web UI / RoboLora and `98` is the CYD display. **These two are the only senders whose broadcasts a buoy will act on** — a broadcast command from a peer buoy is ignored, which is what stops one buoy commanding another.
*   **`AckType`**: Packet type classification:
    *   `1 (GET)`: Request parameter values.
    *   `2 (SET)`: Modify parameter values.
    *   `3 (GETACK)`: Request command execution with receipt verification.
    *   `4 (ACK)`: Receipt confirmation.
    *   `6 (INF)`: Telemetry status update, or an *answer* to a question.
*   **`CommandType`**: Integer mapping to system command (e.g., `51` for `TOPDATA`, `19` for `BUOYPOS`, `83` for `SETUPDATA`, `47` for `DIRDIST`, `21` for `LOCKPOS`, `23` for `DOCKPOS`).
*   **`Status`**: Current state machine status of the buoy (`7 IDLE`, `13 LOCKED`, `16 DOCKED`, …).
*   **`Params`**: Dynamic floating-point or integer parameters depending on the command.
*   **`*`**: End-of-payload delimiter.
*   **`CRC`**: 2-character hexadecimal CRC calculated by XOR'ing all characters between `$` and `*`.

> **Broadcasts that expect an ACK are a trap.** An ACK for `IDr = BUOYIDALL` can never match in `removeAckMsg()`, so a `SET` or `GETACK` broadcast sits in the LoRa retry table for all five retransmits and blocks a slot. Broadcast beacons must use `INF`.

---

## 🚀 Advanced Telemetry Optimizations

### 1. Bandwidth Compression
To maximize LoRa airtime efficiency and range, telemetry strings undergo two compression steps in `RoboCompute.cpp`:
*   **Float Trailing Stripping**: `formatFloat()` trims redundant trailing zeros and decimal points (e.g. `12.5000` becomes `12.5`).
*   **Zero Compression**: `RoboCode()` compresses consecutive zero fields to empty strings (e.g. `,0,0,0,0,` becomes `,,,,,`).

### 2. The Top-to-Sub Link, and Why One-Way Failures Matter
The Top and Sub share **one wire**, half duplex, with hardware separating the two directions (opto-isolated; see `Datasheets/`). The firmware configures two GPIOs per board — Top TX 22 / RX 21, Sub RX 5 / TX 18 — but that is **not** two conductors.

*   **The diagnostic consequence:** if the connector or the wire fails, traffic stops in *both* directions. A failure in **one direction only** therefore cannot be the plug — it is in the direction-separation circuitry, which is per board. Swapping a Sub between hulls localises such a fault to a specific board.
*   `SercomTask` implements **echo filtering**, discarding frames whose sender ID is its own, because a transmitter hears itself on a shared wire.
*   **Implicit ACKs**: any valid `INF` response from the Sub signals that it processed the command, clearing the retry queue without extra traffic.
*   **The Sub sleeps.** After reconnection it sits in power-down and only wakes on an incoming command — send one, wait, then it communicates. Do not read a quiet Sub as a fault until it has been nudged.

### 3. Hardware Self-Healing Watchdogs
*   **LoRa transceiver watchdog**: if transmission fails for 500 ms the SPI/state machine is assumed locked and `InitLora()` forces a full re-init.
*   **Compass watchdog**: static I2C readings under thruster power trigger `COMPASS STUCK - REINIT`.
*   **Serial watchdog**: the Top clears `isSerConnected` after 2 s of silence and re-requests `SETUPDATA` when the Sub returns.
*   **Stack headroom is observable.** The Top's `/data` reports `StkLoop`, `StkLora`, `StkWifi`, `StkSer` and heap figures. A LoRa task running on 100 bytes of stack was the cause of a hard-to-find panic; check these after any change that adds work to a task.

---

## 🧭 In-Field Navigation Calibration State Machines

1.  **In-Field Compass Spin Calibration** — self-guided rotation while measuring magnetic min/max to derive hard-iron offsets.
2.  **In-Field Offset Alignment** — sails a straight leg on compass heading, compares the GPS track, and writes the difference to the stored compass offset.
3.  **Guided 8-Point Compass Calibration** — the hull is turned to the compass's own zero and then indexed round a mechanical fixture 45° at a time, one capture per mark. Produces the 8-point correction table, which is held on the Sub and reported through `SETUPDATA`. The table describes the hull's magnetic deviation only; which way the sensor is bolted in is a separate setting (**Set as North**), applied after the table.

    A GPS-based version of this used to exist, sailing eight 100 m legs and inferring the table from the track. It was removed when the mounting offset moved out of the table's input — its residual arithmetic assumed the old ordering.

---

## 🛠️ Developer Guide

### Prerequisites
*   **Firmware**: [VS Code](https://code.visualstudio.com/) + [PlatformIO IDE](https://platformio.org/).
*   **Python Monitor**: Python 3.10+ with `pyserial` and `tkinter`.

### Compilation & Flash Commands
```bash
pio run                      # build
pio run -t upload            # upload (OTA by default for Top and Sub)
pio device monitor           # serial monitor
```

### Over-The-Air (OTA) Updates
`RoboTop` and `RoboSub` default to `upload_protocol = espota`, so a plain upload goes over the network:

```bash
pio run -t upload --upload-port 192.168.1.71        # by address
pio run -t upload --upload-port top-b7a5099c.local  # or by mDNS name
```

`RoboCYD` and `RoboLora` have **no espota configuration** in their `platformio.ini` (their `upload_port` names a COM port that may not exist), so flash those directly:

```bash
python ~/.platformio/packages/framework-arduinoespressif32/tools/espota.py \
       -i <ip> -p 3232 -f .pio/build/<env>/firmware.bin -r
```

A RoboTop change that touches `data/` also needs `pio run -t uploadfs` — the web page lives in SPIFFS.

> Renaming an SSID or changing the WiFi policy must be rolled out to **the whole fleet at home on `NicE_WiFi`**. A device left on the old scheme cannot be reached in the field.

### Running the Python GUI Dashboard
```bash
pip install pyserial
python RoboPythonDisplay/RoboControl.py
```

---

## 📜 License & Contributions
This project is proprietary and confidential. For contributions, pull requests, or feature requests, contact **Peter de Nijs** (`pnn69pnn@gmail.com`).
