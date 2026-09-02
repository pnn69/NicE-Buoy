# RoboSub — Autonomous Marine Buoy Subunit Firmware

The **`RoboSub`** firmware runs on the submerged ESP32 microcontroller of the NicE-Buoy Autonomous Marine System. Acting as the low-level physical execution and navigation engine, it processes raw sensor data, executes autonomous PID navigation loops, and drives the thrusters via Electronic Speed Controllers (ESCs).

---

## 🏗️ Subsystem Architecture & FreeRTOS Flow

RoboSub utilizes FreeRTOS to manage real-time tasks across the dual cores of the ESP32:
*   **Core 1 (High-Priority Navigation & Sensors)**: Processes timing-critical I2C sensor fusion (`CompassTask`), 100Hz PID locomotion loops (`EscTask`), and single-wire serial bridge decoding (`SercomTask`). This guarantees isolated, high-precision timing away from Wi-Fi overhead.
*   **Core 0 (Communications & Visuals)**: Manages Wi-Fi networking client hooks (`WiFiTask`), addressable LED status animations (`LedTask`), and status buzzer audio signals (`buzzerTask`).

```
┌────────────────────────────────────────────────────────────────────────┐
│                                RoboSub                                 │
├───────────────────────────────────┬────────────────────────────────────┤
│              Core 0               │               Core 1               │
├───────────────────────────────────┼────────────────────────────────────┤
│ - WiFiTask (Local AP Web Server)  │ - CompassTask (ICM-20948 Fusion)   │
│ - LedTask (NeoPixel Animations)   │ - EscTask (100Hz PID Loco Loops)   │
│ - buzzTask (Audio Sounder Beeps)  │ - SerialTask (single-wire bridge)  │
└───────────────────────────────────┴────────────────────────────────────┘
```

```
                      ┌────────────────────────────────────────┐
                      │             RoboTop (UART)             │
                      └───────────────────┬────────────────────┘
                                          │ one wire, half duplex
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
│           CompassTask          │ │        EscTask         │ │        LedTask         │
│  - ICM-20948 I2C Fusion (69/68)│ │  - Rudder + speed PID  │ │  - Status NeoPixels    │
│  - Madgwick AHRS Filter (100Hz)│ │    (pidrudspeed.cpp)   │ │  - Battery Telemetry   │
│  - 3D Copilot Web Calibration  │ │  - Differential mix    │ └────────────────────────┘
└────────────────────────────────┘ │  - Port (BB) / Stbd(SB)│
                                   └────────────────────────┘

     There is no separate PIDTask. The loops live in pidrudspeed.cpp and are
     driven from EscTask, which also writes the ESC duty cycles.
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
*   **Active Calibration & Fusion Modes (`icm_mode`)**: Users can dynamically select between four mathematical fusion modes on the web panel depending on operational conditions:
    *   **Mode 1: Only Hard Iron (No Soft, No Tilt)**: Applies simple hard-iron offsets ($hi_{x,y,z}$) to center the magnetic sphere back to $(0,0,0)$ on the local 2D plane. Ideal for basic diagnostic checks.
    *   **Mode 2: Hard & Soft Iron (No Tilt)**: Applies both centering offsets and soft-iron scaling matrices ($si_{x,y,z}$) to deform the magnetically squeezed ellipsoid back into a perfect sphere. Assumes the buoy remains physically level (flat-water).
    *   **Mode 3: Hard Iron & Pitch + Roll (No Soft)**: Applies centering offsets and standard 3D trigonometric pitch ($p$) and roll ($r$) tilt-compensation. Recommended if soft-iron calibration is uninitialized but the buoy is rocking.
    *   **Mode 4: Hard & Soft Iron & Pitch + Roll (Default)**: The most advanced mode. Combines 3D hard-iron offsets, soft-iron scaling, and live geometric pitch and roll projections to calculate extremely precise headings even during severe angular tilting in turbulent seas.
*   **Tilt-Compensated Magnetometer Math**: When Mode 3 or 4 is active, the system calculates the pitch ($p$) and roll ($r$) vectors from the low-passed accelerometer and projects the 3D magnetometer readings back onto a virtual horizontal plane:
    $$X_h = m_x \cos(p) + m_y \sin(r)\sin(p) + m_z \cos(r)\sin(p)$$
    $$Y_h = m_y \cos(r) - m_z \sin(r)$$
    This resolves the exact heading as: $\text{Heading} = \text{atan2}(Y_h, X_h) \times \frac{180}{\pi}$.
*   **NVS Persistence & Hard/Soft Iron Scaling**: Loads custom 3D calibration matrices (Hard Iron: `hi_x`, `hi_y`, `hi_z`; Soft Iron: `si_x`, `si_y`, `si_z`) from persistent Preferences NVS (Flash) to scale and offset raw magnetic distortion.
*   **3D Copilot Interactive Calibration Web Dashboard**:
    *   Integrates an interactive **3D Compass Calibration Tool** served via HTTP from the ESP32.
    *   Features real-time 3D plotting and feedback to help operators execute figure-eight and pitch/roll movements (Z-span checking) in the field.
    *   Validates point count (minimum 500 points) and multi-axis coverage before letting users commit the calculated parameters directly to NVS, playing an audio cue upon successful calibration storage.
*   **Highly Stable Averaging Filter**: Runs heading outputs through a low-pass Exponential Moving Average (EMA) filter and caches them into a 25-step circular averaging buffer (`NUM_DIRECTIONS 25`), smoothing out MEMS noise and momentary magnetic anomalies caused by high-power thruster surges.

### 3. Dual-Loop PID Navigation (`pidrudspeed.cpp` / `pidrudspeed.h`)
Autonomy is governed by independent, high-speed PID loops running at **100Hz** in `pidrudspeed.cpp`, driven from `EscTask`:
*   **Speed PID**: Computes the required thrust based on the remaining distance to the target waypoint.
*   **Rudder PID**: Calculates the required heading correction to align the buoy's current heading with the target direction.
*   **Navigation Modes**:
    *   `IDLE`: Thrust output disabled; motors neutral.
    *   `MANUAL`: Bypasses PID control to allow direct manual piloting over LoRa or WiFi.
    *   `LOCKED`: Autonomous position-holding (GPS Anchor) using localized coordinate boundaries.
    *   `DOCKING`: Automated waypoint navigation to guide the buoy to its charging station or home harbor.

### 4. UART Serial Interface (`sercom.cpp` / `sercom.h`)
RoboSub communicates with the `RoboTop` over **one shared wire**, half duplex, with hardware
separating the two directions (opto-isolated). Sub **RX 5 / TX 18**, Top **TX 22 / RX 21** - two
GPIOs per board, but not two conductors.

> **Diagnostic consequence:** a bad connector or wire stops traffic in *both* directions. A failure
> in **one direction only** is in the direction-separation circuitry, which is per board. Swapping
> the Sub between hulls localises such a fault to a specific board - that is how a dead receive
> side was traced to one Sub rather than to its Top or the cable.

*   **Half-Duplex Echo Filtering**: Because the RX and TX lines are tied together, the transmitting ESP32 receives its own transmissions. `SercomTask` filters out these loopback echoes by comparing the sender MAC address against its own address, discarding self-transmitted frames automatically.
*   **Implicit Acknowledgments**: Every periodic `INF` telemetry response from RoboSub serves as an implicit acknowledgment for commands, which reduces communication overhead and prevents packet congestion.

### 5. Wi-Fi Connectivity & Dashboard Web Server (`subwifi.cpp` / `subwifi.h`)
RoboSub hosts a lightweight local web server to display real-time telemetry diagnostics and host the interactive 3D compass calibration tool:

*   **Priority list**: `NicE_WiFi` (password `!Ni1001100110`) then its own **`SUB_<id>`** AP
    (password `geenanker`, `192.168.4.1`, captive portal). No scanning - the SSID is tried directly
    with a blind `WiFi.begin()`, because a scan blocks for seconds and drags the single radio off
    the AP channel.
*   **The Sub deliberately skips `Robo_WiFi`**, the CYD's field AP, to keep its client slots for the
    Tops and a phone. Nothing is lost by this - see below.
*   **Home is tried once, at boot.** If `NicE_WiFi` is not in reach the Sub raises `SUB_<id>` and
    stays on it until the next reboot - no periodic re-hunting. The old build retried every 45 s,
    and because the ESP32 has one radio each attempt pulled it off the AP's channel for up to 15 s,
    so the `SUB_<id>` AP kept vanishing from the phone's WiFi list and coming back. A Sub that *did*
    join home at boot still re-joins if the link drops, but the first re-join that fails raises its
    own AP and ends the hunt for good. To get back onto `NicE_WiFi`, power cycle.
*   **mDNS**: reachable as `sub-<id>.local` in both modes. Only `ArduinoOTA.begin()` may initialise
    the responder - a second one has crashed this hardware.
*   **The Sub never transmits on UDP.** The broadcast in the WiFi loop is commented out, and the
    `udpIn` queue is filled but never drained, so no UDP command reaches the handler either. All of
    the Sub's telemetry reaches you folded into the Top's `TOPDATA`, heading and battery included.
*   **Web Services**: `/data` (JSON telemetry) and `/params` (stored tuning), plus the compass
    calibration endpoints.
*   **Debug channel**: plain text on **UDP 1002**, silent at idle. Prints `SER in cmd=..` for every
    frame arriving on the serial link - the quickest way to prove whether the Sub's receiver works
    without opening the hull.

### 5b. Power-down behaviour
After reconnection the Sub sits in **power-down** and only wakes on an incoming command: send one,
wait, then it communicates. Do not read a quiet Sub as a fault until it has been nudged. It is
awake, though, if it is answering - a Sub that acknowledges several commands while its Top still
reports `SubOk=false` is a real one-way break, not sleep.

### 5c. The Sub owns the tuning
Every parameter in the NVS section below lives here. The Top holds a RAM copy only, fetched with
`SETUPDATA` when the serial link comes up. **`compassOffset`, `revBB`, `revSB` and `swap_BB_SB` are
hull-specific** - they describe the physical boat, not this board - so moving a Sub to another hull
carries the wrong values with it until they are re-entered.

### 5d. Set as North belongs to the buoy

`SET_AS_NORTH` (87) is solved here and nowhere else, by `computeSetAsNorthOffset()`: it takes the
heading out of the pipeline *after* the compass table and the trim, negates it, and stores the
result as `compassOffset` in NVS. Absolute, not incremental — it does not read the previous offset —
so pressing it twice is idempotent and a command delivered over both LoRa and UDP does no harm.

That absoluteness is the point. The retired `compassOffset - dirMag` needed three or four presses to
creep onto north; this lands on it first time. The handheld carried a copy of the old formula until
recently and did its own arithmetic locally, which is why Set as North worked from the Top's web
page and not from the CYD. Both now send the command and let the buoy answer it. **Do not
re-implement this solve at the sending end** — the offset depends on the state of the table and the
trim, which only the Sub knows.

`SET_AS_LEVEL` (93) follows the same rule for the attitude datum.

> `main.cpp`'s command filter accepts **any** `SETUPDATA` regardless of addressing
> (`|| dataIn.cmd == SETUPDATA`). Harmless while the Sub has no UDP input path, but worth
> remembering before one is added.

---

## 🎛️ Peripheral Controllers

*   **Battery Telemetry (`adc.cpp`)**: Reads battery voltages via ESP32 ADC pins, applies smoothing filters to eliminate voltage sags from thruster surges, and logs the cell power telemetry.
*   **Sounder Feedback (`buzzer.cpp`)**: Drives a piezo sounder, emitting audible frequencies and beep sequences to signal boot status, profile loading status, and save actions.
    *   **A save now chirps (`beep(6)`, 2000 Hz for 80 ms).** A save made during a calibration
        happens with somebody stood over the hull and nowhere near a screen, so "did that take?" is
        a question only the buoy can answer. Fired after the `MEM_PUT`s and the reload, so it means
        the values are in NVS rather than that a frame arrived — and **debounced at one second**,
        because a SAVE goes out on LoRa *and* UDP and the Top forwards both copies a few hundred
        milliseconds apart. Two chirps for one press would read as two saves.
    *   **Its own tone, not a reuse.** Every neighbour already means something: `10` is the
        button-press tick, `1` the rising tune for a computation that produced a result, `5` the
        compass-table tune, `-1` failure.
    *   **`SET_AS_NORTH` was playing `beep(-1)` on success** — the failure tone, the one a buoy plays
        when a start line cannot be computed. It stores the compass offset in NVS, so it plays the
        save chirp now. The same mix-up had already been found and fixed in `subwifi.cpp`; a good
        save has to sound like one, or the operator learns to ignore the buzzer altogether.
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
*   **Mounting Alignment (`compassOffset`)**:
    *   `compassOffset` (Default: `0.0`): Mounting angular offset calibration (set by "Set as North") to align the physical thruster output with magnetic North.
    *   There is deliberately **no separate declination setting**. It existed once, but nothing ever
        applied it to a heading and nothing could set it, and it is redundant anyway: "Set as North"
        is measured against a real bearing, so whatever declination applies where you sail is
        already inside `compassOffset`. Two knobs for one correction only invites applying it twice.
        The `SET_DECLINATION`/`STORE_DECLINATION` commands still exist on the wire so command
        numbering is unchanged, but both ends now ignore them.
*   **Hard & Soft Iron 3D Calibration Matrices (`magHard`, `magSoft`)**:
    *   `magHard[3]` (NVS Keys: `mH00` to `mH02`): 3-axis offset vector used to correct Hard Iron distortions.
    *   `magSoft[3][3]` (NVS Keys: `mS00` to `mS22`): 3x3 scaling matrix used to normalize Soft Iron spatial distortions, generated by the 3D Copilot calibration tool.

### 4. Adaptive Boundary Keep Limits
*   **Hold / Anchor Radius (`holdRad`)**:
    *   `holdRad` (Default: `2.0`): Specifies the circular anchor boundary (in meters) for autonomous station keeping. If the buoy sails inside this radius, thrust is cut to neutral to save battery.

---

## 🛠️ Building & Flashing

RoboSub is built and managed using the **PlatformIO** ecosystem:

Environment **`robo-esp-v3`** (`board = esp32dev`). Libraries: `RoboCompute`, `RoboTone`, PID,
`ESP32Servo`, `FastLED`, SparkFun ICM-20948, Madgwick.

```bash
pio run                                              # build
pio run -t upload                                    # OTA, default 192.168.1.189
pio run -t upload --upload-port sub-fe914828.local   # a different unit
```

`upload_protocol = espota`: the Subs live in sealed hulls and are not on USB, so serial was never
the realistic path. A Sub that has just been reconnected may need a command first to wake it before
it will answer.
