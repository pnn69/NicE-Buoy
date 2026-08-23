# RoboTop — Autonomous Marine Buoy Master Supervisor Firmware

The **`RoboTop`** firmware runs on the master ESP32 microcontroller of the NicE-Buoy Autonomous Marine System. It serves as the primary intelligence, networking, and supervisory node, managing global positioning, wireless telemetry channels, client-facing control dashboards, and delegating physical motor actions to the Sub Unit (`RoboSub`).

---

## 🏗️ Architecture & Core Tasks

RoboTop leverages FreeRTOS to coordinate real-time operations across the ESP32's dual cores:

```
┌─────────────────────────────────────┬─────────────────────────────────────┐
│               Core 0                │               Core 1                │
├─────────────────────────────────────┼─────────────────────────────────────┤
│ WiFiTask   8192  web, UDP, OTA      │ LoraTask   8192  SPI long-range RF  │
│ SerialTask 6000  link to the Sub    │ GpsTask    2000  TinyGPS++ polling  │
│                                     │ buttonTask 2048  press detection    │
│                                     │ LedTask    2000  / buzzTask 2048    │
└─────────────────────────────────────┴─────────────────────────────────────┘
                 loop() on core 1: navigation, state machine, RF dispatch
```

The numbers are stack sizes in bytes, and they matter. `LoraTask` was once given 4000 and ran with **100 bytes of headroom**, which tripped the stack canary under load and produced a panic that pointed at every task except the guilty one. Live headroom is published in `/data` as `StkLoop`, `StkLora`, `StkWifi`, `StkSer` — check it after any change that adds work to a task.

---

## ⚡ Core Functional Modules

### 1. GPS Acquisition & Navigation Telemetry
*   **Hardware Interface**: GPS module over UART, polling raw NMEA sentences.
*   **Sentence Parsing**: `TinyGPSPlus` extracts latitude, longitude, heading and speed over ground.
*   **Fix validity**: a fix requires `isValid()` **and** an age under 2000 ms **and** a position that is not `0,0`. `isValid()` alone only means "a position field was parsed" — it stays true across a sentence carrying zeros, and passing that on sends the rest of the firmware chasing a point off the coast of Africa.

### 2. Long-Range LoRa Telemetry (`loratop.cpp`)
*   **Hardware Interface**: SPI LoRa transceiver for bidirectional telemetry with the shore station and the other buoys.
*   **Retry & ACK Protocol**: `pendingMsg[10]` holds frames sent with `ack = GETACK` or `SET`, retransmitting each up to 5 times until acknowledged.
    > **A broadcast must never ask for an ACK.** An ACK for `IDr = BUOYIDALL` can never match in `removeAckMsg()`, so the frame occupies its slot for all five retransmits and re-broadcasts each time. Beacons use `INF`.
*   **Self-Healing SPI Watchdog**: if transmission fails over a 500 ms window the radio is assumed locked and `InitLora()` forces a full re-init.

### 3. WiFi, Web Dashboard and OTA (`topwifi.cpp`)
*   **Priority list**: `NicE_WiFi` → `Robo_WiFi` → its own `TOP_<id>` AP. No scanning — the SSIDs are tried in order with a blind `WiFi.begin()`, because `WiFi.scanNetworks()` blocks for seconds and drags the single radio off the AP channel.
*   **Passwords**: `NicE_WiFi` is `!Ni1001100110`; `Robo_WiFi` and the fallback AP are **`geenanker`**.
*   **The fallback AP keeps hunting.** It runs in `WIFI_AP_STA` on `192.168.4.1` (its own gateway, 8 client slots) and continues looking for a real network underneath — but it will **not** tear itself down while a client is connected.
*   **Captive portal** while in AP mode: wildcard DNS on port 53, so joining `TOP_<id>` opens the dashboard by itself. Disabled as a station, where a 404 must remain a 404.
*   **mDNS**: reachable as `top-<id>.local` in both modes. Only `ArduinoOTA.begin()` may initialise the responder — a second `MDNS.begin()` has crashed this hardware.
*   **HTML5 Dashboard**: served from SPIFFS and streamed straight from the file, not cached in a RAM `String` (a fragmented heap silently truncated the page mid-`<script>`). The page **polls `/data`**; there is no WebSocket on the Top.
*   **Endpoints**: `/` (dashboard), `/data` (JSON telemetry), `/command` (actions).
*   **Dual-rate transmit scheduler**: UDP telemetry every **250 ms** for fluid web feedback; LoRa throttled to **5 s** with randomised jitter for collision avoidance, battery and duty-cycle reasons.

### 4. Single-Wire Half-Duplex Serial Bridge (`sercom.cpp`)
*   **Physical layer**: **one shared wire**, half duplex, with hardware separating the directions (opto-isolated). The firmware names two GPIOs per board — Top **TX 22 / RX 21**, Sub **RX 5 / TX 18** — but that is not two conductors.
    > **Diagnostic consequence:** a bad connector or wire stops traffic in *both* directions. A failure in **one direction only** is therefore in the direction-separation circuitry, which is per board — swap the Sub between hulls to localise it.
*   **Echo filtering**: a transmitter hears itself on a shared wire, so frames whose sender ID is our own are discarded.
*   **Implicit ACKs**: any valid `INF` from the Sub clears the retry buffer.
*   **Watchdog**: `isSerConnected` clears after 2 s of silence and a fresh `SETUPDATA` request is issued when the Sub returns.
*   **The Sub sleeps** after reconnection and only wakes on an incoming command — send one, wait, then it talks.

### 5. Who owns the tuning
The **Sub** owns it. PID gains, `compassOffset`, `holdRad`, `revBB`/`revSB`/`swap_BB_SB` live in the Sub's NVS; the Top keeps a RAM copy fetched with `SETUPDATA` when the serial link comes up. `/data` reports `rev` (the count of replies received) and `SubOk` (serial alive) — `rev: 0` with real values elsewhere means the Top has never heard back and is running on zeroed gains.

### 6. Automated Regatta Start Line & Track Calculations
To facilitate competitive sailing regattas, RoboTop coordinates the positions of multiple buoys to automatically establish a fair, geometrically synchronized starting line:

*   **Intelligent Triangulation & Role Determination (`calcTrackPos`)**: Rather than relying on static or arrival-order database slots, RoboTop uses geometric triangulation and wind vectors to determine the roles of the three buoys (`PORT`, `STARBOARD`, `HEAD`):
    *   **Starting Line Identification**: Computes the Great-Circle distances between all three buoy pairs. The two buoys with the **shortest distance** between them are the starting line pins; the furthest is the upwind **HEAD** buoy.
    *   **Port vs. Starboard Allocation**: Calculates the bearing between the two pins and measures the signed smallest angular difference against the live wind direction $W_{\text{dir}}$. Positive ($\ge 0^\circ$) makes the first buoy **PORT**; negative makes it **STARBOARD**.
*   **Geographical Midpoint & Width**: Computes the pins' midpoint by coordinate average and the line width $d$ by Haversine.
*   **Wind-Aligned Perpendicular Squaring (`recalcStartLine`)**: The line must be exactly perpendicular to the wind:
    *   Port end bearing: $\theta_{\text{Port}} = (W_{\text{dir}} + 270^\circ) \bmod 360^\circ$
    *   Starboard end bearing: $\theta_{\text{Starboard}} = (W_{\text{dir}} + 90^\circ) \bmod 360^\circ$
*   **Vector Position Projection (`adjustPositionDirDist`)**: Projects new targets outward from the midpoint along those bearings by $d/2$, squaring the line to the wind while keeping midpoint and length intact.
*   **Asynchronous Coordination Broadcast (`SENDTRACK`)**: Dispatches updated coordinates to Port, Starboard and Head over LoRa.

### 7. Advanced Circular Wind Telemetry & Standard Deviation
*   **Circular Vector Addition (`averageWindVector`)**: Arithmetic averaging fails at the $360^\circ$ wrap (the mean of $350^\circ$ and $10^\circ$ is $180^\circ$ arithmetically but $0^\circ$ physically). Each sample $A_i$ is decomposed into unit vectors:
    $$x_i = \cos(A_i \times \tfrac{\pi}{180}), \quad y_i = \sin(A_i \times \tfrac{\pi}{180})$$
    and recombined with the four-quadrant arctangent:
    $$W_{\text{dir}} = \text{atan2}(\bar{y}, \bar{x}) \times \tfrac{180}{\pi}$$
*   **Yamartino Circular Standard Deviation (`deviationWindRose`)**: from the mean resultant length $R$
    $$R = \frac{\sqrt{(\sum \cos A_i)^2 + (\sum \sin A_i)^2}}{N}, \qquad W_{\text{std}} = \sqrt{-2 \ln R} \times \tfrac{180}{\pi}$$
    $R = 1$ is perfect alignment, $R = 0$ complete dispersal.

### 8. Command Routing Rules (`handleRfData`)
*   `IDr` is matched against our MAC; `1` (`BUOYIDALL`) and `0` are broadcasts.
*   A **broadcast command is only obeyed when the sender is `0x99` (web/RoboLora) or `0x98` (CYD)**. A broadcast from a peer buoy is ignored — this is what stops one buoy commanding another.
*   Frames that are not for us are **bridged to the other interface** (UDP↔LoRa), never back out the one they arrived on, which would loop.
*   `DOCKPOS` is a *question*. The answer is addressed to the asker and sent as `INF`, and an `INF` frame is never answered — otherwise two Tops answer each other's answers indefinitely.

---

## 🩺 Diagnostics

*   **Debug channel** — plain text broadcast on **UDP 1002**, silent at idle. Carries every non-telemetry RF and serial frame, the DOCK/DOCKPOS step markers and every beep with the line that requested it.
*   **Breadcrumbs surviving a panic** — held in `RTC_NOINIT` memory and reported next boot as `Crumb`, `CrumbLora`, `CrumbWifi`, `CrumbSer`. `90–96` = loop phase, `1000+cmd` = RF command dispatched, `2000+cmd` = serial command, `200–206`/`300–303`/`400–403` = LoRa/Sercom/WiFi task.
    > A **power cycle wipes RTC RAM** and the evidence with it. A reset-pin press or a panic reboot preserves it.
*   **`/data` health fields** — `Uptime`, `ResetReason`, `SubOk`, `rev`, the four `Stk*` figures and `HeapFree` / `HeapMin` / `HeapMaxBlk`.

---

## 🛠️ Building & Flashing

PlatformIO environment **`robo-esp`** (`board = esp32dev`). Dependencies: `RoboCompute`, `RoboTone`, Arduino-PID-Library, TinyGPSPlus, FastLED, arduino-LoRa, ArduinoOTA.

```bash
pio run                                              # build
pio run -t upload                                    # OTA, default 192.168.1.71
pio run -t upload --upload-port top-b7a5b578.local   # a different unit
pio run -t uploadfs                                  # required after any data/ change
```

`upload_protocol = espota`, so a plain upload goes over the network — the Tops are not normally on USB.
