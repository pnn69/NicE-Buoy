# RoboTop — Autonomous Marine Buoy Master Supervisor Firmware

The **`RoboTop`** firmware runs on the master ESP32 microcontroller of the NicE-Buoy Autonomous Marine System. It serves as the primary intelligence, networking, and supervisory node, managing global positioning, wireless telemetry channels, client-facing control dashboards, and delegating physical motor actions to the Sub Unit (`RoboSub`).

---

## 🏗️ Architecture & Core Tasks

RoboTop leverages FreeRTOS to coordinate real-time operations across the ESP32's dual cores:
*   **Core 0 (Communications & Serial Bus)**: Dedicated to hosting the web server, broadcasting UDP telemetry, running wireless client WebSockets, and managing RS-485 half-duplex serial bus scheduling (`SercomTask`).
*   **Core 1 (GPS, Math & Long-Range RF)**: Focuses on high-precision GPS polling (UART), Great-Circle navigation math calculations, and SPI-driven long-range LoRa RF operations (`LoraTask`).

```
┌────────────────────────────────────────────────────────────────────────┐
│                                RoboTop                                 │
├───────────────────────────────────┬────────────────────────────────────┤
│              Core 0               │               Core 1               │
├───────────────────────────────────┼────────────────────────────────────┤
│ - WiFiTask (Local AP Web Server)  │ - GpsTask (TinyGPS++ Polling)      │
│ - SercomTask (RS-485 Serial Bridge)│ - LoraTask (SPI long-range RF)     │
│ - UDP Broadcasting (Port 1001)    │ - Route Computation (Great-Circle) │
└───────────────────────────────────┴────────────────────────────────────┘
```

---

## ⚡ Core Functional Modules

### 1. GPS Acquisition & Navigation Telemetry
*   **Hardware Interface**: Interlines with a GPS module (e.g., NEO-8M) over UART, polling raw NMEA sentences.
*   **Sentence Parsing**: Leverages the `TinyGPSPlus` parser to extract exact latitude, longitude, heading, and Speed Over Ground (SOG).
*   **Safety Filtering**: Implements outlier detection and HDOP (Horizontal Dilution of Precision) filtering to ignore coordinate spikes caused by satellite multipath interference in marine environments.

### 2. Long-Range LoRa Telemetry Gateway (`loratop.cpp`)
*   **Hardware Interface**: Interlines with an SPI-driven LoRa transceiver to establish long-range bidirectional telemetry channels with the shore-based monitoring station.
*   **Retry & ACK Protocol**: Implements a dedicated transmission buffer and acknowledgement checking loop. Relies on structured ASCII frames and verification IDs.
*   **Self-Healing SPI Watchdog**: Monitors SPI transaction states. If packet transmissions hang or fail consecutively over a 500ms window, the system automatically forces a full hardware and software reset of the LoRa registers (`InitLora()`).

### 3. Wi-Fi Access Point & Web Sockets (`topwifi.cpp`)
*   **Dual Wi-Fi Modes**: Can be configured as a standalone **Access Point (AP)** (allowing users to connect directly to the buoy in the water) or **Station (STA)** (connecting to an existing local infrastructure network).
*   **HTML5 Dashboard**: Serves a highly interactive, responsive control web-dashboard from ESP32 local storage.
*   **Asynchronous WebSockets**: Streams high-frequency telemetry (heading, speed, position, PID error states) directly to connected client browsers.
*   **UDP Broadcast Server**: Periodically broadcasts system status strings over UDP (Port 1001) to enable instant discovery and monitoring by nearby desktop dashboards.
*   **Optimized Dual-Rate Transmit Scheduler**: Implements dynamic rate separation in the supervisory clock:
    *   **UDP/WiFi Telemetry (High-Frequency)**: Telemetry is transmitted at a rapid **250ms** interval to guarantee fluid, real-time feedback on web monitors.
    *   **LoRa/RF Telemetry (Low-Frequency)**: Telemetry is throttled down to **5000ms (5 seconds)** with randomized collision-avoidance jitter to prevent channel crowding, conserve battery power, and abide by legal RF duty cycles.

### 4. RS-485 Half-Duplex Serial Bridge (`sercom.cpp`)
*   **Physical Decoupling**: Communicates with `RoboSub` over a physical serial line using half-duplex RS-485.
*   **Single-Wire Echo Filtering**: Since physical RX/TX lines are tied, the UART hardware receives its own transmissions. `SercomTask` filters out these loopback echoes by identifying its own MAC ID as the sender, ignoring these self-reflected frames.
*   **Implicit ACK Handling**: Telemetry responses periodically received from the Sub unit are processed as implicit acknowledgements of command delivery, clearing retry buffers and maximizing serial bus availability.

### 5. Automated Regatta Start Line & Track Calculations
To facilitate competitive sailing regattas, RoboTop coordinates the positions of multiple buoys to automatically establish a fair, geometrically synchronized starting line:

*   **Intelligent Triangulation & Role Determination (`calcTrackPos`)**: Rather than relying on static or arrival-order database slots, RoboTop uses geometric triangulation and wind vectors to mathematically determine the roles of the three buoys (`PORT`, `STARBOARD`, or `HEAD`):
    *   **Starting Line Identification**: Computes the Great-Circle distances between all three buoy pairs ($d_{0-1}$, $d_{0-2}$, and $d_{1-2}$). The two buoys with the **shortest distance** between them are mathematically identified as the starting line pins. The remaining, furthest buoy is automatically designated as the upwind **HEAD (windward) buoy**.
    *   **Port vs. Starboard Allocation**: Calculates the geographic bearing between the two starting line pin buoys. It then measures the signed smallest angular difference relative to the live wind direction vector ($W_{\text{dir}}$):
        *   If the signed angle is **positive ($\ge 0^\circ$)**, the first buoy is designated as the **PORT** pin and the second is the **STARBOARD** pin.
        *   If the signed angle is **negative ($< 0^\circ$)**, the first buoy is designated as the **STARBOARD** pin and the second is the **PORT** pin.
*   **Geographical Midpoint & Width Determination**: Identifies the participating start line buoys, computes their exact geographic midpoint using arithmetic coordinate averages, and calculates the total starting line width $d$ (Great-Circle distance) using the Haversine formula.
*   **Wind-Aligned Perpendicular Squaring (`recalcStartLine`)**: Imports the live, filtered average wind direction ($W_{\text{dir}}$) from the master supervisor. To ensure a completely fair start, the starting line must be exactly perpendicular ($90^\circ$) to the wind direction:
    *   **Port End Bearing**: $\theta_{\text{Port}} = (W_{\text{dir}} + 270^\circ) \bmod 360^\circ$
    *   **Starboard End Bearing**: $\theta_{\text{Starboard}} = (W_{\text{dir}} + 90^\circ) \bmod 360^\circ$
*   **Vector Position Projection (`adjustPositionDirDist`)**: Projects the new target coordinates outward from the calculated starting line midpoint along the Port and Starboard bearings by a distance of exactly $d / 2$. This dynamically aligns (squares) the starting line perpendicular to the wind while keeping its original midpoint and length perfectly intact.
*   **Asynchronous Coordination Broadcast (`SENDTRACK`)**: Once starting line or course track parameters are calculated, RoboTop automatically schedules a broadcast over the long-range LoRa RF network (`loraOut`), dispatching updated coordinates to Port, Starboard, and Head buoys simultaneously to coordinate the entire fleet.

### 6. Advanced Circular Wind Telemetry & Standard Deviation
To ensure the supervisor possesses steady, reliable, and noise-resistant wind references in turbulent marine environments, RoboTop employs specialized circular statistics to aggregate and smooth raw wind-vane angles:

*   **Circular Vector Addition (`averageWindVector`)**: Simple arithmetic averaging of angles fails near the $360^\circ$ wrap-around boundary (e.g., the average of $350^\circ$ and $10^\circ$ is mathematically $180^\circ$, but physically $0^\circ$/$360^\circ$). To solve this, RoboTop decomposes each wind angle sample $A_i$ into Cartesian unit vectors:
    $$x_i = \cos(A_i \times \frac{\pi}{180}), \quad y_i = \sin(A_i \times \frac{\pi}{180})$$
    It then aggregates and computes the mean coordinates ($\bar{x}, \bar{y}$), resolving the correct, physically continuous circular average direction using the four-quadrant arctangent function:
    $$W_{\text{dir}} = \operatorname{atan2}(\bar{y}, \bar{x}) \times \frac{180}{\pi}$$
*   **Yamartino Circular Standard Deviation (`deviationWindRose`)**: Traditional linear standard deviation is highly vulnerable to wraps near North. RoboTop implements rigorous circular standard deviation by first calculating the magnitude of the mean resultant vector $R$:
    $$R = \frac{\sqrt{(\sum \cos(A_i))^2 + (\sum \sin(A_i))^2}}{N}$$
    The value $R$ acts as an indicator of wind vector consistency (where $R=1.0$ is perfect alignment and $R=0.0$ is complete dispersal). It then calculates the angular dispersion (circular standard deviation in degrees) using:
    $$W_{\text{std}} = \sqrt{-2 \ln(R)} \times \frac{180}{\pi}$$
    This establishes an incredibly stable, jitter-free Wind Standard Deviation index for navigation planning.

---

## 🛠️ Building & Flashing

RoboTop is built and managed using the **PlatformIO** ecosystem:

1.  **Configuration (`platformio.ini`)**: Targets the `robo-esp-v3` environment and includes dependencies such as `TinyGPSPlus`, `Adafruit_BusIO`, and `WiFiManager`.
2.  **Compilation**:
    ```powershell
    C:\Users\Peter.d.Nijs\.platformio\penv\Scripts\platformio.exe run
    ```
3.  **OTA Upload**:
    The system supports Over-The-Air updates using PlatformIO's OTA upload protocol:
    ```powershell
    C:\Users\Peter.d.Nijs\.platformio\penv\Scripts\platformio.exe run -t upload
    ```\n