# RoboLora — Long-Range RF USB Gateway Firmware

The **`RoboLora`** firmware runs on the shore-based or vessel-tethered LoRa-to-USB gateway controller. It serves as the physical hardware transceiver link connecting the PC monitoring station with the autonomous buoys over miles of open water.

---

## 🏗️ Gateway Architecture & Functionality

RoboLora manages the relay of long-range telemetry frames with low latency, bridging long-range RF signals directly into a physical PC COM interface:

```
┌─────────────────┐             ┌───────────────────┐             ┌─────────────────┐
│     RoboSub     │   LoRa RF   │     RoboLora      │ USB Serial  │RoboPythonDisplay│
│       or        │◀───────────▶│   - LoraTask      │◀───────────▶│    (PC GUI)     │
│     RoboTop     │             │   - SercomTask    │             │                 │
└─────────────────┘             └───────────────────┘             └─────────────────┘
```

---

## ⚡ Core Functional Modules

### 1. LoRa Physical Transceiver Layer (`LiLlora.cpp` / `LiLlora.h`)
*   **Hardware Interface**: Controls an SPI-connected LoRa transceiver (e.g., RFM95/SX1278) to broadcast and receive long-range RF packets.
*   **Parameter Optimization**: Implements full runtime setting configurations for carrier frequency, signal bandwidth, spreading factors, and coding rates to maintain reliable link budgets in harsh weather conditions.
*   **Asynchronous Polling**: Runs an optimized FreeRTOS background task to monitor the LoRa FIFO buffer, raising real-time interrupts upon packet arrivals to prevent buffer overflows.

### 2. Local OLED Diagnostic Screen (`oled_ssd1306.cpp` / `oled_ssd1306.h`)
*   **Hardware Interface**: Drives a localized monochrome SSD1306 OLED screen over I2C.
*   **Live Status Monitoring**: Displays vital real-time diagnostics including:
    *   Last received packet payload summary.
    *   Transceiver signal health: **RSSI** (Received Signal Strength Indication) and **SNR** (Signal-to-Noise Ratio).
    *   Local battery voltages (measured via internal ADC).
    *   System operational state markers.

### 3. USB-to-UART Bridging (`sercom.cpp` / `sercom.h`)
*   **High-Speed Uplink**: Implements a high-baudrate (115200bps) UART-to-USB bridge that streams decoded ASCII comma-separated packages directly into the host PC's operating system.
*   **Control Reception**: Translates downstream command strings originating from the desktop Python dashboard and packages them into target-specific LoRa transmission frames.

### 4. Advanced Web-Based Control Panel (`data/` / `controlwifi.cpp`)
*   **Captive Portal & DNS Automatic Redirection**: Integrates a lightweight `DNSServer` running on standard UDP Port 53 in Access Point mode. All DNS requests are resolved and redirected to the ESP32's local AP IP address (`192.168.1.84`). Combined with an HTTP 302 redirect for unrecognized URLs, this forces connected operating systems (iOS, Android, Windows, macOS) to automatically launch the localized HTML5 dashboard control panel.
*   **Intuitive Dashboard Interface**: Serves a highly customized HTML5 dashboard (`index.html`, `index.js`, `style.css`) from the local SPIFFS partition over an integrated Wi-Fi Access Point or Local Station.
*   **Dual-Source Telemetry Prioritization**: Real-time client-side filter prioritizes high-speed UDP Wi-Fi data (updating fluidly at **250ms** intervals) and ignores slower, redundant LoRa packets (still logged in their respective consoles) to eliminate UI lag or gauge flickering.
*   **Responsive Dual-Column Configuration Popup**: Restructured the setup modal into a dual-column layout on wider screens, placing Speed PID vertically under Rudder PID, and featuring highly legible, enlarged labels and values in bold blue monospace. Includes:
    *   Interactive **Set as North** auto-offset calibration based on the buoy's live heading.
    *   Rudder and Speed PID coefficients, speed limits, battery boundaries, and motor option toggles (motor swapping, reverse BB/SB).
*   **Isolated WebSockets Logging**: WebSocket messages are prefixed with `LORA:` and `UDP:` to safely route incoming radio traffic and local network broadcasts to separate browser monitors without console pollution.

### 5. Transmission Safety & Loop Protections (`main.cpp` / `sercom.cpp`)
*   **Telemetry Feedback Loop Prevention**: Patched a firmware bug in `fillBuoyArr` where newly discovered buoys copied telemetry directly without clearing active `cmd`/`ack` properties. This now guarantees the system does not get locked in infinite re-transmission loops of stale command packets.
*   **Serial Commands-Only Forwarding Filter**: Configured `SercomTask` (`src/sercom.cpp`) to only forward actual commands (`GET`, `SET`, `GETACK`) from the USB Serial line to the LoRa radio. It successfully discards passive, high-frequency telemetry, preventing radio channel saturation, saving battery, and allowing local Wi-Fi scan threads to execute seamlessly without timeouts.

---

## 🛠️ Building & Flashing

RoboLora is managed via the **PlatformIO** ecosystem:

1.  **Configuration (`platformio.ini`)**: Targets the local microcontroller environment (e.g., `pico32`) and binds dependency libraries such as `Adafruit_SSD1306` and custom `RoboTone` (CRC).
2.  **Compilation**:
    ```powershell
    C:\Users\Peter.d.Nijs\.platformio\penv\Scripts\platformio.exe run
    ```
3.  **Upload**:
    ```powershell
    C:\Users\Peter.d.Nijs\.platformio\penv\Scripts\platformio.exe run -t upload
    ```\n