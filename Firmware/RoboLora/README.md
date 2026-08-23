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
*   **Priority list**: `NicE_WiFi` (password `!Ni1001100110`) then `Robo_WiFi` (password
    `geenanker`) then its own **`LORA_<id>`** AP (password `geenanker`, `192.168.4.1`). No scanning:
    the SSIDs are tried in order with a blind `WiFi.begin()`.
*   **RoboLora is a client of the field network, not its host.** It used to raise the field AP
    itself under the same name the CYD uses; two devices broadcasting one SSID on one subnet is two
    separate networks wearing the same badge, and the fleet would split between them. **The CYD is
    the field AP** - see [RoboCYD](../RoboCYD/README.md).
*   **The fallback AP keeps hunting.** It runs in `WIFI_AP_STA` and keeps looking for a real network
    underneath, but will not tear itself down while a client is connected.
*   **mDNS**: reachable as `lora-<id>.local` in both modes. Only `ArduinoOTA.begin()` may initialise
    the responder.
*   **Captive Portal & DNS Redirection**: a `DNSServer` on UDP port 53 resolves every lookup to our
    own AP address while we are the access point, so a phone's connectivity probe lands on the
    dashboard and the "sign in to network" sheet opens it automatically. The 302 for unknown URLs is
    aimed at `WiFi.softAPIP()` - it used to be hardcoded to `192.168.1.84`, which was the Top's old
    AP address rather than ours - and it is **disabled when we are a station**, where a 404 must stay
    a 404 or a mistyped fetch silently returns the whole dashboard.
*   **Sender identity**: every frame RoboLora puts on UDP is stamped `IDs = 0x99`, the web-authority
    ID. Buoys act on broadcast commands only from `0x99` (web/RoboLora) and `0x98` (the CYD), so this
    stamp is what makes operator commands authoritative - and why it must not be applied to traffic
    merely being relayed.
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

Environment **`pico32`**. Libraries include `RoboCompute`, `Adafruit_SSD1306` and `RoboTone`.

```bash
pio run                  # build
pio run -t upload        # serial upload, upload_port = COM30
```

**There is no espota configuration**, and `COM30` may not exist on the machine you are using. The
gateway does run ArduinoOTA, so flash it over the network directly:

```bash
python ~/.platformio/packages/framework-arduinoespressif32/tools/espota.py \
       -i <ip> -p 3232 -f .pio/build/pico32/firmware.bin -r
```\n