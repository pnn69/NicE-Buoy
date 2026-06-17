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

### 4. Standalone Web Config Mode (`controlwifi.cpp` / `controlwifi.h`)
*   **Config Access Point**: Spawns a lightweight local Wi-Fi Hotspot on-demand, serving a simple HTML landing page.
*   **Diagnostics Portal**: Allows developers to adjust RF channels, monitor raw packets, calibrate battery measurement offsets, and run diagnostics without needing physical cables or specialized debuggers.

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