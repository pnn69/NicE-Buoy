# RoboLog/RoboControl - Main Development Prompt

## Role & Goal
You are building `RoboControl.py`, the main Python GUI control application for the autonomous racing buoy fleet.
Your goal is to recreate this application exactly as it exists today. It sits on the user's PC, connects to the `RobobuoyTop` and `LoraController` devices, visualizes real-time telemetry from up to 3 buoys simultaneously, and allows the user to send commands and modify PID/setup configurations.

## Architecture & Technologies
* **Language:** Python 3
* **GUI Framework:** `tkinter` and `ttk`.
* **Networking:** 
  * `socket`: Listens for UDP broadcasts on port `1001` (from `RobobuoyTop`) and sends commands via UDP.
  * `pyserial`: Connects to `LoraController` via USB COM port to send/receive over LoRa.
* **Concurrency:** `threading` module to run non-blocking `udp_listener` and `serial_reader` loops alongside the Tkinter `mainloop()`.

## Core Features & Logic
1. **Dynamic Layout (3 Columns):**
   * The UI displays a 3-column layout. Each column dynamically binds to a Buoy ID as packets arrive (`$IDr,IDs...`).
   * **Visual Indicators:**
     * A `Canvas` Windrose showing 4 color-coded directional vectors (Target/Red, Magnetic/Green, Wind/Blue, GPS/Black).
     * Two vertical power bars representing Port (BB) and Starboard (SB) ESC thrust percentages.
     * A horizontal progress bar representing Sub Battery voltage (17V to 25V).
   * **Status & Commands:**
     * Network status indicators for UDP (OK/--) and LoRa (OK/--), plus a sync status if they mismatch.
     * Buttons: `LOCK`, `DOCK`, `SETUP`.
     * Inputs: `Dir` and `Dist` with a `Send` button (CMD 47). A `Map` button to open OpenStreetMap in the browser based on GPS coords.
2. **Network Fallback Logic:**
   * Sending commands defaults to UDP. If the UDP connection goes silent for > 5 seconds, the application automatically falls back to routing the command out through the LoRa Serial port.
3. **The Setup Window (CMD 83):**
   * When `SETUP` is clicked, a "Loading..." window appears while the app fires `CMD 83` (`SETUPDATA`) repeatedly.
   * *Anti-Echo Logic:* The `check_data_and_open` loop must verify that the incoming data contains actual values (not just empty strings `""` or compressed zeros `"0"`) before determining that the setup data has genuinely been received from the Sub buoy.
   * Once valid data is fetched, a scrollable Toplevel window opens containing Entry fields for: Rudder PID (`Kpr`, `Kir`, `Kdr`), Speed PID (`Kps`, `Kis`, `Kds`), Limits (`maxSpeed`, `minSpeed`, `pivotSpeed`), and `compassOffset`.
   * Includes two In-Field Calibration buttons: Compass Calibration (CMD 70) and Offset Calibration (CMD 71), which trigger confirmation `messagebox` dialogs before firing.
4. **NMEA Parsing & Zero Compression:**
   * Parse sentences format: `$IDr,IDs,ACK,CMD,STATUS,val1,val2...*CRC`.
   * Check the CRC before processing.
   * *Crucial:* The firmware uses zero-compression (e.g. `,,0,,`). The Python parser MUST normalize empty fields back into `"0"` strings immediately upon splitting the CSV list, otherwise indexing errors and missing UI updates will occur.

## Constraints
* Ensure the UI remains responsive. All GUI updates (e.g., `log_message` writing to the log text widget) must be routed to the main thread via `master.after(0, ...)`.
* Assume a maximum of 3 buoys connected concurrently.