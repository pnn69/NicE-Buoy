# RoboPythonDisplay — Multi-Buoy Desktop Control Dashboard

The **`RoboPythonDisplay`** sub-project is a desktop-based, graphical command-and-control dashboard written in Python utilizing the **Tkinter** user interface toolkit. It provides the pilot with live status updates, spatial mapping, and configuration portals for up to 3 active marine buoys.

---

## 🏗️ System Overview & Communications

RoboPythonDisplay acts as the centralized pilot terminal. It aggregates telemetry from multiple channels and sends downstream mission profiles:

```
┌────────────────────────────────────────────────────────────────────────┐
│                          RoboPythonDisplay                             │
├───────────────────────────────────┬────────────────────────────────────┤
│           UDP Listener            │           Serial Listener          │
├───────────────────────────────────┼────────────────────────────────────┤
│ Receives high-frequency local     │ Connects directly to the USB       │
│ Wi-Fi broadcast telemetry from   │ RoboLora Gateway to receive distant│
│ nearby buoys (Port 1001).         │ long-range RF telemetry packages.  │
└───────────────────────────────────┴────────────────────────────────────┘
```

---

## ⚡ Core Operational Features

### 1. Real-Time Telemetry Visualization
*   **Graphical Windroses & Compasses**: Displays live heading and target direction bearings dynamically.
*   **Thruster Indicators**: Provides vertical bar gauges showing active Port (BB) and Starboard (SB) motor duty cycles (percentage of thrust).
*   **Voltage Tracking Plots**: Monitors battery voltages to track capacity depletion trends in real-time.
*   **GPS Tracking**: Projects live latitude and longitude positions onto an interactive telemetry feed.

### 2. Mission Control Panel
*   **State Machine Overrides**: Allows the pilot to switch the buoy's operational mode at the click of a button between `IDLE`, `MANUAL`, `LOCKED` (GPS Anchor), and `DOCKING` (Waypoint Sailing).
*   **Target Coordinate Injection**: Transmits coordinates and target waypoint distances directly to the buoys' supervisory loops.

### 3. Live In-Field PID Tuning
*   **Sliders Interface**: Displays slider inputs for Proportional, Integral, and Derivative coefficients ($K_p$, $K_i$, $K_d$) for both the Rudder and Speed loops.
*   **OTA Gains Injection**: Transmits the custom coefficients over LoRa. The Sub Unit instantly adopts the new gains without needing physical re-flashing or system restarts.

### 4. Multithreaded GUI Stability
*   **Background Listeners**: Spawns non-blocking daemon threads to process incoming serial COM signals and Wi-Fi UDP sockets asynchronously.
*   **Thread-Safe UI Redrawing**: Implements Tkinter-safe message loops utilizing `master.after()` callbacks, preventing GUI freezes or thread collisions while processing high-volume telemetry.

---

## 🚀 Getting Started

### Prerequisites
Ensure you have Python 3.10+ installed along with necessary dependencies:
```bash
pip install pyserial
```

### Running the Dashboard
Connect your **`RoboLora`** USB gateway to an active COM port, navigate to the directory, and launch the application:
```bash
python RoboControl.py
```\n