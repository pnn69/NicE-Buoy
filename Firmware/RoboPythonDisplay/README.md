# RoboPythonDisplay (PC Control Dashboard)

A Python-based graphical user interface (GUI) for monitoring and configuring the NicE-Buoy fleet from a base station PC.

## Key Features
- **Tkinter Dashboard:** Real-time visual dashboard monitoring up to 3 buoys simultaneously.
- **Dual Telemetry Links:** Listens to telemetry via UDP (local WiFi network) and provides a LoRa fallback (via serial to a connected LoRa receiver).
- **Visual Indicators:** Features a dynamic "Windrose" canvas showing magnetic heading, target heading, wind direction, and GPS course.
- **PID & Setup Configuration:** Dedicated setup windows to push new PID coefficients, Speed Limits, and Compass Offsets to the buoys over the air.
- **In-Field Calibration:** Triggers autonomous compass and offset calibration routines remotely.