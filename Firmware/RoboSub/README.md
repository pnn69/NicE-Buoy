# RoboSub (Underwater Unit Firmware)

The `RoboSub` firmware runs on the submerged ESP32 unit of the NicE-Buoy system. It is responsible for the physical locomotion and orientation of the buoy.

## Key Features
- **Differential Thrust Control:** Drives two ESCs (Electronic Speed Controllers) for the port (BB) and starboard (SB) thrusters.
- **PID Navigation:** Implements independent PID loops for Speed and Rudder (heading) to autonomously hold position (LOCKED) or navigate to a waypoint (DOCKING).
- **Magnetometer (Compass):** Interfaces with an I2C compass (e.g., LIS2MDL/LSM303) for precise heading data, including hard/soft iron calibration and tilt compensation.
- **Power Monitoring:** Reads battery voltage via ADC and reports power telemetry.
- **Serial Tether:** Receives movement targets (Target Distance/Direction) and PID tuning parameters from the `RoboTop` via a robust serial connection.