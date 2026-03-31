# RobobuoySub - Main Development Prompt

## Role & Goal
You are building the firmware for `RobobuoySub`, the core propulsion and navigation logic of an autonomous robotic racing buoy. It runs on an ESP32.
Your goal is to recreate this project exactly as it exists today, serving as the "boat" that receives commands from `RobobuoyTop` and drives two thrusters to hold station or navigate to waypoints.

## Hardware & Architecture
* **Microcontroller:** ESP32 (esp32dev)
* **Sensors:** I2C Compass (LIS2MDL / LSM303), I2C ADC.
* **Actuators:** Two ESCs (Electronic Speed Controllers) for Port (BB) and Starboard (SB) thrusters.
* **Libraries:** `PID_v1` (br3ttb), `ESP32Servo`, `Adafruit_LIS2MDL`, `RoboCompute`, `ArduinoOTA`.
* **RTOS Architecture:** FreeRTOS Tasks and Queues (`compassIn`, `serIn`, `serOut`, `motorCmd`). 
* **Hardware Interfaces:**
  * `Serial1`: Receives encoded commands from `RobobuoyTop`.
  * `I2C`: Interfaces with sensors.
  * `PWM`: Drives the ESCs using Servo signals.

## Core Responsibilities & Logic
1. **Control Loops (PID):**
   * **Rudder PID:** Computes the turning effort (`Kpr`, `Kir`, `Kdr`) to point the bow towards the target angle (`tgDir`).
   * **Speed PID:** Computes the forward thrust (`Kps`, `Kis`, `Kds`) to reach the target distance (`tgDist`).
   * **Hover PID (Station Keeping):** Uses an Adaptive "Stiffener" logic. It uses the I-term (Integral) as a proxy for wind speed. If the I-term increases (fighting wind), it dynamically multiplies the P-gain (Proportional) by up to 2.0x to tighten response in rough weather while remaining gentle in calm weather. Limits thrust to `maxSpeed`.
2. **Fleet Geometry & Start Lines:**
   * Calculates dynamic waypoints for up to 3 buoys using `recalcStartLine()` and `reCalcTrack()` (from `RoboCompute`).
   * Evaluates wind angle to determine which buoy is the HEAD, PORT, or STARBOARD mark.
3. **In-Field Calibration:**
   * Supports `CMD 70` (Compass Calibration: spins the boat in circles to map Hard/Soft iron offsets).
   * Supports `CMD 71` (Compass Offset Calibration: sails a straight line by GPS to determine the offset between physical heading and magnetic heading).
4. **Telemetry & Parameters:**
   * Receives `SETUPDATA` (CMD 83) requests. Must reply with its full configuration: Rudder PID, Speed PID, `maxSpeed`, `minSpeed`, `pivotSpeed`, `compassOffset`, and `minOfsetDist`.
   * Ensures `minOfsetDist` and `gpsDir` are correctly formatted as standard base-10 strings (without base arguments like `String(val, 0)`) before encoding.

## Constraints
* Ensure the ESCs have a proper initialization sequence (arming).
* `RoboCompute` handles NMEA packet encoding (`$IDr,IDs,ACK,CMD,STATUS,val1,val2...*CRC`). Zero-compression is strictly enforced.
* OTA hostname must be `Sub_<MAC_ADDRESS>`.