# Robobuoy Firmware Refactor Summary

This document summarizes the changes made to the Robobuoy firmware ecosystem to improve stability, communication integrity, and memory safety.

## 1. Architectural Changes: Centralized Dependency
- **Direct Source Compilation**: Updated `platformio.ini` for all projects (`Top`, `Sub`, `Base`, `Remote`, `LoraController`) to use `lib_extra_dirs` pointing to `../RobobuoyDependency`.
- **Benefit**: This ensures that all projects compile against the master version of `RoboCompute` and `RoboTone` directly. Changes made to the central library are immediately reflected across all projects without manual copying.

## 2. Master Library Improvements (`RoboCompute`)
- **Pointer-Based API**: Refactored major functions (`PidDecode`, `PidEncode`, `hooverPid`, `CalcRemoteRudderBuoy`) to use `RoboStruct*` pointers.
  - **Why**: Prevented expensive memory copying of the large `RoboStruct` and ensured that parameter updates (like PID tuning) persist across function calls.
- **Adaptive Station Keeping (I-Driven Stiffening)**: Implemented a wind-proxy logic in `hooverPid`.
  - **Logic**: The loop uses the Integral (I) term to estimate wind force. If the I-term is high (strong wind), the Proportional (P) gain is automatically doubled to "stiffen" the response.
- **3-Buoy Start Line Logic**: Updated `recalcStartLine` to handle complex layouts.
  - **Feature**: Automatically identifies the "HEAD" (upwind) buoy and repositions the remaining two "DOWNWIND" buoys symmetrically around their midpoint, perpendicular to the wind.
- **Telemetry Decoding Fixes**: Resolved critical bugs in `RoboDecode`:
  - Fixed `WINDDATA` and `TOPDATA` where array indices were duplicated, causing telemetry data to be overwritten or lost.
  - Fixed `PID` decoding to correctly assign P, I, and D terms to their respective variables.
- **Safety**: Fixed an out-of-bounds array access in `calcTrackPos`.

## 3. Communication Integrity
- **UDP Double CRC Fix**: Removed a redundant CRC encoding step in `topwifi.cpp`.
  - **Problem**: Packets were being encoded twice (`$$data*CRC*CRC`), making them unreadable.
- **Serial Reliability**: Fixed a bug in `RobobuoySub/sercom.cpp` where `random(1200, 750)` was called. 
  - **Fix**: Swapped to `random(750, 1200)` as the Arduino `random()` function requires `min < max`. This ensures proper retransmission timing.

## 4. GPS System Overhaul (`RobobuoyTop`)
- **Thread Safety**: Refactored `RouteToPoint` to use standalone mathematical functions instead of the shared `TinyGPSPlus` object.
  - **Why**: Prevented race conditions between the asynchronous GPS task and the main logic loop.
- **Outlier Recovery**: Improved `handelGpsData` in `main.cpp`.
  - **Feature**: Added a "persistent outlier" check. If the GPS receives 30 consecutive "jumps" >50m, it now accepts the new position. This allows the buoy to recover if it is physically moved while powered on.

## 5. Task Performance & Stability
- **Serial Robustness**: 
  - Implemented `Serial.readStringUntil('\n')` with a 100ms timeout in both projects.
  - **Benefit**: Prevents tasks from stalling if a character is missed or a packet is malformed.
- **Non-Blocking State Machines**: Refactored `buzzerTask` and `CompassTask`.
  - **Fix**: Replaced long `vTaskDelay` and hardware busy-waits with state-machine logic.
  - **Benefit**: The system remains responsive to high-priority commands (like E-Stops or new beep requests) even during long operations.
- **Queue Optimization**: Switched to `xQueueOverwrite` for high-frequency telemetry (Compass, GPS).
  - **Benefit**: Ensures the main loop always processes the freshest data and prevents queue-full blocking.
- **LED Efficiency**: Added a 10ms timeout to `LedTask` queue reads to prevent CPU "spinning" (busy-waiting) when idle.

## 6. Hardware & Logic Safety
- **ESC Jitter Removal**: Corrected the ramping logic in `esc.cpp` that caused a ±1 pulse-width oscillation when the motors were supposed to be stationary.
- **Type Safety**: Fixed a critical struct mismatch in the `Sub` loop where `PwrData` was being sent to a queue expecting `LedData`.
- **I2C Reliability**: Added explicit `Wire.begin()` calls and verified sensor initialization to prevent bus hangs.

## 7. Memory Management
- **Stack Safety**: Increased `buzzerTask` stack size from 1000 to 2048 bytes.
- **Auto-Migration**: Added logic to `initMemory` to automatically detect and fix the legacy `Doclat` typo, moving existing coordinates to the corrected `Docklat` key.
- **Hardware Binding**: Both projects now use the hardware MAC address to verify memory ownership, ensuring defaults are re-applied if the processor is swapped.
- **Signature Sync**: Synchronized all `.h` and `.cpp` files to ensure function signatures match, resolving multiple linker and compiler errors.

## 8. Future Feature Plan: Dynamic Compass Offset Calibration
... (existing content) ...

## 9. Proposal: Optimized Navigation Strategy for Dual-Thruster Buoys
Given the dual-thruster configuration (differential thrust) with forward/reverse capability, the following strategy is proposed for transitioning to a new waypoint (`tgLat`, `tgLng`).

### Objective:
Minimize "hunting" (oscillating around the course) and ensure the buoy arrives at the target coordinates with the correct heading, regardless of its starting orientation.

### Recommended Implementation: Three-Phase Steering logic

#### Phase 1: Zero-Radius Rotation (The "Pivot")
*   **Trigger**: Angle error to waypoint > 30°.
*   **Action**: Use the thrusters in **opposing directions** (e.g., Left Forward, Right Reverse) to rotate the buoy on its center axis.
*   **Advantage**: This is the most energy-efficient way to correct a large heading error. It prevents the buoy from sailing a large, inefficient arc while trying to turn.

#### Phase 2: Differential Forward Translation (The "Transit")
*   **Trigger**: Angle error < 30° and distance > `minOfsetDist`.
*   **Action**: Both thrusters run forward with a **speed bias** based on the PID output.
    *   `BaseSpeed` = Calculated by Speed PID.
    *   `LeftThruster` = BaseSpeed - RudderPID_Output.
    *   `RightThruster` = BaseSpeed + RudderPID_Output.
*   **Advantage**: Maintains forward momentum while making fine-grained course corrections.

#### Phase 3: High-Precision Station Keeping (The "Hover")
*   **Trigger**: Distance < `minOfsetDist`.
*   **Action**: Switch to a sensitive "Pulse" mode or use very low duty cycles.
*   **Refinement**: If the buoy overshoots, it should use **Reverse Thrust** immediately rather than trying to turn 180 degrees. The dual-thruster setup allows for "backing up" which is much more stable for maintaining a precise coordinate.

### Safety & Efficiency Considerations:
1.  **Thrust Asymmetry Compensation**: Implement a small scaling factor if one motor is slightly more powerful than the other.
2.  **Wind-Vane Mode**: In high winds, the buoy should prioritize keeping its "nose" (the most aerodynamic/hydrodynamic profile) into the wind, even if it means sailing slightly "sideways" to the waypoint.
3.  **Deadband**: Implement a 2-3 degree "deadband" where no steering correction is made to prevent motor jitter and save battery.

---
**Status**: All projects are now compiling correctly and are functionally robust.

