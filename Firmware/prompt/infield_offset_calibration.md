# In-Field Autonomous Offset Calibration (GPS vs. Compass)

## Objective
Implement an autonomous "In-Field Offset Calibration" routine to align the magnetic compass with the true GPS course. The buoy will sail a straight line using only its compass for navigation, then compare the actual path taken (measured by GPS) to the intended magnetic heading. The resulting angular difference is the `compassOffset`.

## 1. User Interface (RoboLog/RoboControl.py)
*   **New Button**: Add an "In-Field Offset" button next to the "In-Field Calib" button.
*   **Confirmation Dialog**: Prompt the user: *"Deploying In-Field Offset Calibration will cause the buoy to sail South for ~2 minutes. Ensure you have enough open water to the South. Continue?"*
*   **Command Transmission**: Send a specific command ID (e.g., `CMD = 71`) over UDP and/or LoRa.
*   **Status Feedback**: The buoy should broadcast status `71` while calibrating.

## 2. Communication Protocol
*   **New Command Constant**: Define `INFIELD_OFFSET_CALIBRATE = 71` in `RoboCompute.h`.
*   **Routing**: Ensure `RobobuoyTop` forwards `CMD 71` to `RobobuoySub`.

## 3. Sub Firmware State Machine (RobobuoySub)
*   **Trigger Handling**: When `CMD 71` is received, set buoy status to `71` and start the sequence.
*   **Calibration Sequence (Total ~130 seconds)**:
    *   **Phase 0 (0-10s) - Stabilization**: 
        *   Set `target_heading = 180.0` (Magnetic South).
        *   Engage thrusters at `50%` speed (using standard PID navigation logic, but locked to 180°).
        *   Wait for the buoy to stabilize its course.
    *   **Phase 1 (at 10s) - Start Point**:
        *   Record the current GPS coordinates (`Lat1`, `Lon1`).
    *   **Phase 2 (10s - 130s) - Sailing**:
        *   Continue sailing at `50%` speed, maintaining a magnetic heading of exactly `180.0°`.
        *   Crucially, **do not** use GPS for course corrections during this time; use only the compass.
    *   **Phase 3 (at 130s) - End Point**:
        *   Record the final GPS coordinates (`Lat2`, `Lon2`).
        *   Stop the thrusters (`bb = 0`, `sb = 0`).

## 4. Calculation & Storage
*   **Calculate GPS Course**:
    *   Use the `calculateAngle(Lat1, Lon1, Lat2, Lon2)` function from `RoboCompute` to determine the true heading traveled according to GPS.
*   **Compute Offset**:
    *   The difference between the intended magnetic course (`180.0°`) and the actual GPS course is the offset.
    *   `offset = GPS_Course - 180.0` (normalized to -180 to +180).
*   **Verification**:
    *   If the distance traveled is too short (e.g., < 10 meters), abort and log an error; the data is unreliable.
*   **Persistence**:
    *   Update `compassCalc.compassOffset += offset` (accumulative) or set it directly.
    *   Call `CompasOffset(&compassCalc, SET)` to store the new value in NVS.
*   **Completion**:
    *   Play a completion tone, log the new offset to Serial, and return status to `IDLE` (7).

## 5. Safety
*   **Abort**: Any `IDLE` or `LOCK` command from the user must instantly terminate the calibration and stop the motors.
*   **Fence**: If the buoy detects it is sailing into a known "danger zone" (if implemented) or if GPS signal is lost for > 5 seconds, abort.

## 6. Implementation Steps
1. Define `INFIELD_OFFSET_CALIBRATE` in `RoboCompute`.
2. Update `RobobuoyTop` to route the new command.
3. Implement the timer/phase logic in `RobobuoySub`.
4. Integrate the GUI button in `RoboControl.py`.