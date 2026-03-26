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
*   **Calibration Sequence (Total ~300+ seconds)**:
    *   **Phase 0 (0-10s) - Initialization & Home**:
        *   **RECORD HOME (P0)**: Record current `Lat0, Lon0`. This is the point we want to return to.
        *   Set `target_heading = 180.0` (Magnetic South).
        *   Engage thrusters at `50%` speed to stabilize.
    *   **Phase 1 (at 10s) - Record Start Point (P1)**:
        *   Record `Lat1, Lon1`.
    *   **Phase 2 (10s - 130s) - Calibration Leg**:
        *   Sail at `50%` speed maintaining magnetic `180.0°`.
        *   Navigation: **Compass only**.
    *   **Phase 3 (at 130s) - Record Calibration End Point (P2)**:
        *   Record `Lat2, Lon2`.
        *   **IMMEDIATE CALCULATION**: Calculate the GPS course from P1 to P2. Calculate the error vs 180.0°.
        *   **APPLY & STORE**: Update `compassCalc.compassOffset` and call `CompasOffset(&compassCalc, SET)`.
        *   **RE-INITIALIZE**: Call `InitCompass()` to ensure the new offset is active.
    *   **Phase 4 (130s - 250s) - Return Leg (Verification)**:
        *   Set `target_heading = 0.0` (Magnetic North) to return towards the starting point.
        *   Use the **newly calibrated `compassOffset`**.
        *   Sail for 120 seconds using only the compass.
    *   **Phase 5 (at 250s) - Record Final Point (P3)**:
        *   Record `Lat3, Lon3`.
        *   **VALIDATE**: Calculate the GPS track from P2 to P3. 
    *   **Phase 6 (Final) - Return Home**:
        *   If the validation error is within a reasonable range (informative), the buoy should now engage its standard navigation logic.
        *   Set `targetLat = Lat0, targetLon = Lon0` (The Home position).
        *   Navigate back to the starting point using the fully calibrated GPS + Compass system.
        *   Once within range (e.g., 2m), stop thrusters and return to `IDLE` (7).

## 4. Calculation & Validation
*   **Protocol Note**: Since the buoy moves too slowly for reliable real-time GPS Course-Over-Ground (COG), all validation must be done using point-to-point coordinate math.
*   **Leg 1 Analysis (Calibration)**:
    *   `GPS_Course_1 = calculateAngle(Lat1, Lon1, Lat2, Lon2)`.
    *   `New_Offset = GPS_Course_1 - 180.0`. This aligns the 180° Magnetic heading with the actual GPS path taken.
*   **Leg 2 Analysis (Verification)**:
    *   `GPS_Course_2 = calculateAngle(Lat2, Lon2, Lat3, Lon3)`.
    *   `Validation_Error = abs(GPS_Course_2 - 0.0)` (normalized). 
*   **Leg 3 (Navigation)**: 
    *   The final leg back to P0 serves as the real-world test of the autonomous station-keeping capability with the fresh calibration.
*   **Completion**:
    *   Log `GPS_Course_1`, `GPS_Course_2`, and `New_Offset` to Serial. Play a completion tone once the buoy is back at P0.

## 5. Safety
*   **Abort**: Any `IDLE` or `LOCK` command from the user must instantly terminate the calibration and stop the motors.
*   **Fence**: If the buoy detects it is sailing into a known "danger zone" (if implemented) or if GPS signal is lost for > 5 seconds, abort.

## 6. Implementation Steps
1. Define `INFIELD_OFFSET_CALIBRATE` in `RoboCompute`.
2. Update `RobobuoyTop` to route the new command.
3. Implement the timer/phase logic in `RobobuoySub`.
4. Integrate the GUI button in `RoboControl.py`.