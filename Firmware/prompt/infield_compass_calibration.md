# In-Field Autonomous Compass Calibration

## Objective
Implement an autonomous "In-Field Calibration" routine that runs while the buoy is deployed in the water. The user will trigger this procedure from the Python control script. The buoy will perform a sequence of maneuvers (circles of varying radii) to collect comprehensive magnetometer data, calculate hard-iron (and optionally soft-iron) calibration factors, store them in NVS, and return to an idle state.

## 1. User Interface (RoboLog/RoboControl.py)
*   **New Button**: Add an "In-Field Calib" button below or next to the existing "SETUP" button for each buoy.
*   **Confirmation Dialog**: Since this makes the buoy move autonomously, prompt the user: *"Deploying In-Field Calibration will cause the buoy to move in circles for ~60 seconds. Ensure the area is clear. Continue?"*
*   **Command Transmission**: Send a specific command ID (e.g., `CMD = 70`) over UDP and/or LoRa to the target buoy.
*   **Status Feedback**: Update the `Status` label in the GUI to `"CALIBRATING"` while the command is running (the buoy should broadcast its status as 70 during this time).

## 2. Communication Protocol (RoboCompute.h & RoboCompute.cpp)
*   **New Command Constant**: Define `INFIELD_CALIBRATE = 70` in `RoboCompute.h` alongside existing commands like `PIDRUDDER` or `MAXMINPWR`.
*   **Message Routing**: Ensure `RobobuoyTop` correctly routes `CMD 70` to `RobobuoySub` via the serial link.

## 3. Sub Firmware State Machine (RobobuoySub)
*   **Trigger Handling (`main.cpp`)**: When `ser->cmd == INFIELD_CALIBRATE` is received, change the buoy's internal status to `CALIBRATING` and trigger the calibration sequence.
*   **Non-Blocking Sequence (`compass.cpp` or `esc.cpp`)**: Implement a timer-based state machine that prevents the `loop()` from blocking while the buoy moves.
*   **Proposed Maneuver Sequence (Total ~60 seconds)**:
    *   **Phase 1 (0-15s) - Wide Right Turn**: Set `bb = 20%`, `sb = 5%`.
    *   **Phase 2 (15-30s) - Tight Right Turn**: Set `bb = 25%`, `sb = -5%`.
    *   **Phase 3 (30-45s) - Wide Left Turn**: Set `bb = 5%`, `sb = 20%`.
    *   **Phase 4 (45-60s) - Tight Left Turn**: Set `bb = -5%`, `sb = 25%`.
    *   *Note: These values should scale with `rud->maxSpeed` to ensure they are safe but effective.*
*   **Data Collection**:
    *   Continuously sample the raw magnetometer values (`event.magnetic.x`, `y`, `z`).
    *   Track the absolute minimums and maximums (`min_mag[3]`, `max_mag[3]`).
    *   *(Optional Enhancement)*: Stream this raw data back to the Python script (using the `RAWCOMPASSDATA` command) so the user can see a live scatter plot.

## 4. Calculation & Storage
*   **End of Sequence**: Once the 60 seconds expire, stop the thrusters (`bb = 0`, `sb = 0`).
*   **Hard-Iron Calculation**: 
    ```cpp
    compassCalc.magHard[0] = (max_mag[0] + min_mag[0]) / 2.0f;
    compassCalc.magHard[1] = (max_mag[1] + min_mag[1]) / 2.0f;
    compassCalc.magHard[2] = (max_mag[2] + min_mag[2]) / 2.0f;
    ```
*   **Soft-Iron Calculation (Simple Approach)**: Reset the soft-iron matrix to the identity matrix (`1.0` on the diagonal, `0.0` elsewhere) unless a more advanced least-squares fitting algorithm is implemented.
*   **Storage**: Call the existing `hardIron(&compassCalc, SET)` and `softIron(&compassCalc, SET)` functions to persist the data to the ESP32 Preferences (NVS).
*   **Feedback**: Play a success tone on the buzzer and return the buoy status to `IDLE` (7).

## 5. Safety & Edge Cases
*   **Abort Mechanism**: If the user sends an `IDLE` (8) or `LOCK` (12) command while the calibration is running, the sequence must immediately abort, stop the motors, and discard the collected data.
*   **Communication Loss**: If the buoy loses communication with the controller, it should ideally continue and finish the 60-second calibration, then stop. However, if the watchdog timer is strictly enforced, ensure the calibration routine feeds the watchdog or handles timeouts gracefully.
*   **Sensible Limits**: Reject calibration if the `max - min` range is suspiciously small (e.g., `< 10 µT`), which might indicate a broken sensor rather than a valid calibration circle.

## 6. Implementation Steps for the Developer (LLM)
1. Add the `INFIELD_CALIBRATE` command to `RobobuoyDependency/RoboCompute`.
2. Update `RobobuoyTop/src/main.cpp` to forward the command.
3. In `RobobuoySub/src/compass.cpp`, write the `InFieldCalibrationTask` or state machine.
4. Update `RoboLog/RoboControl.py` with the UI button and confirmation logic.