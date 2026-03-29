# LoraController Functional Specification

## Objective
The LoraController serves as the primary manual interface for the NicE-Buoy system. This document defines the logic for translating physical inputs (2 Potentiometers, 1 Three-way Switch, 1 Multi-function Button) into system commands sent via LoRa/UDP to the buoys.

## 1. Hardware Mapping
*   **POT_RUDDER (Analog)**: Controls the target heading.
*   **POT_SPEED (Analog)**: Controls the target speed (Forward/Reverse).
*   **SWITCH_PIN (Analog)**: 3-way toggle switch for primary operational modes.
*   **SW_P_1 (Digital)**: Multi-function button for state toggles and calibration.
*   **OLED (I2C)**: Real-time telemetry and mode feedback.
*   **Buzzer (PWM)**: Auditory confirmation of commands.

## 2. Potentiometer Logic (Filtering & Mapping)
To ensure smooth control, all analog inputs use an Exponential Moving Average (EMA) filter with a smoothing factor ($\alpha$) of 0.2.

### 2.1 Rudder Potentiometer (`POT_RUDDER`)
*   **Range**: 0 to 4095 (12-bit ADC).
*   **Mapping**: Maps linearly to 0–360° (Heading).
*   **Direction**: Clockwise rotation increases the heading value.
*   **Deadzone**: None (absolute positioning).

### 2.2 Speed Potentiometer (`POT_SPEED`)
*   **Range**: 0 to 4095.
*   **Center Point**: ~2048.
*   **Deadzone**: $\pm 700$ units from center (1348 to 2748).
*   **Mapping**:
    *   `0 to 1348`: Maps to `-100 to 0` (Reverse).
    *   `1348 to 2748`: Fixed at `0` (Neutral).
    *   `2748 to 4095`: Maps to `0 to 100` (Forward).
*   **Jitter Filter**: Only broadcast changes if the raw value shifts by $> 20$ units.

## 3. Mode Switch Logic (`SWITCH_PIN`)
The 3-way switch defines the buoy's high-level state. To prevent accidental state overrides during buoy navigation (switching between buoys), the controller uses **Edge-Triggered Command Logic**.

### 3.1 Edge-Triggered Behavior
*   **Navigation Safety**: When a new buoy is selected via the button (1 click), the controller **latches** the current physical position of the switch without sending a command.
*   **Command Trigger**: A command (`IDLE`, `REMOTE`, or `LOCKED`) is **only** dispatched to the active buoy if the physical switch is moved to a *different* position than it was in at the moment of selection.
*   **Interpretation**: If Buoy A is `IDLE` and the switch is at `Middle`, and you switch to Buoy B (which is `REMOTE`), Buoy B will *remain* `REMOTE` until you physically move the switch to a new position.

| Position | Mode | Logical Action |
| :--- | :--- | :--- |
| **Left** | `REMOTE` | Buoy follows Potentiometer inputs (`DIRSPEED` command). |
| **Middle** | `IDLE` | Buoy stops thrusters and clears target buffers (`IDLE` command). |
| **Right** | `LOCKED` | Buoy holds current GPS coordinates (`LOCKED` command). |

## 4. Button Sequences (`SW_P_1`)
The button uses a sequence-based accumulator to trigger complex actions without a GUI.

### 4.1 Short Press Sequences (Timeout: 500ms)
*   **1 Click**: **Cycle Active Buoy**. Switches the OLED telemetry and control focus to the next buoy in the local array. *Note: Latches the current switch position to prevent immediate mode change.*
*   **5 Clicks**: `DOCK` (Sail to pre-stored dock position if not in REMOTE).

### 4.2 Long Press Sequences (Hold: > 3000ms)
*   **Long Press (0 clicks beforehand)**: `STORE_AS_DOCK` (Save current GPS as Dock).
*   **1 Click + Long Press (in sequence)**: `ALIGN_STARTLINE` / `COMPUTESTART` (Calculate and align start line). *Note: Only active when the buoy is in LOCKED mode.*
*   **4 Clicks + Long Press (in sequence)**: `STORE_AS_DOCK` (Save current GPS as Dock).

## 5. Visual & Auditory Feedback

### 5.1 OLED Display Layout
The 128x64 SSD1306 OLED provides real-time telemetry from the selected buoy.

| Feature | Position | Description |
| :--- | :--- | :--- |
| **Status Row** | Top (y=0) | ID, Mode Icon, and Actual Mag Heading (hidden in IDLE). |
| **Target Info** | Mid-Top (y=12)| IDLE: `MAG:xxx` / Other: `TG:xxx` (Large Font). |
| **GPS Status**  | Mid-Bot (y=30)| `SAT *:xx` (Fix) or `SAT -:xx` (No Fix) (Large Font). |
| **Navigation** | Bot-Mid (y=50)| If Locked/Docked: Displays `Dist` and `Target Dir`. |
| **Battery Bar** | Bottom (y=60) | Horizontal bar mapped from 19.0V to 25.2V. |
| **Thrust Bars** | Far Right | Two vertical bars representing relative BB and SB power levels. |

### 5.2 Auditory Feedback (Buzzer)
*   **Short Click**: High-pitch beep on every detected button press.
*   **Mode Change**: Rising melody on successful transition.
*   **Idle/Abort**: Falling melody on `IDLE` or sequence cancel.

## 6. Implementation Strategy
1.  **ADC Layer (`adc.cpp`)**: Maintain EMA filters and provide "cleaned" values to the main loop.
2.  **Input Handler (`main.cpp`)**: A state-machine-based button counter that resets on a 500ms idle timer.
3.  **Command Dispatcher**: 
    *   Maintain a `lastPhysicalSwitchPos` variable.
    *   Update `lastPhysicalSwitchPos` silently whenever the active buoy ID changes.
    *   Compare current switch/pot state vs. `lastPhysicalSwitchPos`; dispatch `LoRa` packet only on physical movement.
    *   Optional: Send a "Keep-Alive" heartbeat every 5s containing the *last confirmed* command, NOT the physical switch state.
