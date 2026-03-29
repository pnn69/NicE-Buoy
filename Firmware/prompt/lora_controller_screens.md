# LoraController OLED Screens Specification

This document defines the layout, behavior, and data requirements for the three primary display screens on the LoraController. The active screen is determined dynamically by the current `status` of the selected buoy.

---

## 1. IdleScreen
**Active Modes:** `IDLE`, `IDELING`

### Purpose
Provides detailed GPS telemetry when the buoy is standing by.

### Visual Layout (128x64)
*   **Header (Top):** 
    *   Buoy ID (e.g., `ID:b7a5b578`): **Size 1**
*   **Center/Main Focus:** 
    *   Magnetic Heading (`MAG: 120`): **Size 2**
    *   GPS Status (`SAT:12 FIX`): **Size 1**
    *   **Fixed Mode Indicator (`I`): Size 2 at x=96, y=0**
*   **Sub-Focus:**
    *   Latitude (e.g., `Lat: 52.123456`): **Size 1**
    *   Longitude (e.g., `Lon: 4.123456`): **Size 1**   
    *   Battery voltage bar: **Graphic (4px height)**
*   **Right Edge:** 
    *   Dual vertical thrust bars: **Graphic (8px width each)**
*   **Hidden Elements:** 
    *   Target direction/distance

---

## 2. LockScreen
**Active Modes:** `LOCKED`, `LOCKING`, `DOCKED`, `DOCKING`

### Purpose
Displays critical navigation and telemetry data while the buoy is autonomously holding a position or sailing to a dock.

### Visual Layout (128x64)
*   **Header (Top):** 
    *   Buoy ID: **Size 1**
*   **Center/Main Focus:** 
    *   Current Target Heading Current Magnetic Heading (`001: 120`): **Size 2**
    *   Distance to target (`D: 45.2m`) or (`D: 4.3Km`): **Size 2**
    *   **Fixed Mode Indicator (`L` or `D`): Size 2 at x=96, y=0**
*   **Sub-center (Navigation):**
    *   Battery voltage bar: **Graphic (4px height)**
*   **Right Edge:** 
    *   Dual vertical thrust bars: **Graphic (8px width each)**

---

## 3. RemoteScreen
**Active Modes:** `REMOTE`

### Purpose
Provides real-time feedback for manual control inputs (Potentiometers) and the buoy's actual response.

### Visual Layout (128x64)
*   **Header (Top):** 
    *   Buoy ID: **Size 1**
*   **Center/Main Focus:** 
    *   Current Target Heading Current Magnetic Heading (`001: 120`): **Size 2**
    *   Commanded Speed (`SPD: 85%`): **Size 2**
    *   **Fixed Mode Indicator (`R`): Size 2 at x=96, y=0**
*   **Sub-center:** 
    *   Battery voltage bar: **Graphic (4px height)**
*   **Right Edge:** 
    *   Dual vertical thrust bars: **Graphic (8px width each)**

---

## 4. Persistent UI Components (Pixel-Perfect Specs)

To maintain a consistent and exact visual identity across all screens, the Battery Bar and Thrust Bars must adhere to the following strict pixel coordinates, dimensions, and drawing behaviors.

### 4.1 Thrust Bars (Right Edge)
Two vertical bars representing the Bow (BB) and Stern (SB) motor output.
*   **Dimensions & Positioning:**
    *   `THRUST_BAR_WIDTH` = 8px
    *   `THRUST_BAR_GAP` = 0px
    *   **BB Bar (Left):** Drawn at `x = 112`, `y = 0`, `width = 8`, `height = 64`.
    *   **SB Bar (Right):** Drawn at `x = 120`, `y = 0`, `width = 8`, `height = 64`.
*   **Behavior (Center-Origin Fill):**
    *   The center (neutral) point for both bars is exactly `y = 32`.
    *   The maximum fill height (`maxBarH`) in either direction is `32px` (representing 100% effort).
    *   **Positive Effort (> 0%):** The inner bar fills *upwards* from the middle (`fillRect(x + 1, 32 - fillHeight, 6, fillHeight)`).
    *   **Negative Effort (< 0%):** The inner bar fills *downwards* from the middle (`fillRect(x + 1, 32, 6, fillHeight)`).

### 4.2 Battery Bar (Bottom Edge)
A horizontal bar mapping the buoy's sub-battery voltage level to a visual gauge.
*   **Dimensions & Positioning:**
    *   Drawn at `x = 0`, `y = 60`, `height = 4`.
    *   `width` = `112px` (touches the left side of the thrust bars).
*   **Behavior (Linear Mapping):**
    *   **Empty:** `19.0V` maps to a fill width of `0px`.
    *   **Full:** `25.2V` maps to a fill width of `112px`.
    *   The voltage value is constrained to this `minV`/`maxV` range before calculating the pixel width. The fill is drawn from left to right (`fillRect(0, 60, fillWidth, 4)`).

---

## Technical Implementation Notes
*(To be implemented in `oled_ssd1306.cpp`)*
*   Create an `enum ScreenType { SCREEN_IDLE, SCREEN_LOCK, SCREEN_REMOTE }`.
*   A main `updateOled(RoboStruct *buoy)` function will evaluate `buoy->status` and route to specific rendering functions:
    *   `drawIdleScreen(buoy)`
    *   `drawLockScreen(buoy)`
    *   `drawRemoteScreen(buoy)`
