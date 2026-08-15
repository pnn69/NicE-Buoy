# RoboCYD Controller Development & Interface Mandates

This file defines the foundational user interface design conventions, touch response behaviors, and architectural protections for the modular Cheap Yellow Display (RoboCYD) controller. Any future changes, features, or updates to this codebase MUST strictly adhere to these mandates.

---

## 🎨 Layout and Typography Standards

### 1. Compact Buoy List Buttons
* **Font Size:** Must always use standard high-legibility **font size `2`** for both the buoy name/ID and connection details.
* **Button Height:** Standardized to exactly **`40px`** with a tight **`5px`** vertical gap spacing, spanning **`Y: 75 to 205`**.
* This preserves maximum screen height on the `240x320` Portrait display while maintaining elite readability.

### 2. Large Blue `TRACK SETTINGS` Button
* **Placement:** Must sit cleanly and directly below Buoy 3, occupying **`Y: 210 to 245`** (`X: 10 to 230`).
* **Visual Style:** Must be drawn as a standard large, rounded menu button colored **BLUE (`TFT_BLUE`)** with white text, matching the width and shape of the buoy buttons.
* **Trigger Behavior:** Enters the on-screen Track Settings page on a standard single-touch tap/hold.

### 3. Centered `CALIBRATE TOUCH` Button
* **Placement:** Must sit centered at the bottom of the screen, occupying **`Y: 250 to 278`** (`X: 30 to 210`).
* **Visual Style:** Standard centered, rounded grey button (**`TFT_DARKGREY`**) with white text.

---

## 👆 Touch Sensitivity and Calibration Protections

### 1. Instant Single-Tap Calibration Button
* **Standard Behavior:** The `"CALIBRATE TOUCH"` button must behave exactly like all other menu buttons—triggering **instantly on a standard single tap (single click)**.
* **Accidental Protection:** Accidental calibration triggers are already fully protected by the live **3-second automatic discard timeout** and the green **SAVE** confirmation button inside the calibration screen itself. No complex holds or color changes should be applied on the Menu Screen button.
* **Generous Catchment Area:** Symmetrically expand the touchscreen coordinate checks for the calibration button to **`Y: 246 to 320`** and **`X: 10` to `230`** (using the entire bottom portion of the screen below the `TRACK SETTINGS` button as the active touch catchment area). This guarantees 100% responsive clicks on all drifted or shifted physical displays.

### 2. Calibration Entrance Startup Lockout Delay
* **The Constraint:** When entering touch calibration mode, there is a physical touch propagation leak where the user's finger is still pressed against the glass from the Menu button tap during the screen transition.
* **The Rule:** Symmetrically enforce a **touch lock-out delay of at least `800ms`** (using `cal_started_ms`) at the very beginning of `handle_touch_calibration()`. All raw touch sensor readings must be ignored during this period, allowing the user to naturally lift their finger off the screen before target tracking commences.

---

## ⚡ Redraw and Background Protection

### 1. Dynamic UI Overwrite Lock-out
* **The Constraint:** The background dynamic UI loop (`update_dynamic_ui()`) refreshes buoy online presence and IP address displays every `250ms`.
* **The Rule:** `update_dynamic_ui()` must always check `in_track_settings_mode` and **immediately return if true**. This prevents the background refresh cycle from overwriting the active, visible Track Settings Screen and controls.
