# Static Compass Calibration Integration Prompt

## Table of Contents

1. Overview
2. Project Constraints
3. Goal
4. User Workflow
5. Calibration Concept
6. Calibration Positions
7. Calibration Screen Layout
8. Heading Adjustment Behavior
9. Store Position Logic
10. Progress Display
11. Save Button Behavior
12. Protocol Changes
13. ESP32 Firmware Additions
14. NVM Storage Requirements
15. Output Requirements
16. Response Requirements

---

# 1. Overview

I have an existing Robobuoy dashboard consisting of:

- index.html
- style.css
- index.js

Do NOT create a new project.

Do NOT create simplified examples.

Do NOT give pseudocode.

Modify my existing code directly and provide complete copy/paste-ready code blocks for every changed file.

---

# 2. Project Constraints

Use the existing architecture.

Reuse existing systems whenever possible.

Do not redesign the dashboard.

Do not introduce a second compass correction system.

Do not introduce a second steering controller.

Keep the implementation consistent with the existing protocol and UI.

---

# 3. Goal

Add a new feature called:

```text
Static Compass Calibration
```

This feature must become an alternative way of generating the existing 8-point Fourier compass calibration table.

The generated calibration table must be compatible with the existing harmonic/Fourier correction system.

Do NOT create a separate correction system.

Both:

```text
GPS Fourier Calibration
```

and

```text
Static Compass Calibration
```

must produce the same final 8-point correction table.

---

# 4. User Workflow

Add a new button to the Setup dialog:

```text
Static Compass Calibration
```

When pressed a new modal opens.

The modal must not replace the existing Setup dialog.

It is a completely separate modal.

---

# 5. Calibration Concept

The buoy already has:

- Manual Navigation
- Rudder PID
- Pivot speed control
- Heading control

Reuse these systems.

Do NOT create a new steering controller.

The calibration modal shall reuse the existing REMOTE steering system.

The buoy must rotate in place.

Speed is always:

```text
0
```

Only heading is controlled.

The rudder PID and pivot logic already present in the buoy must rotate the buoy toward the selected heading.

---

# 6. Calibration Positions

The calibration consists of exactly 8 points:

```text
N   = 0°
NE  = 45°
E   = 90°
SE  = 135°
S   = 180°
SW  = 225°
W   = 270°
NW  = 315°
```

---

# 7. Calibration Screen Layout

Show:

```text
Position: N

Target Heading: 0°

Adjustment: 0°
```

Buttons:

```text
-1°
+1°
```

Every time + or - is pressed:

- Adjustment changes
- New heading is sent to the buoy
- Existing REMOTE steering command is reused
- Speed remains zero

Example:

```text
Target = 0°
Adjustment = +3°
```

Actual heading sent:

```text
3°
```

---

# 8. Heading Adjustment Behavior

Start at:

```text
North
0°
```

When North is completed:

Automatically advance to:

```text
NE
45°
```

Continue clockwise through:

```text
E
SE
S
SW
W
NW
```

The user must not manually select the next compass point.

Progression is automatic.

---

# 9. Store Position Logic

The operator visually observes the buoy.

They use + and - until the buoy physically points exactly toward the selected compass direction.

Then they press:

```text
Store Position
```

Store:

```text
the adjustment value
```

for that compass point.

Example:

```text
North = +3°
```

After storing:

Automatically advance to the next compass point.

---

# 10. Progress Display

Show all 8 positions:

```text
N
NE
E
SE
S
SW
W
NW
```

Stored positions become green.

Remaining positions stay grey.

Clearly indicate which position is currently active.

---

# 11. Save Button Behavior

After all positions are completed:

```text
Save Fourier Table
```

must send all 8 stored values to the buoy.

Example:

```text
3,5,2,-1,-2,-4,-3,1
```

The generated table must become the active Fourier correction table.

---

# 12. Protocol Changes

Add:

```cpp
STATIC_COMPASS_TABLE = 90
```

to the protocol.

Generate the JavaScript code required to send:

```text
TargetID,99,SET,90,...
```

using the existing sendCommand() function.

---

# 13. ESP32 Firmware Additions

Generate the complete C++ code for:

```cpp
case STATIC_COMPASS_TABLE:
```

Receive:

```cpp
8 correction values
```

Store them into:

```cpp
float fourierTable[8];
```

Save them to NVM using Preferences.

Generate complete production-ready code.

---

# 14. NVM Storage Requirements

Generate code to:

```cpp
Load
Save
Initialize Defaults
Validate Data
```

for:

```cpp
float fourierTable[8];
```

Store the table in non-volatile memory.

---

# 15. Output Requirements

Provide:

## 15.1 Modified index.html

Only the required modified sections.

## 15.2 Modified style.css

Only the required modified sections.

## 15.3 Modified index.js

Complete copy/paste-ready code.

## 15.4 ESP32 Firmware

Complete copy/paste-ready code.

## 15.5 Protocol Enum Additions

All modifications required.

## 15.6 NVM Functions

Save, load, defaults, validation.

## 15.7 Helper Functions

Any additional helper functions required.

## 15.8 Insertion Locations

List the exact insertion point inside:

- index.html
- style.css
- index.js
- firmware

---

# 16. Response Requirements

Do not summarize.

Do not explain the concept.

Do not provide pseudocode.

Do not provide shortened examples.

Provide production-ready code blocks only.

All code must be ready for direct copy/paste into the existing Robobuoy project.