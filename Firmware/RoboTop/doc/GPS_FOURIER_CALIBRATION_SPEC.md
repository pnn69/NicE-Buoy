# Robobuoy GPS-Based Fourier Compass Calibration
Version: 1.0
Status: Design Proposal

---

# Objective

Implement a fully automatic in-field compass calibration routine using GPS displacement vectors.

The purpose is to generate an 8-point interpolation table for the existing Fourier correction system already implemented in the Sub's compass firmware.

The calibration shall compensate for:

- Residual hard-iron errors
- Residual soft-iron errors
- Mechanical installation offset
- Sensor alignment errors
- Magnetic disturbances caused by wiring and electronics

The calibration shall NOT modify:

- Hard-Iron offsets
- Soft-Iron matrix
- Gyro bias
- Madgwick / Complementary filter parameters

Only the interpolation table shall be updated.

---

# Existing System

The Sub already supports:

```cpp
float measured_angles[9];
computeFourierCoefficients();
getInterpolatedHeading();
memInterpolationTable();
```

The interpolation system currently expects 8 calibration points:

```cpp
0°
45°
90°
135°
180°
225°
270°
315°
```

and computes a Fourier correction curve.

---

# System Architecture

## Top Responsibilities

The Top shall:

1. Run the calibration state machine.
2. Control the buoy heading.
3. Record GPS positions.
4. Calculate displacement bearings.
5. Calculate heading errors.
6. Generate interpolation table.
7. Send interpolation table to Sub.
8. Command Sub to save table to NVS.

## Sub Responsibilities

The Sub shall:

1. Accept a new interpolation table.
2. Store it in NVS.
3. Call:

```cpp
computeFourierCoefficients();
```

4. Immediately use the new correction values.

---

# Why GPS Start-End Bearing?

The buoy moves too slowly for reliable GPS COG.

Do NOT use:

```cpp
gps_course_over_ground
```

Use:

```cpp
bearing(
    start_position,
    end_position
)
```

instead.

The start-end displacement bearing is much more stable.

---

# Calibration Heading Set

The calibration shall perform 8 navigation legs.

Required order:

```text
0°
180°

45°
225°

90°
270°

135°
315°
```

Opposite headings must be measured consecutively.

This minimizes the influence of changing water current.

---

# Pair Calibration Principle

Water current introduces a systematic bearing error.

Example:

Current flows East.

North run:

Commanded Heading:

0°

Measured GPS Bearing:

8°

Measured error:

+8°

South run:

Commanded Heading:

180°

Measured GPS Bearing:

172°

Measured error:

-8°

The compass may actually be perfect.

The apparent error is caused by current.

Opposite headings are therefore treated as pairs.

---

# Geometry

Pair A:

```text
0°
180°
```

Pair B:

```text
45°
225°
```

Pair C:

```text
90°
270°
```

Pair D:

```text
135°
315°
```

---

# Calibration State Machine

## Phase 0

Store home position.

```cpp
homeLat = currentLat;
homeLon = currentLon;
```

---

## Phase 1

Command heading.

Example:

```cpp
TGDIRSPEED

heading = 0°
speed = 50
```

---

## Phase 2

Wait for stabilization.

Required condition:

```cpp
abs(dirMag - commandedHeading) < 5°
```

for:

```text
20 seconds
```

before beginning measurement.

---

## Phase 3

Record start position.

```cpp
startLat = currentLat;
startLon = currentLon;
```

---

## Phase 4

Sail straight.

Continue until:

```cpp
distanceBetween(
    startLat,
    startLon,
    currentLat,
    currentLon
)
>
100 meters
```

Recommended:

```text
100-150 m
```

minimum displacement.

Do not use a fixed time interval.

---

## Phase 5

Record end position.

```cpp
endLat = currentLat;
endLon = currentLon;
```

---

## Phase 6

Calculate bearing.

Use existing helper:

```cpp
calculateAngle(
    startLat,
    startLon,
    endLat,
    endLon
);
```

Store:

```cpp
gpsBearing[index];
```

---

## Phase 7

Repeat for all 8 headings.

---

# Error Calculation

For every heading:

```cpp
error =
gpsBearing -
commandedHeading;
```

Normalize:

```cpp
while(error > 180) error -= 360;
while(error < -180) error += 360;
```

Store:

```cpp
headingError[8];
```

---

# Pair Error Computation

Given:

```cpp
e0
e180

e45
e225

e90
e270

e135
e315
```

Compute:

```cpp
pairAverage0 =
(e0 + e180) / 2.0f;

pairAverage45 =
(e45 + e225) / 2.0f;

pairAverage90 =
(e90 + e270) / 2.0f;

pairAverage135 =
(e135 + e315) / 2.0f;
```

These pair averages represent the best estimate of the true compass deviation after most current effects cancel.

Store:

```cpp
corr0
corr45
corr90
corr135
```

using the pair averages.

---

# Interpolation Table Generation

Generate:

```cpp
measured_angles[0] =   0 - corr0;
measured_angles[1] =  45 - corr45;
measured_angles[2] =  90 - corr90;
measured_angles[3] = 135 - corr135;

measured_angles[4] = 180 - corr0;
measured_angles[5] = 225 - corr45;
measured_angles[6] = 270 - corr90;
measured_angles[7] = 315 - corr135;
```

---

# New Protocol Command

Add a new command:

```cpp
STORE_INTERPOLATION_TABLE
```

Payload:

```cpp
float interpolationTable[8];
```

Top sends:

```cpp
STORE_INTERPOLATION_TABLE
```

to Sub.

---

# Sub Action

Upon receiving:

```cpp
STORE_INTERPOLATION_TABLE
```

execute:

```cpp
memInterpolationTable(
    interpolationTable,
    MEM_PUT
);

computeFourierCoefficients();
```

---

# Serial Logging Requirements

Each leg shall log:

```text
Calibration Leg
Commanded Heading
Start Latitude
Start Longitude
End Latitude
End Longitude
Distance Travelled
GPS Bearing
Heading Error
```

Example:

```text
LEG 0

CMD=0°
DIST=124m

GPS=6.4°

ERR=+6.4°
```

---

# Final Step

After all 8 legs:

1. Generate interpolation table.
2. Send interpolation table to Sub.
3. Save to NVS.
4. Recompute Fourier coefficients.
5. Return buoy to home position.
6. Signal completion with buzzer and status message.

---

# Required Future Output

If this markdown file is provided to an LLM, it shall generate:

1. Full Top state machine implementation.
2. Required RoboStruct additions.
3. Required command enum additions.
4. Required serial protocol changes.
5. Required Sub firmware changes.
6. NVS storage handling.
7. Diagnostic logging.
8. Error handling and timeout logic.
9. Complete compilable C++ code.

End of specification.