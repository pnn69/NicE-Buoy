# RoboDependency — Shared Static Libraries & Algorithmic Engines

The **`RoboDependency`** workspace houses core statically-linked C++ library engines shared across both the **`RoboTop`** (Master Supervisor) and **`RoboSub`** (Locomotion Engine) firmwares. Decoupling these common engines ensures mathematical consistency and guarantees zero duplication in message encoding.

---

## 🏗️ Library Structure

```
                             ┌────────────────────────┐
                             │     RoboDependency     │
                             └───────────┬────────────┘
                  ┌──────────────────────┴──────────────────────┐
                  ▼                                             ▼
       ┌────────────────────┐                        ┌────────────────────┐
       │    RoboCompute     │                        │      RoboTone      │
       │ - Navigation Math  │                        │ - XOR 2-Char CRC   │
       │ - Zero-Compression │                        │ - Checksum Audit   │
       │ - State Structures │                        │                    │
       └────────────────────┘                        └────────────────────┘
```

---

## ⚡ Module Breakdown

### 1. `RoboCompute` Library
`RoboCompute` forms the computational backbone of the buoy system, implementing localized navigation math and data compression pipelines:

*   **Great-Circle Path Math (`RouteToPoint()`)**:
    *   Calculates shortest path bearing angles (0 to 360 degrees) and distances (in meters) between arbitrary GPS coordinates (latitude/longitude) on the earth's surface.
    *   Supplies the core error vectors utilized by the high-speed rudder and speed PID tasks on the Sub Unit.
*   **Unified State Struct (`RoboStruct`)**:
    *   Establishes the global data structure (`mainData`) representing the absolute physical state of the buoy (GPS status, heading, thruster duty cycles, battery state, target waypoints, and PID tuning variables).
    *   Allows seamless, thread-safe data serialization across task loops via mutex locks.
*   **Bandwidth-Saving Float Trailing Stripping (`formatFloat()`)**:
    *   Trims redundant trailing zeros and decimal points (e.g., `24.0000` is packed to `24`, and `1.230` to `1.23`) before packaging payload variables, significantly reducing string lengths.
*   **Zero-Compression Payload Packing (`rfCode()` / `rfDeCode()`)**:
    *   Scans finalized ASCII CSV payloads for repetitive zeros representing inactive fields (e.g., zero error, zero speeds, empty target parameters) and replaces them with adjacent commas (e.g., `,0,0,0,0,` is packed to `,,,,,`).
    *   This dual optimization reduces the average LoRa payload size by up to **45%**, decreasing airtime, saving battery, and reducing channel congestion.

### 2. `RoboTone` Library
`RoboTone` is dedicated to ensuring packet integrity across wireless channels and high-noise physical lines:

*   **XOR-Based 2-Character Hex CRC Calculation**:
    *   Computes a fast, efficient longitudinal parity byte (XOR-checksum) of all payload characters between the starting sentinel `$` and payload delimiter `*`.
    *   Translates the result into a standardized 2-character Hexadecimal validation string.
*   **Checksum Verification**:
    *   Audits incoming serial packages on both processors. If a computed checksum mismatches the packet's metadata CRC trailing string, the frame is instantly flagged as corrupted and discarded.

---

## 🛠️ Integration
Both `RoboTop` and `RoboSub` reference these libraries natively inside their PlatformIO build configurations by linking them via the `lib_extra_dirs` parameters:
```ini
lib_extra_dirs =
    ../RoboDependency
```\n