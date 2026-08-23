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
       │ - Navigation math  │                        │ - beep() sounder   │
       │ - RoboStruct       │                        │ - Tone / melody    │
       │ - Codec + CRC      │                        │   tables           │
       │ - Zero-compression │                        │                    │
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

*   **Frame CRC (`addCRCToString()` / `verifyCRC()`)** — also in `RoboCompute`, not `RoboTone`:
    *   Computes a longitudinal parity byte (XOR) over every character between the sentinel `$` and
        the delimiter `*`, rendered as a 2-character hexadecimal string.
    *   Incoming frames whose computed checksum does not match the trailing CRC are discarded.
*   **Circular wind statistics (`averageWindVector()`, `deviationWindRose()`)** — vector averaging
    and Yamartino circular standard deviation, so a wind direction either side of North averages
    correctly instead of landing on 180°.

### 2. `RoboTone` Library
`RoboTone` is the **audible feedback** library — it has nothing to do with checksums.

*   **`void beep(int sound, QueueHandle_t buzzer)`** — queues a tone, a melody or an error pattern
    onto a buzzer task. Callers pass a sound id: `1` is the confirmation arpeggio, `2` the
    unlock/idle pattern, `-1` the error tone, and a large value such as `1000` a single fixed tone.
*   Holds the tone and melody tables (`hz`, `duration`, `repeat`, `pause`) used by both firmwares.

> Sound is the buoy's only feedback in the field, so beeps carry real diagnostic weight. Note that
> a Top emits a single beep on boot — a burst of beeps usually means it has restarted several
> times, not that it acknowledged several commands.

---

## 🛠️ Integration
Both `RoboTop` and `RoboSub` reference these libraries inside their PlatformIO build
configurations via `lib_extra_dirs`:

```ini
lib_extra_dirs =
    ../RoboDependency

lib_deps =
    RoboCompute
    RoboTone
```

> ## ⚠️ Change this, rebuild both
>
> `RoboCompute` is statically linked into **both** firmwares. A change here is not live until
> `RoboTop` *and* `RoboSub` have both been rebuilt and reflashed — and because `RoboStruct` and the
> codec are shared, a half-updated fleet can end up decoding frames differently at each end. Never
> leave this library uncommitted while the firmwares that depend on it are committed.

`RoboLora` links `RoboCompute` too, so a codec change means rebuilding **three** firmwares.
`RoboCYD` is the exception: it links neither library and parses frames with its own code, so a
protocol change has to be mirrored there by hand.\n