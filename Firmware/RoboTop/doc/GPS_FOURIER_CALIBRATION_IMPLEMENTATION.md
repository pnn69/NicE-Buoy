# GPS Fourier Compass Calibration - Implementation Notes

Implements `GPS_FOURIER_CALIBRATION_SPEC.md`. This file records what was built, where it lives, and
where the implementation deviates from the spec and why.

---

## Files

| File | Change |
| --- | --- |
| `RoboDependency/RoboCompute/src/RoboCompute.h` | `STORE_INTERPOLATION_TABLE` (88) and `GPS_FOURIER_CALIBRATE` (89) added to `msg_t`; `float interpolationTable[8]` added to `RoboStruct` |
| `RoboDependency/RoboCompute/src/RoboCompute.cpp` | Encoder and decoder for both new commands |
| `RoboTop/src/gpscalib.h`, `RoboTop/src/gpscalib.cpp` | The whole state machine (new files) |
| `RoboTop/src/main.cpp` | Calls the state machine, routes the command in and the Sub's reply back, button combo, button light |
| `RoboTop/src/topwifi.cpp` | `CALIB_GPS_FOURIER` web command, `CalibMsg` progress field in `/data` |
| `RoboTop/data/index.html` | "GPS Fourier Calibration (8 legs)" button, progress line, status text for 89 |
| `RoboSub/src/main.cpp` | `STORE_INTERPOLATION_TABLE` handler: reports the effective table, or stores a new one in NVS |
| `RoboCYD/src/buoy_data.cpp` | Shows "GPS CALIB"; parses `cmd` 90; `send_gps_fourier_calibrate()` |
| `RoboCYD/src/buoy_data.h` | `cal_*` progress fields on `BuoyData` |
| `RoboCYD/src/main.cpp` | SETUP page 4 (ACTIONS): two start buttons and the live progress panel |

Nothing was changed in the compass fusion, hard iron, soft iron, gyro bias or filter parameters.
The only thing this feature writes is `measured_angles[]` (NVS key `meas_ang`) and the harmonic
enable flag (`interp_en`).

---

## Starting a run

Three ways, all of which just set the Top's status to `GPS_FOURIER_CALIBRATE`:

- Web UI: Setup dialog -> **GPS Fourier Cal - still water** or **- current running**
- Front button: seven short presses followed by one long press (pair-averaged mode)
- Protocol: `GPS_FOURIER_CALIBRATE` addressed to the Top over LoRa or UDP, with
  `gpsCalStillWater` in `fields[5]`

The mode is latched when the run starts, so changing it mid-run cannot switch the interpretation
half way through a set of legs. The front button has no way to ask, so it picks the conservative
option - see the two modes below.

Any status change stops it - pressing IDLE / LOCK / DOCK aborts immediately.

The whole routine runs on the **Top**. The Sub is only ever asked for `TGDIRSPEED` (steer this
heading at this thrust) and `STORE_INTERPOLATION_TABLE`.

---

## Sequence

```text
GC_FETCH_TABLE   ask the Sub which table is in effect       (4 s timeout, 5 tries)
   |
GC_SETTLE        steer leg heading, wait for |err| < 5 deg
   |             held for 20 s                              (180 s timeout)
GC_RUN           record start fix, sail until > 100 m       (420 s timeout)
   |             leg restarts if > 20 deg off for > 15 s    (3 attempts)
   |             ... repeat for all 8 legs ...
GC_STORE_TABLE   thrust to 0, send the new table, wait for
   |             the Sub to echo it back                    (4 s timeout, 5 tries)
GC_GO_HOME       target = start position, status = LOCKED, long beep
```

Leg order is the spec's: `0, 180, 45, 225, 90, 270, 135, 315`. Roughly 30 minutes end to end.

**That order is load-bearing and must not be flattened into a `0, 45, 90, ...` sweep.** Its original
job was arithmetic - a steady current cancels within each pair - and still-water mode no longer
needs that. It stays for a second, independent reason: each leg is immediately undone by its
opposite, so the buoy runs out and straight back along the same line four times, tracing a **half
star** centred on the start position with spokes at 0/45/90/135 only.

Simulated with 20 m of settle travel plus a 105 m measured leg, that keeps the buoy inside an
eastern half disc of about 125 m, never west of the start, finishing where it began:

| | |
| --- | --- |
| furthest from start | ~125 m |
| offset at finish | ~0 m |
| bounding box | N 125, S 88, E 125, W 0 |

A `0, 45, 90, ...` sweep would walk the buoy around an arc instead, ending far from the start and
needing several times the clear water. The still-water table mapping is `perHeading[GC_LEGS[i] / 45]`,
which is order independent, so the ordering costs the maths nothing.

Aborts (any of them cuts thrust and drops to IDLE with the error beep): no GPS fix at the start or
lost for 10 s, Sub silent on the serial link, compass heading frozen for 60 s, more than 1500 m from
the start position, or any of the timeouts above.

---

## Deviations from the spec

**1. `calculateBearing()`, not `calculateAngle()`.**
The spec (and the older `RoboTop/src/calibrate.cpp`) call
`calculateAngle(startLat, startLon, endLat, endLon)`. That helper takes two **2D vectors** and
returns the unsigned angle between them - it knows nothing about latitude and cannot return a
compass bearing. `calculateBearing(lat1, lon1, lat2, lon2)` is the great-circle initial bearing,
0..360, and is what the displacement bearing has to be.

**2. The new table is built on top of the one already in effect.**
The spec writes `measured_angles[i] = ref_i - corr_i`, which is only correct when the Sub is
starting from an identity table - otherwise the correction already in effect is silently discarded
and the buoy is calibrated back to where it started. Instead the Top asks the Sub for the table it
is currently applying and computes

```text
newTable[i] = oldTable[i] - corr[i % 4]
```

Entry `i` of the table is the reading seen while the buoy really pointed at `i*45`, so the
correction applied at that heading is `i*45 - table[i]`; subtracting the residual from the stored
reading adds it to the correction. With an identity table in effect this reduces **exactly** to the
spec's formula, so a first calibration behaves as specified and a second one refines the first.

To make that request meaningful the Sub reports the table that is **in effect**, not the one in NVS:
when the harmonic correction is switched off the compass is uncorrected, so it answers with the
identity table. Reporting a stored-but-unused table would make the Top subtract the same deviation
twice.

**3. Storing a table also switches the harmonic correction on.**
`interp_enabled` defaults to false, and `getInterpolatedHeading()` is only called when it is set. A
freshly calibrated buoy that ignores its own calibration is useless, so the Sub sets and persists
the flag when it commits a table.

**4. Extra safety and quality checks the spec does not mention.**
The runaway/GPS/serial/frozen-compass aborts above, and the "leg wandered off course" restart. A leg
sailed 30 degrees off the commanded heading measures the weather, not the compass.

---

## What this does and does not write

The Sub's `STORE_INTERPOLATION_TABLE` handler writes exactly **two** NVS keys:

| key | meaning |
| --- | --- |
| `meas_ang` | the 8-point interpolation table, `measured_angles[0..8]` |
| `interp_en` | whether `getInterpolatedHeading()` is applied at all |

Nothing else. **Hard iron, soft iron, gyro bias, `compassOffset`, the Madgwick/complementary filter
parameters, PID gains and thruster config are all untouched.** Hard iron is written only by
`HARDIRONFACTORS` (`RoboSub/src/main.cpp`), the `HardI:` serial command (`sercom.cpp`) and the desk
figure-8 (`memIcmCalib`, `compass.cpp`) - none of which are on this path.

### Why that distinction matters

The two live at different layers:

- **Hard-iron calibration works in sensor space.** It subtracts a constant 3-axis vector from the
  raw magnetometer readings, before any heading exists.
- **This procedure works in heading space.** The raw data and its bias are left alone; a correction
  curve is applied downstream, after the heading has been computed:

```cpp
heading += mainData.compassOffset;
if (interp_enabled) heading = getInterpolatedHeading(heading);   // <- only this
```

A residual hard-iron bias *shows up* in the heading domain as a semicircular (one-cycle) deviation,
and still-water mode can cancel that signature. But the magnetometer is still reading biased
numbers - the error is compensated at the output, not removed at the source.

Two practical consequences:

1. **Order is load-bearing. Redo the desk figure-8 -> re-run this calibration.** Never the other way
   round. The figure-8 changes `magHard`, removing bias the interpolation table was compensating
   for; the table then corrects an error that no longer exists and *adds* error.
2. **Heading-domain compensation is approximate.** Hard iron also distorts the field magnitude, and
   its heading-domain signature is only cleanly one-cycle when the sensor is level. Under pitch and
   roll it degrades in a way a sensor-space fix would not.

---

## Testing without committing anything

A **completed** run is the only thing that writes. Every abort path - no GPS fix, Sub silent,
compass frozen, drifted too far, any timeout - returns before the `SET` is sent, so an aborted run
leaves the table exactly as it was.

To inspect or preserve the current state, the Sub's own web server is the tool:

| call | effect |
| --- | --- |
| `GET /data` | returns `meas_ang` (9 floats) and `harmonic_enabled` - this is your backup |
| `GET /reset_harmonic` | table back to identity `0,45,...,315`, saved and recomputed |
| `GET /set_harmonic_point?index=N&measured=V` | set one entry (0..8) |
| `GET /save_harmonic` | commit the edited table to NVS |
| `GET /set_interp_enabled?enabled=1` | turn the correction on (`enabled=0` turns it off) |

**Recommended before a first real run:** read `/data`, keep the `meas_ang` array, then
`/reset_harmonic`. Starting from identity gives a clean single-pass measurement instead of a
correction layered on top of whatever was there. That matters because `computeFourierCoefficients()`
only fits harmonics 1 and 2 - if the existing table is jagged, no first or second harmonic can
represent it and the composition will not converge in one pass.

Sanity check on any table: convert it to errors with `err[i] = i*45 - meas_ang[i]` (normalised to
+/-180). Real hard and soft iron on a buoy is single digits to low tens of degrees. A curve spanning
a hundred degrees peak to peak is not a deviation curve, it is a leftover experiment.

---

## The two modes, and why the choice matters

Every leg error splits into a part that is the same at a heading and its opposite, and a part that
flips sign:

```text
even(C) = [e(C) + e(C+180)] / 2
odd(C)  = [e(C) - e(C+180)] / 2
```

A current is purely odd: to first order it biases the bearing by `(Vc/Vb) * sin(theta_c - C)`, which
negates at `C + 180`. **But a one-cycle hard-iron deviation has exactly the same shape.** One set of
legs sailed once cannot tell them apart, so the operator has to say which condition applies.

**Pair averaged** (`gpsCalStillWater = false`) throws the odd part away. Immune to a steady set, but
it also throws away real hard iron, and the surviving curve satisfies `err(t) = err(t+180)` by
construction - so `A1`/`B1` in `computeFourierCoefficients()` come out at zero. Corrects the
constant `A0` and the two-cycle `A2/B2` (soft iron) only.

**Still water** (`gpsCalStillWater = true`) uses each leg's own error as the deviation at that
heading. `A1`/`B1` survive, so the full classical deviation shape is compensated: A (constant), B/C
(semicircular), D/E (quadrantal). This is the better calibration **whenever the water is genuinely
still and the buoy presents little windage** - and it is the only mode that can compensate the
semicircular term, which is what residual hard iron looks like in the heading domain, including the
part that only appears in the water with the thrusters powered.

**Careful with the words "hard iron" here.** This procedure does NOT recalibrate hard iron - see
*What this does and does not write* above. The distinction changes the order you must run things
in.

Worked example, `e(0) = +10`, `e(180) = +2`, no current:

| | pair averaged | still water |
| --- | --- | --- |
| correction at 0 | +6 | +10 |
| correction at 180 | +6 | +2 |
| residual at 0 | **+4** | **0** |
| residual at 180 | **-4** | **0** |

Choosing still-water mode on a day when a current *is* running writes that current permanently into
the compass table. It is recoverable - re-run in the right conditions, or reset the table from the
Sub's harmonic page - but it is the one way to make the buoy worse than before.

### Telling which you had

`buildTable()` logs the even/odd split per pair either way, plus the worst odd leg, a single-sinusoid
fit through the four odd values, and that fit's residual. A steady current *is* one sinusoid, so a
near-zero residual supports the current explanation; a large one means the odd part is not a
one-cycle effect at all (noisy legs, a shifting breeze, a leg that was not really held) and the
fitted amplitude is not worth reading.

The **worst odd leg** is reported alongside the fitted amplitude on purpose: the fit is the best
single sinusoid through four points, so when they do not lie on one it understates the worst leg.
Odd values of `[4, 0, 0, 0]` fit an amplitude of 2.0 while one leg is really 4.0 out.

---

## Wire format

```text
STORE_INTERPOLATION_TABLE, status, t0, t1, t2, t3, t4, t5, t6, t7
```

Ten fields, two decimals each, well inside the 25-field decoder limit. Presence is decided by the
field count (`count > 9`), not by empty tokens - `RoboCode()` compresses any all-zero token to `""`,
so a `0.00` entry arrives as an empty field and must still be read as zero.

| ack | meaning |
| --- | --- |
| `GET` / `GETACK` | Top -> Sub: which table are you applying? |
| `SET` | Top -> Sub: store this in NVS, recompute the coefficients, switch the correction on |
| `INF` | Sub -> Top: the table I am applying / the table I just stored |

The Sub only writes on `SET`. The Top's own frames are dropped by the half-duplex echo filter in
`sercom.cpp` before they can be mistaken for a reply.

---

## Progress reporting (`GPS_FOURIER_STATUS`, 90)

While a run is active the Top broadcasts a progress frame with `ack = INF` and `IDr = BUOYIDALL`:
**UDP every 1 s, LoRa every 5 s**. LoRa gets the slower rate because it is already carrying
`BUOYPOS` + `TOPDATA` every second while the buoy is not idle; a 30 minute run adds ~360 frames.

Two frames are **forced** regardless of the rate limits, so a listener always learns how the run
ended instead of just going quiet:

- step `GPSCAL_DONE` when the calibration completes, immediately followed by a
  `STORE_INTERPOLATION_TABLE` broadcast (`ack = INF`) carrying the resulting table
- step `GPSCAL_ABORTED` on any abort, sent after the status has already dropped to IDLING

### Field layout

As a raw comma-split sees it — the same indexing `RoboCYD/src/buoy_data.cpp` already uses, where
`fields[0..4]` are `IDr, IDs, ack, cmd, status`:

| index | field | meaning |
| --- | --- | --- |
| `fields[3]` | `cmd` | `90` |
| `fields[4]` | `status` | buoy status; `89` = `GPS_FOURIER_CALIBRATE` while running |
| `fields[5]` | `gpsCalStep` | phase, see `gpscal_step_t` |
| `fields[6]` | `gpsCalLeg` | leg index `0..7` into the `0,180,45,225,90,270,135,315` order |
| `fields[7]` | `tgDir` | heading commanded for this leg (0 outside the steering phases) |
| `fields[8]` | `dirMag` | heading the buoy is reporting right now |
| `fields[9]` | `gpsCalDist` | metres covered on this leg so far (0 outside `RUN`) |
| `fields[10]` | `gpsCalErr` | **live** signed error of the leg in `fields[6]`, degrees |
| `fields[11]` | `gpsCalLastErr` | signed error of the last **completed** leg, degrees |

`gpsCalErr` is the bearing from this leg's start fix to the current fix, minus the commanded
heading — computed exactly the way the final value is at the end of the leg, so it can be watched
converging rather than only appearing once the leg is over.

It reads **0 until `gpsCalDist` passes 10 m**. Over the first few metres the start and current fix
are within GPS noise of each other, so the bearing between them is close to random and the error
would swing through ±180° for the first half minute. No sentinel is needed to detect this —
`gpsCalDist` is in the same frame, so `dist < 10` means "not meaningful yet".

During `STORE` and `DONE` both `gpsCalLeg` and `gpsCalErr` refer to leg 7, by then measured rather
than live. `gpsCalLastErr` exists so a display still has a result on screen through the 20 s settle
phase of the next leg, when there is no live measurement at all.

`gpscal_step_t` (defined in `RoboCompute.h`, so the CYD can be written against it):

| value | phase |
| --- | --- |
| 0 | `GPSCAL_IDLE` |
| 1 | `GPSCAL_FETCH_TABLE` — asking the Sub which table it applies |
| 2 | `GPSCAL_SETTLE` — steering the leg heading, waiting for it to hold |
| 3 | `GPSCAL_RUN` — measuring displacement |
| 4 | `GPSCAL_STORE` — handing the table to the Sub |
| 5 | `GPSCAL_DONE` — finished, returning to the start position |
| 6 | `GPSCAL_ABORTED` — stopped early |

The live heading deviation is `tgDir − dirMag`, so it is not sent separately.

### RoboCYD

Done. `parse_buoy_packet()` accepts `cmd` 90 and stores the run into `BuoyData` (`cal_step`,
`cal_leg`, `cal_cmd_dir`, `cal_dist`, `cal_err`, `cal_last_err`, `cal_seen_ms`).

**Starting a run:** SETUP screen -> **PG 4/4**, headed `COMPASS OFFSET CAL` (pages 1-3 are headed
`SETUP`). Neither header carries the buoy ID any more - the nav screen it was opened from already
shows it. The page carries two buttons:

| box | mode |
| --- | --- |
| top left, green - `CAL STILLWTR` | still water, per leg |
| top right, orange - `CAL CURRENT` | pair averaged |

Tapping a box only selects it; the **`+`** button then starts the run. Two steps on purpose - every
other control on that screen toggles on either `+` or `-`, and this one commands the buoy to sail
for half an hour, so it must not be reachable by a single stray touch.

`-` is greyed out on this page and both buttons are inert unless a mode box is armed. Without that
guard, `+`/`-` would have edited whichever parameter was left selected on a previous page -
invisibly, since that value is not displayed here - and a later SAVE would commit an edit the
operator never made. The strip between the buttons shows `SELECT MODE` until a box is armed, then
`TAP + TO RUN`.

The frame is `id,98,3,89,89,<0|1>` (`send_gps_fourier_calibrate()`). It needs its own sender because
`send_buoy_command()` emits an empty payload, which would always decode as pair-averaged.

**Watching a run:** the rest of the page is a live panel - phase, leg *n* of 8, commanded heading, a
progress bar for the leg, the live error, and the last completed leg's error. It repaints only when
a value changes; `update_dynamic_ui()` runs every 250 ms and a full repaint at that rate flickers.
While no run is active the same panel explains what the two modes do.

`draw_gps_cal_panel()` restores `MC_DATUM` and text size 1 on **both** exit paths. TFT_eSPI datum is
global state and the function is called from the middle of `update_setup_dynamic()`; leaving
`TL_DATUM` behind made the strip below draw left-aligned on repaint frames and centred on cached
frames, so the text landed in two different places and the left-aligned tail stuck permanently to
the `+` button. Anything drawn after a helper here must set the datum rather than inherit it.

Page 4 renders without waiting for the `SETUPDATA` reply, unlike the parameter pages, because it
shows no setup values.

The status **label** needed no change - "GPS CALIB" comes from status `89` on the ordinary
`TOPDATA`/`BUOYPOS` telemetry, which is mapped in `buoy_data.cpp`.

---

## Log output

Every leg prints its full record to the serial console, prefixed `#GPSFOURIER:`:

```text
#GPSFOURIER: LEG 0
#GPSFOURIER:   CMD   = 0
#GPSFOURIER:   START = 52.32049144, 4.96544428
#GPSFOURIER:   END   = 52.32160312, 4.96565901
#GPSFOURIER:   DIST  = 124.3m
#GPSFOURIER:   GPS   = 6.4
#GPSFOURIER:   ERR   = +6.4
```

followed by the four pair averages, the old table and the new table, and a summary of all eight legs
at the end. The web dashboard shows a one-line progress message under the Setup buttons while a run
is active.
