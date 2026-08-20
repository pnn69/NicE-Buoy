# Update report — night of 2026-08-20 / 21

Branch `cyd-setupdata-retry`, commits `9bf2dfab` … `6ebe6b23` (9 commits).
All firmware built and flashed: both Tops (`192.168.1.71`, `192.168.1.78`), both Subs
(one over OTA at `192.168.1.101`, one over USB `COM41`) and the CYD (`COM11`, also on the
network at `192.168.1.235`).

Started from one report — "Active Trim is not stored when set from the web page" — and the
audit that followed pulled in the rest.

---

## What was fixed

### 1. Active Trim did not save — `9bf2dfab`

**Cause.** The Setup page's "Active Trim Enabled" tick box travels in the `SETUPDATA` frame
(field 16, `RoboCode()`). Everything on the way to the Sub was correct, but the Sub's
`SETUPDATA` handler never looked at the field — it stored PIDs, speeds, offset, inversion and
swap and silently dropped the flag. It looked *actively* broken rather than merely inert because
the Sub re-announces its trim state every second over `ADAPTIVE_TRIM`, so the tick box flipped
back on the next `/data` poll. Only the **Toggle Trim** button worked, because that sends
`ADAPTIVE_TRIM`, which the Sub does handle.

**Fix.** The Sub applies and persists the flag via `memCompassTrim()` on `SETUPDATA` SET,
leaving the accumulated trim *value* untouched (it is not in that frame). The Top takes the flag
from both the incoming frame and the Sub's reply so the page updates immediately.

**Verified on hardware** — replaying exactly what UPDATE CONFIG sends, values read back from the
buoy so nothing else changed:

```
start   : enabled=true  trim=-8.774
+2s     : enabled=false trim=-8.774
+4s     : enabled=false trim=-8.774     <- previously reverted within 1 s
+6s     : enabled=false trim=-8.774
restored: enabled=true  trim=-8.774
```

Confirmed on both buoys. Original setting restored afterwards.

### 2. Two more parameters written but never read back — `9bf2dfab`

- `swap_BB_SB` — stored on every save, missing from the boot load.
- `compassOffset` — written to both `magCorr` (double) and the legacy `Delta` (int), but only the
  int was read at boot, throwing away the decimals of an offset the page accepts in 0.1 steps.

Both were later made moot by item 6, which removed them from the Top entirely.

### 3. Mech Corr removed — `9bf2dfab`

Dead system-wide: loaded at compass init and never added to a heading, absent from the wire
format, and unsettable for remote buoys. Had it ever been implemented it would have done exactly
what `compassOffset` already does. **Set as North is not a separate parameter either** — it
computes that same `compassOffset` from the current heading (`RoboSub/src/main.cpp:531`).
Removed from the page, `topwifi.cpp`, both `datastorage` modules, the Sub's compass init and the
shared `RoboStruct`. The orphaned `mechCorr` NVS key is harmless and was left alone.

### 4. Harmonic (Fourier) correction exposed — `bb52a726`

The 8-point table from the GPS calibration is only applied while the Sub's `interp_enabled` is
on, and that switch could previously only be reached from the Sub's own web page — unreachable
from the boat. A buoy could finish a half-hour calibration and keep sailing on an uncorrected
compass with nothing in the UI to say so.

`SETUPDATA` gains **field 20**. It is **tri-state (1 = off, 2 = on)**, not the plain 0/1 the other
flags use: `RoboCode()` compresses an all-zero token to `""`, so a plain 0 could not be told apart
from a field a shorter frame never sent — and `RoboControl.py` still writes `SETUPDATA` frames
that stop at `swap_BB_SB`.

Verified end to end against the Sub's own page, which reports the flag independently:
`Top=true/Sub=1 → toggled → Top=false/Sub=0 → held → restored`.

### 5. Reboot rebooted the buoy five times — `eccf0fc5`, `ff7e85bc`

**Cause.** The Reboot buttons added to the CYD sent `ack = GETACK (3)`, copied from the display's
existing `send_buoy_command()`. `RoboTop/src/loratop.cpp:369` puts any GETACK or SET packet in a
retransmit table with `retry = 5` and resends it about once a second until the target answers —
and the receive path deliberately sends no immediate ACK for a GETACK. A buoy busy rebooting
never answers, so all five retries landed on a freshly booted Top. `retry = 5`, five reboots.

**Fix.** Fire-and-forget actions go out as `INF (6)`, which is not enrolled. `send_buoy_command()`
takes the ack as a defaulted argument so mode changes (LOCK/DOCK/IDLE), where the retry *is*
wanted and the buoy does reply, are untouched. `send_gps_fourier_calibrate()` switched too.
RoboTop additionally ignores a `REBOOT` arriving in its first 15 s of uptime, which breaks the
cycle whatever the sender does.

**Verified:** one command → one reboot (down at 2.9 s, up at 10.5 s, nothing for the next 80 s).

### 6. The Sub owns its own settings — `e3105624`

Thruster inversion and swap describe how the motors are wired to the Sub (`RoboSub/src/esc.cpp:176,181`
is the only code in the system that applies them), and the compass, its offset and both PID loops
are on the Sub as well. All of it already lived in the Sub's NVS. The Top kept a **duplicate** in
its own flash and restored it at boot — two copies of one truth.

That duplication is what made a Sub moved to another hull look like corrupted settings: each Top
went on reporting its stale cache until the new Sub answered, and the two buoys appeared to have
swapped configuration. **Nothing was actually wrong.**

Removed from the Top's flash and boot load, plus the mirrored writes in `handleRfData()` and the
web save: `thrusterInversion`, `thrusterSwap`, `pidRudderParameters`, `pidSpeedParameters`,
`computeParameters`, `CompasOffset` and the legacy int `CompassOffsetCorrection`. The Top keeps a
RAM copy for display and relaying and asks the Sub for `SETUPDATA` at serial link-up, which it
already did. It still owns dock position, dock approach, buoy id and WiFi credentials.

**Verified on a cold boot:** the Top's first HTTP response already carried the Sub's
Kpr/Kps/offset/holdRad/thruster flags — no window in which the Setup dialog could show or save a
blank.

### 7. Dead command path found — `670049ea`

**Desk Calibration never worked from anywhere except the Top's own physical button.** The Sub has
always handled `CALIBRATE_MAGNETIC_COMPASS`, but `handleRfData()` had no case for it, so the web
page's button was silently dropped. Added.

### 8. UI work — `670049ea`, `2a94329b`

- **CYD display**: setup laid out to mirror the web form — six pages, one section heading each
  (PID / SPEED & COMPASS / TRIM & THRUSTERS / DOCKING / CALIBRATION / GPS COMPASS CAL), the
  heading shown as the page title. Slots are now a named table (`SETUP_NAMES`, `enum SetupSlot`)
  with the arithmetic in one `setup_adjust()`, replacing two mirrored if-chains of magic indices —
  the drift that had made the new Harmonic box unselectable. Desk Cal and Reboot added behind the
  two-step the GPS calibration already used.
- **CYD web page** (a separate page from RoboTop's, served at `192.168.1.235`): rebuilt to the
  Top's sections, order and wording. Two things were missing rather than differently arranged:
  **`revBB` had no control at all** — the save echoed back `b.data["revBB"]`, so BB inversion could
  not be changed from that page — and Desk Cal, both GPS Fourier modes and Reboot had no buttons.

### 9. Diagnostics added — `127c4b95`, `6ebe6b23`

`/data` now carries `Uptime`, `ResetReason` (`PANIC` / `WATCHDOG-task` / `BROWNOUT` / `power-on` /
`software`) and the Top's own `TopVolt` / `TopCurr` / `TopPerc`.

---

## The reported crash on 192.168.1.78 — not reproduced

| Test | Result |
|---|---|
| 70 s at 0.4 s polling, 3 min at 1.5 s | no outage, `rev` never reset, `SubOk` true throughout |
| `index.html` ×5, `/data` ×40 rapid, `SETUPDATA` GET, full `SETUPDATA` SET | alive after each |
| LOCK ×3 (45 s, 22 s, 35 s) | `Uptime` climbed straight through all three |
| Rail under full thruster load | 15.28 → 15.26 V; Sub 24.06 → 24.00 V — no sag |

`ResetReason` never left `software`, which is what an OTA restart looks like — and `.78` was
flashed eight times during the session. The most likely explanation is those uploads. Now
instrumented: a real fault will read `PANIC`, `WATCHDOG-task` or `BROWNOUT`.

The bench LOCK trace showed the heading frozen at 79° under full pivot thrust. Per the last water
test that is a bench artefact — the buoy could not physically turn.

---

## Open items

1. **Compass calibration** — the actual blocker per the last water test. The GPS Fourier run is
   now reachable from both UIs, and the Harmonic Correction switch decides whether the resulting
   table is applied. Current state: `b7a5b578` harmonic ON, `b7a5099c` harmonic OFF.
2. **Top battery divider.** Firmware is already byte-identical to the Sub's (same GPIO35, same
   `21.97/1.630`, no attenuation difference in either project), so the discrepancy is a fitted
   component. Top reads 15.27 V against the Sub's 24.06 V on `.78`, and 15.68 vs 24.51 on `.71` —
   agreeing to 0.8%, so a fixed scaling error. If both boards share a pack the constant wants to be
   about **21.16** instead of 13.4785; a 10 kΩ / 500 Ω divider gives exactly 21.0, so the likely
   story is a 10 k fitted on the Top where the Sub has 6.2 k. **Not applied** — deriving it from
   another board's reading assumes a shared pack. Needs a meter. Nothing on the Top acts on this
   value (`main.cpp:1111` only stores it), so it is cosmetic.
3. **±180° wrap chatter.** With the target near-astern the heading error sits on the wrap and the
   command flips full-scale between `BB=-75/SB=+63` and `BB=+75/SB=-75`. Afloat the buoy turns and
   it resolves, so not urgent, but it is a real rough edge in the rudder PID.
4. **`RoboLora/data/index.html`** is a 92 % copy of the CYD's *old* page — same outdated Setup
   form, presumably the same missing `revBB`. Untouched.
5. **`RoboControl.py` writes short `SETUPDATA` SET frames** (stops at `swap_BB_SB`). Fields beyond
   that keep whatever the receiving struct already holds. The count-based guards in `RoboDecode()`
   limit the damage and the new harmonic field is tri-state for exactly this reason, but
   `compass_trim_enabled` and the three dock fields remain exposed.
6. **The Sub disables the ESP32 brownout detector** (`RoboSub/src/main.cpp:153`), the Top does not.
   So a supply dip resets the Top while the Sub carries on. Copying the Sub's line would mask real
   power problems and risks running flash out of spec — worth a deliberate decision rather than
   matching by default.
7. **Intermittent ICM-20948 on one Sub.** One boot logged `ICM-20948: CRITICAL ERROR - Sensor not
   found on I2C bus!` and reported heading 0.00; the next boot found it and read a real heading.
   Flaky power-up or I2C, worth watching.
8. **Unverified suspicions**, noticed while reading and never chased:
   - `handleRfData()` `case ADAPTIVE_TRIM` writes `RfOut->compass_trim*` with no sender check.
   - `case SUBDATA` adopts another node's heading and battery as its own when `IDs == 0`.
   - `is_local` in `case SETUPDATA` is true whenever `from_udp` is true.
   None of these was shown to misbehave; they are listed as places to look, not as findings.

## Hardware notes

- The two Subs were physically swapped between hulls during the session. Since the tuning lives in
  the Sub's NVS, PIDs, compass offset and thruster flags travelled with them — correct behaviour.
- Tops keep stable addresses (`.71` = `f8:b3:b7:a5:09:9c`, `.78` = `f8:b3:b7:a5:b5:78`). Subs move,
  so never assume the pairing: verify by matching the Sub's heading against each Top's `MagDir`.
- ArduinoOTA listens on **UDP** 3232 — scanning TCP 3232 finds nothing. Probe TCP 80 instead.
- A RoboTop change also needs `-t uploadfs`; the CYD serves its own page and needs its own
  `uploadfs`. Its `platformio.ini` says `COM15`, but it enumerated as `COM11`.
