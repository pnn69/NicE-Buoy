# RoboTop — Autonomous Marine Buoy Master Supervisor Firmware

The **`RoboTop`** firmware runs on the master ESP32 microcontroller of the NicE-Buoy Autonomous Marine System. It serves as the primary intelligence, networking, and supervisory node, managing global positioning, wireless telemetry channels, client-facing control dashboards, and delegating physical motor actions to the Sub Unit (`RoboSub`).

---

## 🏗️ Architecture & Core Tasks

RoboTop leverages FreeRTOS to coordinate real-time operations across the ESP32's dual cores:

```
┌─────────────────────────────────────┬─────────────────────────────────────┐
│               Core 0                │               Core 1                │
├─────────────────────────────────────┼─────────────────────────────────────┤
│ WiFiTask   8192  web, UDP, OTA      │ LoraTask   8192  SPI long-range RF  │
│ SerialTask 6000  link to the Sub    │ GpsTask    2000  TinyGPS++ polling  │
│                                     │ buttonTask 2048  press detection    │
│                                     │ LedTask    2000  / buzzTask 2048    │
└─────────────────────────────────────┴─────────────────────────────────────┘
                 loop() on core 1: navigation, state machine, RF dispatch
```

The numbers are stack sizes in bytes, and they matter. `LoraTask` was once given 4000 and ran with **100 bytes of headroom**, which tripped the stack canary under load and produced a panic that pointed at every task except the guilty one. Live headroom is published in `/data` as `StkLoop`, `StkLora`, `StkWifi`, `StkSer` — check it after any change that adds work to a task.

---

## ⚡ Core Functional Modules

### 1. GPS Acquisition & Navigation Telemetry
*   **Hardware Interface**: GPS module over UART, polling raw NMEA sentences.
*   **Sentence Parsing**: `TinyGPSPlus` extracts latitude, longitude, heading and speed over ground.
*   **Fix validity**: a fix requires `isValid()` **and** an age under 2000 ms **and** a position that is not `0,0`. `isValid()` alone only means "a position field was parsed" — it stays true across a sentence carrying zeros, and passing that on sends the rest of the firmware chasing a point off the coast of Africa.

### 2. Long-Range LoRa Telemetry (`loratop.cpp`)
*   **Hardware Interface**: SPI LoRa transceiver for bidirectional telemetry with the shore station and the other buoys. **433 MHz** (`LoRa_frequency` in `loratop.h`), library defaults otherwise — SF7, 125 kHz, CR 4/5, hardware CRC on. The band is `#define`d separately here, in RoboLora and in RoboCYD, and all three must match; see the RoboLora README.
*   **Retry & ACK Protocol**: `pendingMsg[10]` holds frames sent with `ack = GETACK` or `SET`, retransmitting each up to 5 times until acknowledged.
    > **A broadcast must never ask for an ACK.** An ACK for `IDr = BUOYIDALL` can never match in `removeAckMsg()`, so the frame occupies its slot for all five retransmits and re-broadcasts each time. Beacons use `INF`.
*   **Self-Healing SPI Watchdog**: if transmission fails over a 500 ms window the radio is assumed locked and `InitLora()` forces a full re-init.

### 2b. The transparent repeater

A node relays frames addressed to **someone else**, in case that someone is out of the sender's
range but inside ours. Every rule below exists because the naive version of it failed in the field,
so they are worth reading before changing any of them. Same design in `RoboLora`.

**Unicast only — with one exception.** Broadcasts are not relayed. They used to be, simply because
a broadcast is not "addressed specifically to us" and so fell through into the relay branch. All
periodic telemetry is broadcast (`handleTimerRoutines` sets `IDr = BUOYIDALL`), so every Top
rebroadcast every other Top's telemetry. At SF7/125 kHz a 116-byte `TOPDATA` is ~195 ms of airtime,
so three locked buoys sending one frame a second came to roughly **195% channel occupancy**. The
channel could not carry it, and what got dropped was the traffic that mattered — a COMPUTE STARTLINE
would compute and its `SETLOCKPOS` frames would never arrive. Relaying a broadcast buys nothing
anyway: a node close enough to hear the relay had almost certainly heard the original, and every
buoy broadcasts its own telemetry regardless.

The exception is **`LORA_LINK`**, and it is an exception precisely because neither of those
arguments holds: the whole point of a link report is to arrive from a node you *cannot* hear
directly, and nobody else can produce it on that node's behalf. One frame a minute per node is about
0.2% of the channel.

**Dedup cache** (`checkAndRecordRepeaterMessage`, 16 entries, 20 s). A frame is matched verbatim and
relayed at most once per 20 s window. This is what stops a relay going round in circles between two
nodes that can both hear each other.

**Deferred, staggered relays.** A relay is queued with a due time, not transmitted from inside
`onReceive()`. Synchronous relaying meant every node that heard the frame relayed it in the same
millisecond, so the relays collided with each other — and a node is deaf while it transmits, so each
one also missed whatever else was on the air. Nothing about that design could work with more than
one relay in the field.

The delay comes from the node's **own MAC** (`repeatDelayMs()`), folded down to one of
`REPEAT_NODE_SLOTS`, so two nodes holding the same frame are never due at the same moment. Pure
randomness would still collide occasionally; a MAC-derived slot cannot, provided the two boards fold
to different slots. A small random jitter is added only to break the tie when they do not.

| Constant | Value | Why |
|---|---|---|
| `REPEAT_SLOTS` | 4 | relays that may be waiting at once |
| `REPEAT_NODE_SLOTS` | 5 | distinct per-MAC time slots |
| `REPEAT_SLOT_MS` | 250 | wider than the airtime of any frame relayed, so slot 0 finishes before slot 1 starts |
| `REPEAT_BASE_MS` | 40 | dead time before slot 0, lets the sender's tail clear |
| `REPEAT_JITTER_MS` | 60 | tie-break when two MACs fold to the same slot |
| `REPEAT_EXPIRY_MS` | 3000 | give up on a relay the radio will not accept |

**The slot budget is bounded by the retry interval, not chosen freely**: `4 x 250 + 40 + 60 =
1100 ms`, comfortably inside the ~1500 ms the sender's retry table waits. Widen the slots or add
more and relays start landing after the retransmit they were meant to save.

**Cancel on heard** (`cancelRepeat`). Before anything else in `onReceive()`, a queued relay matching
the frame just heard is dropped — somebody beat us to it, or the sender resent it. Deliberately
ahead of the early returns, so it applies to frames addressed to us as well. Combined with the
stagger, this means **in practice only the lowest-slot node in range actually transmits**: the
others hear that relay, drop their copy, and the frame reaches the far node exactly once.
`cancelRepeatOnAck` does the same when an ACK proves the target already has it — matched on
`cmd` + `target_id` rather than on the text, because the ACK is not the frame.

**Two boards can share a slot**, and with three Tops and five slots that is not unlikely. Not fatal:
both relay at once, that relay is lost to the collision, and the sender's next retransmit produces a
fresh attempt with newly drawn jitter. The stagger removes the *guaranteed* collision the old
synchronous design had; it does not have to remove every possible one.

**A wedged radio does not disable the repeater.** If `sendLora()`'s inter-packet guard keeps
refusing, the slot stays busy and is retried next pass rather than being dropped — the old
fire-and-forget call lost the relay on every guarded return. After `REPEAT_EXPIRY_MS` the relay is
stale enough to be useless and the slot is freed, which is also what stops one stuck transceiver
silently holding every slot.

### 3. WiFi, Web Dashboard and OTA (`topwifi.cpp`)
*   **Priority list**: `NicE_WiFi` → `Robo_WiFi` → its own `TOP_<id>` AP. No scanning — the SSIDs are tried in order with a blind `WiFi.begin()`, because `WiFi.scanNetworks()` blocks for seconds and drags the single radio off the AP channel.
*   **Passwords**: `NicE_WiFi` is `!Ni1001100110`; `Robo_WiFi` and the fallback AP are **`geenanker`**.
*   **The fallback AP keeps hunting.** It runs in `WIFI_AP_STA` on `192.168.4.1` (its own gateway, 8 client slots) and continues looking for a real network underneath — but it will **not** tear itself down while a client is connected.
*   **Captive portal** while in AP mode: wildcard DNS on port 53, so joining `TOP_<id>` opens the dashboard by itself. Disabled as a station, where a 404 must remain a 404.
*   **mDNS**: reachable as `top-<id>.local` in both modes. Only `ArduinoOTA.begin()` may initialise the responder — a second `MDNS.begin()` has crashed this hardware.
*   **HTML5 Dashboard**: served from SPIFFS and streamed straight from the file, not cached in a RAM `String` (a fragmented heap silently truncated the page mid-`<script>`). The page **polls `/data`**; there is no WebSocket on the Top.
*   **Endpoints**: `/` (dashboard), `/data` (JSON telemetry), `/command` (actions).
*   **Dual-rate transmit scheduler**: UDP telemetry every **250 ms** for fluid web feedback; LoRa throttled to **5 s** with randomised jitter for collision avoidance, battery and duty-cycle reasons.

### 4. Single-Wire Half-Duplex Serial Bridge (`sercom.cpp`)
*   **Physical layer**: **one shared wire**, half duplex, with hardware separating the directions (opto-isolated). The firmware names two GPIOs per board — Top **TX 22 / RX 21**, Sub **RX 5 / TX 18** — but that is not two conductors.
    > **Diagnostic consequence:** a bad connector or wire stops traffic in *both* directions. A failure in **one direction only** is therefore in the direction-separation circuitry, which is per board — swap the Sub between hulls to localise it.
*   **Echo filtering**: a transmitter hears itself on a shared wire, so frames whose sender ID is our own are discarded.
*   **Implicit ACKs**: any valid `INF` from the Sub clears the retry buffer.
*   **Watchdog**: `isSerConnected` clears after 2 s of silence and a fresh `SETUPDATA` request is issued when the Sub returns.
*   **The Sub sleeps** after reconnection and only wakes on an incoming command — send one, wait, then it talks.

### 5. Who owns the tuning
The **Sub** owns it. PID gains, `compassOffset`, `holdRad`, `revBB`/`revSB`/`swap_BB_SB` live in the Sub's NVS; the Top keeps a RAM copy fetched with `SETUPDATA` when the serial link comes up. `/data` reports `rev` (the count of replies received) and `SubOk` (serial alive) — `rev: 0` with real values elsewhere means the Top has never heard back and is running on zeroed gains.

**The mirror is re-read every `SUB_SETUP_RESYNC_MS` (20 s), not only on connect.** Anything that
changes a setting without going through a Setup dialog — `SET_AS_NORTH`, the Sub's own web page,
an in-field calibration — used to leave this Top serving a stale value indefinitely; measured on the
bench, a Sub on `-43.30` against a Top still reporting `-53.00` minutes later. Re-asking the owner
is the only thing that covers all of them, and it is self-healing.

> **Consequence for anything that reads `/data` or the broadcast live.** The reply goes out to
> `BUOYIDALL` on **both** transports, asked for or not. That is safe for the web Setup dialog, which
> fills its fields once on open, and it was written with that in mind. It is *not* safe for a client
> that edits a live-updating copy of the same struct: the CYD's Setup screen did exactly that, and
> every value dialled in was replaced by the buoy's own a few seconds later. The fix belongs at the
> reading end — see the RoboCYD README — but the behaviour to design against is here.

### 6. Automated Regatta Start Line & Track Calculations
To facilitate competitive sailing regattas, RoboTop coordinates the positions of multiple buoys to automatically establish a fair, geometrically synchronized starting line:

*   **Intelligent Triangulation & Role Determination (`calcTrackPos`)**: Rather than relying on static or arrival-order database slots, RoboTop uses geometric triangulation and wind vectors to determine the roles of the three buoys (`PORT`, `STARBOARD`, `HEAD`):
    *   **Starting Line Identification**: Computes the Great-Circle distances between all three buoy pairs. The two buoys with the **shortest distance** between them are the starting line pins; the furthest is the upwind **HEAD** buoy.
    *   **Port vs. Starboard Allocation**: Calculates the bearing between the two pins and measures the signed smallest angular difference against $W_{\text{dir}}$. Positive ($\ge 0^\circ$) makes the first buoy **PORT**; negative makes it **STARBOARD**.
*   **$W_{\text{dir}}$ is the mean of BOTH pins' wind, not one buoy's (`meanWindDir`)**: it used to
    be `rsl[0].wDir` — whichever buoy the command happened to reach. Two anemometers a line's length
    apart disagree by a few degrees in steady air and by rather more in a shifty one, and there is
    no reason to prefer either; the mean squares the line to the wind *across* the line, which is
    the wind the fleet actually starts in. In a three-buoy course the upwind **HEAD** mark is left
    out of the average on purpose — it is no part of the line, and it is the furthest from where the
    start happens. All three consumers now use it: `recalcStartLine()`, `reCalcTrack()` (which hangs
    the head mark's bearing, the offset back down to the line and the starboard side off this one
    figure) and `calcTrackPos()`. Slot 0 remains the fallback, because `handleStatus()` puts this
    buoy's filtered wind there and the compute guards have already refused to run without it.
*   **Geographical Midpoint & Width**: Computes the pins' midpoint by coordinate average and the line width $d$ by Haversine.
*   **Wind-Aligned Perpendicular Squaring (`recalcStartLine`)**: The line must be exactly perpendicular to the wind:
    *   Port end bearing: $\theta_{\text{Port}} = (W_{\text{dir}} + 270^\circ) \bmod 360^\circ$
    *   Starboard end bearing: $\theta_{\text{Starboard}} = (W_{\text{dir}} + 90^\circ) \bmod 360^\circ$
*   **Vector Position Projection (`adjustPositionDirDist`)**: Projects new targets outward from the midpoint along those bearings by $d/2$, squaring the line to the wind while keeping midpoint and length intact. *Length intact* now holds exactly: this function placed with an earth radius of 6371000 m while `distanceBetween()` measured with 6372795 m, so measuring $d$ and then placing at $d$ returned $1.000282\,d$ and every press of ALIGN STARTLINE grew the line by 0.028%. Both use `EARTH_MEAN_RADIUS`. It was 9 mm on a 31 m line and never the reason a line looked a different length — but the whole design of `recalcStartLine()` is that measuring and placing are inverses, and they were not.
*   **The line is computed from the lock TARGETS, not the hulls.** The screens draw it between the
    actual positions, because that is what the crew can see; the arithmetic deliberately does not.
    Feed it hull positions and each press picks up wherever the buoys were wandering inside their
    hold radius, so the line changes length every time it is aligned. Targets are fixed, so the
    length is preserved. You see the hulls, the maths uses the targets.
*   **Asynchronous Coordination Broadcast (`SENDTRACK`)**: Dispatches updated coordinates to Port, Starboard and Head over LoRa.

### 7. Advanced Circular Wind Telemetry & Standard Deviation
*   **Circular Vector Addition (`averageWindVector`)**: Arithmetic averaging fails at the $360^\circ$ wrap (the mean of $350^\circ$ and $10^\circ$ is $180^\circ$ arithmetically but $0^\circ$ physically). Each sample $A_i$ is decomposed into unit vectors:
    $$x_i = \cos(A_i \times \tfrac{\pi}{180}), \quad y_i = \sin(A_i \times \tfrac{\pi}{180})$$
    and recombined with the four-quadrant arctangent:
    $$W_{\text{dir}} = \text{atan2}(\bar{y}, \bar{x}) \times \tfrac{180}{\pi}$$
*   **Yamartino Circular Standard Deviation (`deviationWindRose`)**: from the mean resultant length $R$
    $$R = \frac{\sqrt{(\sum \cos A_i)^2 + (\sum \sin A_i)^2}}{N}, \qquad W_{\text{std}} = \sqrt{-2 \ln R} \times \tfrac{180}{\pi}$$
    $R = 1$ is perfect alignment, $R = 0$ complete dispersal.

### 8. Command Routing Rules (`handleRfData`)
*   `IDr` is matched against our MAC; `1` (`BUOYIDALL`) and `0` are broadcasts.
*   A **broadcast command is only obeyed when the sender is `0x99` (web/RoboLora) or `0x98` (CYD)**. A broadcast from a peer buoy is ignored — this is what stops one buoy commanding another.
*   Frames that are not for us are **bridged to the other interface** (UDP↔LoRa), never back out the one they arrived on, which would loop.
*   `DOCKPOS` is a *question*. The answer is addressed to the asker and sent as `INF`, and an `INF` frame is never answered — otherwise two Tops answer each other's answers indefinitely.

---

## 🩺 Diagnostics

*   **Debug channel** — plain text broadcast on **UDP 1002**, silent at idle. Carries every non-telemetry RF and serial frame, the DOCK/DOCKPOS step markers and every beep with the line that requested it.
*   **Breadcrumbs surviving a panic** — held in `RTC_NOINIT` memory and reported next boot as `Crumb`, `CrumbLora`, `CrumbWifi`, `CrumbSer`. `90–96` = loop phase, `1000+cmd` = RF command dispatched, `2000+cmd` = serial command, `200–206`/`300–303`/`400–403` = LoRa/Sercom/WiFi task.
    > A **power cycle wipes RTC RAM** and the evidence with it. A reset-pin press or a panic reboot preserves it.
*   **`/data` health fields** — `Uptime`, `ResetReason`, `SubOk`, `rev`, the four `Stk*` figures and `HeapFree` / `HeapMin` / `HeapMaxBlk`.

---

## 🛠️ Building & Flashing

PlatformIO environment **`robo-esp`** (`board = esp32dev`). Dependencies: `RoboCompute`, `RoboTone`, Arduino-PID-Library, TinyGPSPlus, FastLED, arduino-LoRa, ArduinoOTA.

```bash
pio run                                              # build
pio run -t upload                                    # OTA, default 192.168.1.71
pio run -t upload --upload-port top-b7a5b578.local   # a different unit
pio run -t uploadfs                                  # required after any data/ change
```

`upload_protocol = espota`, so a plain upload goes over the network — the Tops are not normally on USB.
