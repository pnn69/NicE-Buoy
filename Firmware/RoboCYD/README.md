# RoboCYD — Touchscreen Fleet Controller and Field Access Point

The **`RoboCYD`** firmware runs on a "Cheap Yellow Display" board: an ESP32 with a 320×240 TFT and a resistive touch panel. It is the handheld controller for the fleet — it shows every buoy, sends lock/dock/idle commands, and serves a web dashboard and a fullscreen map.

It also has a second job that nothing else does: **out on the water, the CYD *is* the network.**

---

## 🌐 The CYD is the field access point

Every node follows the same priority list, but the CYD sits at the top of it:

| Device | Priority list | In the field |
|---|---|---|
| **RoboCYD** | `NicE_WiFi` → **becomes `Robo_WiFi`** | the field AP |
| RoboTop | `NicE_WiFi` → `Robo_WiFi` → `TOP_<id>` | joins the CYD |
| RoboSub | `NicE_WiFi` → `SUB_<id>` | always its own AP |
| RoboLora | `NicE_WiFi` → `Robo_WiFi` → `LORA_<id>` | joins the CYD |

*   At home the CYD joins `NicE_WiFi` like everything else.
*   When no `NicE_WiFi` is in reach it raises **`Robo_WiFi`**, password **`geenanker`**, on
    **`192.168.1.1/24`** — the same subnet as home, so the field network and the living room look
    identical from a browser. The AP is its own gateway; advertising a gateway that never answers
    makes Android decide the network is broken and drop it.
*   **8 client slots.** The Arduino default of 4 is exactly three Tops plus a phone, leaving nothing
    for a laptop. The ESP32 silicon allows 15.
*   **The Sub deliberately does not join**, keeping those slots for the Tops and a phone.

### It decides once, at boot
Migrating back to home mid-session would tear `Robo_WiFi` out from under every Top at once. So the
CYD re-checks for home **only while nobody is connected to it** — the "carried everything indoors"
case. With a Top attached it stays put.

### Getting in without an IP
*   **Captive portal** while it is the AP: wildcard DNS on port 53 means a phone's own connectivity
    probe lands on the dashboard and the "sign in to network" sheet opens it unprompted. The 302 is
    aimed at the real softAP address, and is **off** when the CYD is a station — there, a 404 must
    stay a 404.
*   **mDNS**: `robocyd.local`, in both modes. Only `ArduinoOTA.begin()` may initialise the
    responder; a second `MDNS.begin()`, or an `MDNS.end()` while OTA holds it, has crashed this
    hardware. Add records with `MDNS.addService()` on top of the one OTA created.

---

## ⚡ Core modules

### `cyd_lora.cpp` — radio on its own task
The radio is drained by a **dedicated FreeRTOS task**, not from `loop()`.

`loop()` also runs OTA, the web server and every screen repaint, and a full redraw pushes enough
pixels over SPI to take far longer than one packet is on the air. The Tops send `BUOYPOS`
immediately followed by `TOPDATA`, so the radio must be read twice inside ~130 ms — and the draw
loop was not coming back that fast. The second frame of every pair was overwritten before it was
ever read: a buoy sending them one for one showed up here as **12 `BUOYPOS` and 0 `TOPDATA` per
minute**.

The task does nothing but move bytes off the radio into a queue. Parsing stays on the main loop, so
`buoys[]` keeps a single writer and the Arduino `String`s inside it are never reallocated under the
draw code. A **mutex guards every SX127x access**, because the task polls continuously while
`send_lora_packet()` is called from the touch handlers on the other core, sharing one SPI bus and
one radio.

### `buoy_data.cpp` / `.h` — fleet state
Holds up to three buoys: identity, position, heading, battery, status, RSSI and link freshness.

Buoys also carry **the waypoint they were told to hold**, taken from `LOCKPOS`/`DOCKPOS` (cmd 21/23)
— the only frames that carry it. `TOPDATA` and `BUOYPOS` say where a buoy *is*, never where it is
meant to be. It ages out on a timestamp (`buoy_has_waypoint()`), because a Top stops beaconing the
moment it leaves `LOCKED`/`DOCKED`, and a stale mark on the plot is worse than no mark.

**Only real telemetry claims a slot.** There are three, and `LORA_LINK` (94) is answered and
returned from before the slot search ever runs. A link report says what a node *hears*; it does not
say the node is a buoy. RoboLora reports its links like everybody else, and while every sender was
eligible the gateway walked into a slot and sat there — a row in the menu with no heading, no
position and nothing to steer, and one fewer slot for something that has all three. Nothing is lost
by refusing it: a node that is also a buoy is registered by its own `TOPDATA`/`BUOYPOS` anyway.

**The Setup screen owns its fields while it is open.** `buoys[]` is not only the fleet model, it is
also the Setup screen's edit buffer — there is no separate copy — so an incoming `SETUPDATA` lands
directly on unsaved work. And one arrives constantly: RoboTop re-reads the Sub's settings every
`SUB_SETUP_RESYNC_MS` (20 s) and broadcasts the answer to `BUOYIDALL` on both transports, asked for
or not. That poll is right in itself, and it was written against the *web* dialog, which fills its
fields once on open and is genuinely safe from it. This screen reads `buoys[]` live, so it was not:
values dialled in on `+`/`-` reverted a few seconds later, and Set as North's computed offset was
replaced by the buoy's old one before SAVE could send it. Once the screen has its initial load the
parser leaves those fields alone until it is left; entering Setup clears `setup_data_loaded` and
re-asks, so they are still fresh every time it opens. `setup_await_refresh` lets exactly one reply
through afterwards, for the case below. Telemetry is untouched — it is not edited, and the screen
needs it live.

### `main.cpp` — screen and menu
*   Menu buttons are laid out to the button edges rather than centred, so a full
    `Buoy n: b7a5b578` fits beside a one-character status badge.
*   A button is repainted **only when the ID, presence, IP or status changes**. RSSI is deliberately
    kept out of that cache key: it moves with every packet and would repaint the whole 40 px button
    at packet rate, which is the flicker the caching exists to prevent. It gets its own small
    repaint.
*   The status badge is `I` idle, `L` locked, `D` docked. `LOCKING`/`DOCKING` report the letter they
    are heading for but are flagged unsettled — a buoy that cannot get a GPS fix sits in `LOCKING`
    indefinitely, and a plain `L` there would claim it is holding station when it has not started.
*   A waypoint further from the fleet than a sane course mark is **pegged to the rim** rather than
    stretching the plot until the buoys are unreadable.

### Setup actions the **buoy** performs

Set as North used to be arithmetic done here: `compass_offset - mag_dir` written into `buoys[]`,
followed by a jump to the compass page and a wait for SAVE. It sent nothing. That failed three ways
at once — the value was overwritten by the resync described above, it needed a SAVE the operator had
to know to press, and the page jump is what "it jumps to another screen" meant. Underneath all of it
the formula was the superseded one: the Sub retired `compassOffset - dirMag` because it took three
or four presses to creep onto north, which is exactly what pressing the button repeatedly looks
like.

It now sends **`SET_AS_NORTH` (87)** — the same command the Top's web page sends. The Sub solves it
with `computeSetAsNorthOffset()`, which reads the heading out of the pipeline *after* the compass
table and the trim and lands on north in one press, commits it to its own NVS and beeps. RoboTop
re-reads the setup immediately (`requestSubSetup("after SET_AS_NORTH")`) and that reply is the one
`setup_await_refresh` admits, so the new offset appears on the compass page by itself. Nothing to
save, no page jump, and one implementation of the solve instead of two.

Set as Level (`SET_AS_LEVEL`, 93) already worked this way and is unchanged. The rule for this page
is now uniform: **if the buoy owns the value, the buoy computes it.**

### Wind is both ends of the line

Port and starboard used to be decided by `pick_wind_reference_buoy()` — the first buoy in the list
with a non-zero reading, i.e. whichever one happened to be there. Two anemometers a line's length
apart disagree by a few degrees in steady air and by rather more in a shifty one, and there is no
reason to prefer either.

`start_line_wind()` returns the **mean of what the two line-end buoys report**, found via
`find_start_line_pair()` — the closest pair, measured where they actually are. With three buoys the
upwind mark is left out: it is no part of the line. `mean_wind_dir()` is a vector mean, not an
arithmetic one, or 350° and 10° would average to 180° instead of 0°. A buoy reading 0/0 has no
reading rather than a northerly, so it drops out and the other end is used alone.

The wind **label** now comes from the same place as the geometry. Showing one buoy's reading beside
a line squared to a different figure is how an operator stops trusting both. The spread printed next
to it is the *worse* of the two, not their mean — it is a confidence figure, and two readings do not
make a shifty wind steady.

The same solve exists in three places on purpose, and they have to stay in step: `meanWindDir()` in
RoboCompute for the squaring the Top actually performs, `mean_wind_dir()` here, and
`mapMeanWindDir()` in `index.js`.

### `cyd_wifi.cpp` — web, WebSocket and UDP
*   Serves `index.html`, `index.js`, `style.css` from SPIFFS, plus a **standalone fullscreen map**
    in its own tab with an independent redraw loop and a canvas sized to the window.
*   **WebSocket on port 81** relays everything the CYD hears to the browser, prefixed `LORA:` or
    `UDP:` so radio and network traffic stay in separate consoles. This is also the easiest way to
    watch LoRa traffic from a PC — the CYD is the only node that rebroadcasts it.
*   **UDP 1001** for protocol frames.
*   Commands typed by the browser are forwarded **verbatim** to both LoRa and UDP; the CYD does not
    stamp IDs. The web UI builds the whole frame, addressing it to a specific buoy with sender `99`
    (hex `0x99`).

#### The web map had to be taught what the screen already knew

The dashboard map and the touchscreen map disagreed, and the touchscreen was right. Three reasons,
all now fixed in `index.js`:

*   **It never read the waypoint.** `LOCKPOS`/`DOCKPOS` are the only frames carrying a set point as
    coordinates, and the page had no handler for either despite defining the enum. It reconstructed
    each waypoint by projecting `Target Dir`/`Target Dist` off the buoy's own fix — and that
    reconstruction is refused when the reported distance is 0, which is exactly what a buoy that has
    *arrived* reports. So the moment the fleet settled on station every waypoint vanished and the
    map fell back to measuring between two hulls wandering inside their hold radius. It now records
    the real coordinates and ages them out on the same 20 s window the firmware uses, deliberately
    the same figure so the two views drop a stale mark together. Projection survives only as a
    fallback before a waypoint frame has been heard.
*   **The line is drawn between the hulls.** That is what the crew can see from the water, and what
    the touchscreen always drew. The *figure* beside it is still measured off the two waypoints
    where both ends have one — the dots wander, the waypoints do not, and EXECUTE has to be able to
    put the line back where it was but longer. Without both waypoints the length still shows and
    `-`/`+`/EXECUTE grey out rather than firing on a target that cannot be computed.
*   **The gateway took a panel.** Same defect as in the firmware, same fix: only `TOPDATA` and
    `BUOYPOS` prove a sender is a buoy. RoboLora has no fix so it never reached the map itself, but
    it did occupy one of three slots.

#### No confirmation dialogs

All five `confirm()` popups are gone. A modal in the middle of a field job is the worst place for
one, and each was guarding something that did not need it: EXECUTE is already the deliberate second
step after dialling a length in and stays greyed out until the figure differs; re-capturing a
calibration leg writes nothing until CLOSE and is the normal way to *fix* a bad reading, so the
dialog blocked the recovery rather than the mistake; discarding a run loses nothing, as the dialog
itself said; the desk calibration's instructions belong on the page beside the button, not behind a
dismissal; and a reboot loses nothing because every setting is in NVS. What happened is still
reported — after the fact, in the map message and the log, rather than in the way.

> A caveat worth knowing: `index.js`'s `handleJsonFallback()` contains
> `b.id = b.id || "0001"`, and `"0001"` parses as hex `1` — which **is** `BUOYIDALL`. A command sent
> to a buoy carrying that fallback ID would go to the whole fleet. The path is currently dormant
> (it needs a non-`$` JSON message the CYD never sends), but `"0001"` is a poor sentinel for
> "unknown buoy".

### `boot_screen.cpp` — splash
Draws `data/boot_logo.565` at startup. The logo is generated by `tools/make_boot_logo.py`; the LVGL
reference it was derived from is under `Doc/robobuoy_startup_lvgl_reference/`.

### `cyd_touch.cpp`, `cyd_display.cpp`, `RGBledDriver.cpp`
Resistive touch (XPT2046), TFT_eSPI display setup, and the onboard RGB LED.

---

## 🛠️ Building & Flashing

Environment **`cyd`** (`board = esp32dev`, `platform = espressif32@6.5.0`). Note
`src_dir = .` with `build_src_filter = +<src/>`. Libraries: `TFT_eSPI`, `XPT2046_Bitbang_Slim`,
`lvgl`, `LoRa`, `WebSockets`.

```bash
pio run                  # build
pio run -t upload        # serial upload, upload_port = COM15
pio run -t uploadfs      # required after any data/ change
```

**There is no espota configuration**, and `COM15` may not exist on the machine you are using. The
CYD does run ArduinoOTA, so flash it over the network directly:

```bash
python ~/.platformio/packages/framework-arduinoespressif32/tools/espota.py \
       -i <ip> -p 3232 -f .pio/build/cyd/firmware.bin -r
```

`monitor_filters = esp32_exception_decoder` is already set, so a panic on this board prints a
decoded backtrace with function names — worth remembering, because the CYD is the one node normally
reachable over USB.
