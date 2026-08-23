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
