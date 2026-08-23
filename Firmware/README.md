# NicE-Buoy Firmware

This directory holds every firmware image in the system. For the system-level picture — architecture, the frame format, the protocol rules and the calibration state machines — see the [repository README](../README.md). This file is the firmware index: what each project is, how they are built and flashed, and the conventions they share.

> These two files used to be near-identical copies, which is how they drifted apart. Keep system-wide material in the root README and firmware-specific material here.

---

## Projects

| Directory | Target | Role | README |
|---|---|---|---|
| `RoboTop` | ESP32 (`esp32dev`) | Buoy top: GPS, LoRa, WiFi, web dashboard, commands the Sub | [link](RoboTop/README.md) |
| `RoboSub` | ESP32 (`esp32dev`) | Buoy sub: thrusters, ICM-20948 compass, PID, owns all tuning | [link](RoboSub/README.md) |
| `RoboCYD` | ESP32 + 320x240 TFT | Touchscreen controller; **is** the field AP `Robo_WiFi` | [link](RoboCYD/README.md) |
| `RoboLora` | ESP32 (`pico32`) | USB/WiFi to LoRa gateway with web UI | [link](RoboLora/README.md) |
| `RoboDependency` | library | `RoboCompute` (maths, `RoboStruct`, codec) and `RoboTone` | [link](RoboDependency/README.md) |
| `RoboChargingStation` | ESP32 | Solar monitoring and charging dock | [link](RoboChargingStation/README.md) |
| `RoboPythonDisplay` | Python | Tkinter desktop dashboard | [link](RoboPythonDisplay/README.md) |

`RoboTop` and `RoboSub` both link `RoboDependency`; change `RoboCompute` and **both** must be rebuilt and reflashed.

---

## 📶 WiFi policy (all devices)

One priority list, identical everywhere:

```
NicE_WiFi  ->  Robo_WiFi  ->  own AP
```

| Device | Priority list | In the field |
|---|---|---|
| RoboCYD | `NicE_WiFi` → *becomes* `Robo_WiFi` | the field AP |
| RoboTop | `NicE_WiFi` → `Robo_WiFi` → `TOP_<id>` | joins the CYD |
| RoboSub | `NicE_WiFi` → `SUB_<id>` | always its own AP |
| RoboLora | `NicE_WiFi` → `Robo_WiFi` → `LORA_<id>` | joins the CYD |

*   `NicE_WiFi` password `!Ni1001100110`; `Robo_WiFi` and every fallback AP use **`geenanker`**.
*   `Robo_WiFi` is `192.168.1.1/24` with **8 client slots** (the Arduino default of 4 is exactly three Tops plus a phone). Per-node fallback APs are `192.168.4.1`, each its own gateway.
*   **The Sub skips `Robo_WiFi` on purpose**, to keep the CYD's slots for the Tops and a phone. Nothing is lost — the Sub never transmits on UDP; its telemetry reaches you inside the Top's `TOPDATA`.
*   Nodes **never stop hunting**: the fallback AP runs in `WIFI_AP_STA`, so a buoy on its own AP keeps looking for a real network underneath it. It will not tear its AP down while a client is connected.
*   No scanning. `WiFi.scanNetworks()` blocks for seconds and drags the single radio off the AP channel; the SSIDs are tried in order with a blind `WiFi.begin()` instead.

**Getting in without an IP address** — two mechanisms, because neither covers both modes:

*   **Captive portal** while the node is an AP: wildcard DNS on port 53 means the phone's own connectivity probe lands on the dashboard. Off when the node is a station, where a 404 must stay a 404.
*   **mDNS** in both modes: `top-<id>.local`, `sub-<id>.local`, `robocyd.local`, `lora-<id>.local`.

> Only `ArduinoOTA.begin()` may initialise mDNS. A second `MDNS.begin()` — or an `MDNS.end()` while OTA holds the responder — has crashed this hardware before. Add records with `MDNS.addService()` on top of the responder OTA created.

> Changing an SSID or the policy must be rolled out to the **whole fleet at home on `NicE_WiFi`**. A device left on the old scheme is unreachable in the field.

---

## 🔌 UDP ports

| Port | Traffic |
|---|---|
| `1001` | protocol frames (`$...*CRC`) — telemetry and commands |
| `1002` | plain-text debug log from Tops and Subs, silent at idle |

Listening to the debug channel is the quickest way to see what a buoy is doing:

```bash
python -c "import socket; s=socket.socket(2,2); s.bind(('',1002));
while 1: print(s.recvfrom(2048)[0].decode('utf-8','replace'), end='')"
```

---

## 🛠️ Build and flash

```bash
pio run                      # build
pio run -t upload            # upload
pio device monitor           # serial monitor
```

| Project | Default upload | Notes |
|---|---|---|
| `RoboTop` | OTA (`espota`, `192.168.1.71`) | a `data/` change also needs `pio run -t uploadfs` |
| `RoboSub` | OTA (`espota`, `192.168.1.189`) | sealed hull, never on USB in practice |
| `RoboCYD` | serial (`COM15`) | **no espota config** — see below |
| `RoboLora` | serial (`COM30`) | **no espota config** — see below |

Override the target per invocation:

```bash
pio run -t upload --upload-port 192.168.1.78
pio run -t upload --upload-port top-b7a5b578.local
```

`RoboCYD` and `RoboLora` name COM ports that may not exist on the machine you are using, but both run ArduinoOTA, so flash them directly:

```bash
python ~/.platformio/packages/framework-arduinoespressif32/tools/espota.py \
       -i <ip> -p 3232 -f .pio/build/<env>/firmware.bin -r
```

Environments: `RoboTop` → `robo-esp`, `RoboSub` → `robo-esp-v3`, `RoboCYD` → `cyd`, `RoboLora` → `pico32`.

---

## Conventions worth knowing before editing

*   **The Sub owns the tuning.** PID gains, `compassOffset`, `holdRad`, `revBB`/`revSB`/`swap_BB_SB` live in the Sub's NVS. The Top keeps a RAM copy fetched via `SETUPDATA`. `compassOffset` and the thruster flags are **hull-specific** and do not belong to the board they are stored on.
*   **The Sub has no LoRa.** Its only link to the outside is the serial line to its Top.
*   **A broadcast must never ask for an ACK.** `IDr = BUOYIDALL` with `ack = SET`/`GETACK` can never be matched in `removeAckMsg()` and jams a LoRa retry slot for five retransmits. Use `INF` for beacons.
*   **Only `0x99` (web/RoboLora) and `0x98` (CYD) may command by broadcast.** A buoy ignores broadcast commands from a peer buoy — that is what stops one buoy driving another.
*   **Watch task stack headroom.** The Top's `/data` exposes `StkLoop`, `StkLora`, `StkWifi`, `StkSer`, `HeapFree`, `HeapMin`, `HeapMaxBlk`. A LoRa task left with 100 bytes of stack caused a panic that looked like everything except what it was.
