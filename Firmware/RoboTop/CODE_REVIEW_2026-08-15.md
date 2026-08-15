# Code Review — 2026-08-15

**Scope:** uncommitted working-tree diff (`git diff HEAD`) at commit `6c847d32`.

**Files reviewed:**
- `Firmware/RoboTop/src/main.cpp`
- `Firmware/RoboTop/src/topwifi.cpp`
- `Firmware/RoboTop/data/index.html`
- `Firmware/RoboTop/platformio.ini`
- `Firmware/RoboDependency/RoboCompute/src/RoboCompute.cpp`
- `Firmware/RoboTop/restore_robocompute.py`
- `Firmware/RoboTop/restore_robocompute_clean_final.py`

**Status:** no code was changed. 15 findings below, ranked by severity.

> Findings 1–4 are one cluster: the rewritten routing block in `main.cpp` around
> lines 1071–1084 lost both its `is_for_me` broadcast branch and its sender-authority
> guards. That is the place to start.

---

## High

### 1. `src/main.cpp:1084` — bridging direction inverted, causes an infinite broadcast storm

`if (!is_for_me) { if (from_web) loraOut else udpOut }` sends a packet that
*arrived over UDP* (any packet whose `IDs` is not 0x98/0x99, i.e. all
buoy-to-buoy traffic) straight back out on `udpOut`, which `WiFiTask` turns into
`udp.broadcast()`.

Two concrete failures:

- **WiFi→LoRa bridging is gone** — a LoRa-only buoy never sees UDP-originated
  directed traffic.
- **Broadcast storm** — the sender ID is not rewritten, so `udp_setup`'s
  `udpDataIn.IDs != espMac()` self-filter never trips. Buoy X rebroadcasts a
  packet addressed to B, buoy Y receives it (still `IDs` = original sender),
  rebroadcasts it, X receives it again, forever. Commands excluded from the
  dedup filter (`REMOTE`, `DIRDIST`, `SETUPDATA`, `LOCKING`, `DOCKING`,
  `IDELING`) have nothing to stop the loop.

---

## Medium

### 2. `src/main.cpp:1071` — broadcasts no longer execute locally

`from_web` packets are only "for me" on an exact MAC/ID match, so
`IDr == BUOYIDALL` and the legacy `IDr == 0` broadcast are no longer executed
locally (the old `else if (from_udp && broadcast)` local+forward path was
deleted).

A broadcast command from the remote/screen (`IDs` = 0x98) or from a web client
over UDP is now silently bridged to LoRa and ignored by every WiFi-attached
buoy, while LoRa-attached buoys still execute it — the fleet ends up in
inconsistent states. `RoboLora/src/main.cpp:162` leaves `IDr = 0` until a buoy
is selected, which lands exactly in this hole.

### 3. `src/main.cpp:1274` — broadcast SETUPDATA writes every buoy's EEPROM

The `if (RfIn.IDr == RfOut->mac)` guard around the "forward SETUPDATA to Sub"
block was removed. A broadcast SETUPDATA (`IDr == BUOYIDALL`, which
`topwifi.cpp:509` generates whenever the target buoy's `IDs` is still 0) is now
pushed into *every* listening buoy's Sub over `serOut` with `ack = SET`, causing
each Sub to commit another buoy's PID / max-speed / thruster config to its
EEPROM.

### 4. `src/main.cpp:1228` — remote buoys can overwrite our config

The `is_local` gate (`IDs == mac || 0x98 || 0x99 || from_udp`) was dropped, so a
SETUPDATA with `ack` 1/2/3 arriving over LoRa from *another buoy's* MAC and
addressed to us (or broadcast) now runs `pidRudderParameters` /
`pidSpeedParameters` / `CompasOffset` / `thrusterSwap` / `thrusterInversion`
with `SET` and overwrites our persisted configuration with the remote buoy's
values.

### 5. `RoboCompute/src/RoboCompute.cpp:423` — encoder/decoder asymmetry

`RoboCode` gained `case DOCKING:` / `case LOCKING:` (5 payload fields) and
`case IDELING:` (line 436), but `RoboDecode` has no matching cases — the
receiver falls into `default:`, discards `tgDir` / `tgDist` / `tgSpeed` /
`wDir` / `wStd` and prints `RoboDecode: Unknown CMD %d` for every such packet.

### 6. `RoboCompute/src/RoboCompute.cpp:235` — `ADAPTIVE_TRIM` deleted but still live

`ADAPTIVE_TRIM` was removed from both `RoboDecode` (line 235) and `RoboCode`
(line 419), but the feature is still wired end to end:
`data/index.html:486,507` → `topwifi.cpp:498/552` → `main.cpp:1499`, with the
Sub's reply consumed at `main.cpp:1670`.

Pressing "Toggle Trim" / "Clear" now emits a packet with no `compass_trim` /
`compass_trim_enabled` payload, and the Sub's trim report is never parsed, so
`compass_trim` stays stale forever.

### 7. `RoboCompute/src/RoboCompute.cpp:250` — dock-approach fields dropped from the wire

`RoboCode` / `RoboDecode` for SETUPDATA (and BUOYPOS at lines 134/313) no longer
carry `compass_trim_enabled`, `dockApproachDist`, `dockApproachDir`,
`dockingToWaypoint`, while `topwifi.cpp:545-548` still fills them from the web
form and `main.cpp:1226/1249` still logs and stores them.

Setting dock-approach parameters for a remote buoy over LoRa/UDP now silently
loses them. Worse: `RoboDecode` leaves those struct fields untouched, so the
remote applies whatever was in its receive struct and calls
`memDockApproach(..., SET)` with stale values.

### 8. `RoboCompute/src/RoboCompute.cpp:770` — wrong wind index in `recalcStartLine`

In the third branch the start-line rotation uses `rsl[1].wDir`, while the other
two branches use `rsl[0].wDir` and the only caller (`main.cpp:559`,
`buoyPara[0].wDir = stat->wDir`) sets the wind direction on index 0 only.

When buoys 1 and 2 are the closest pair and buoy 1 has no wind telemetry, the
line is laid out relative to 0° (north) instead of the real wind, placing both
marks in the wrong position.

---

## Low

### 9. `RoboCompute/src/RoboCompute.cpp:237` — `case NOP:` removed

`NOP` is 0, and `formatFloat` / the zero-compression pass turns zero tokens into
empty strings, so any message whose first field is empty also decodes as cmd 0.
Each one now prints `RoboDecode: Unknown CMD 0`, spamming the serial console at
telemetry rate.

### 10. `src/topwifi.cpp:388` — `TgLat` / `TgLng` always zero for remote buoys

`AddDataToBuoyBase` does `*buoyPara[i] = dataIn` with a zero-initialised struct
filled by `rfDeCode`, and neither BUOYPOS nor TOPDATA encodes `tgLat` / `tgLng`.
The new waypoint markers on the radar map can therefore only ever appear for
buoy 1, and every telemetry packet also wipes any previously known remote
target.

### 11. `data/index.html:782` — radar map centre vs. bounds mismatch

`centerX` / `centerY` are still the centroid of the *buoys only*, but
`minX/maxX/minY/maxY` now include the target waypoints. With
`scale = 0.7 * min(w,h) / maxDim`, a waypoint far from the buoy centroid is
offset by up to `maxDim/2` after scaling and is drawn at or beyond the canvas
edge.

Fix: use the bbox centre (`(minX+maxX)/2`) now that waypoints participate in the
bounds.

### 12. `data/index.html:986` — stale buoy ID in the header

`updateUI` returns early on `!b || b.ID === "0"` without touching the new
`buoyIdText`. Select buoy 1 (shows "TEST_TOP"), then switch to a disconnected
buoy 2: the header reads `TEST_TOP | Buoy 2 Not Connected`, i.e. the ID of a
different buoy.

### 13. `src/main.cpp:1946` — unthrottled status broadcast can saturate the radio

The new end-of-loop status broadcast fires on every `mainData.status` change
with no rate limit and no coordination with the periodic telemetry timer,
pushing to both `udpOut` and `loraOut`. The loop body has no delay of its own,
so any state that oscillates between two values (e.g. IDELING↔IDLE handling at
`main.cpp:1770-1780`) will fill the 10-deep LoRa queue.

Consider a minimum interval or debounce.

### 14. `restore_robocompute_clean_final.py:1` — divergent embedded copy + CRLF corruption

The two restore scripts embed *different* copies of `RoboCompute.cpp`:
`restore_robocompute.py` matches the current file byte-for-byte, but
`restore_robocompute_clean_final.py` still uses `String(x, n)` instead of
`formatFloat(x, n)` (no zero-trimming, different wire size) and different
`count >` guards. Running the "clean_final" script silently reverts the codec.

Additionally both write with `open(..., newline='\r\n')` while the embedded
content already contains CRLF (966 / 964 CRLF pairs), so every `\n` is
re-translated and the generated file gets `\r\r\n` line endings.

### 15. `platformio.ini:20` — local dev settings staged for commit

`monitor_port` COM41→COM51, and the upload path switched from `espota` /
192.168.1.78 to serial `COM51`. Anyone else building from this tree loses OTA
upload and gets a COM port that does not exist on their machine.
