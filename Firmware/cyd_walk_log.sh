#!/usr/bin/env bash
# Range walk logger.
#
# The handheld is carried away from the buoys. The two Tops stay put and stay pollable, and each
# reports what it HEARS from the CYD over LoRa (sender id 98) in its /data LoraLinks table. That
# gives RSSI and frames-per-minute against distance, measured at the buoy end - the direction that
# decides whether a command lands.
#
# Also sampled: whether the CYD still answers WiFi. The moment that stops while the LoRa count
# keeps ticking is exactly what killed the field session.
#
# AND the relay's own signal at each Top, because the link table cannot be read without it.
#
#   RoboTop's linkNote() files packetRssi against the SENDER ID INSIDE THE FRAME. The repeater
#   relays frames verbatim, so a relayed copy still says "from 98" but arrives with the RELAY's
#   signal strength - and linkFirstCopy() keeps whichever copy lands first, direct or relayed,
#   with no way to tell them apart. A Top that has lost the handheld directly but still hears
#   RoboLora's relay of its beacon therefore records the handheld as strong.
#
#   Caught live at 13:16 on this walk: Top .78 reported the CYD at -59 while Top .71, two metres
#   away, reported -117. -59 is exactly what .78 hears RoboLora at. So the rly columns are the
#   control: when t7x_rssi equals t7x_rly, that sample is the relay, not the handheld.
#
# Full sample set to the CSV; only state CHANGES to stdout.

CSV=/c/tmp/cyd_walk.csv
T71=192.168.1.71
T78=192.168.1.78
CYD=192.168.1.235
RLY=1c3101ac

if [ ! -f "$CSV" ]; then
  echo "iso_time,elapsed_s,cyd_http,t71_rssi,t71_count,t71_age,t71_rly,t78_rssi,t78_count,t78_age,t78_rly,suspect" > "$CSV"
fi

START=$(date +%s)

# One /data fetch per Top per sample, reused for every field - polling it six times was six HTTP
# round trips for one reading, and the values could disagree across them.
fetch() { curl -s -m 5 "http://$1/data" 2>/dev/null; }

peer() {
  # $1 = json, $2 = peer id, $3 = field
  echo "$1" | grep -o "{\"id\":\"$2\"[^}]*}" | grep -o "\"$3\":-\?[0-9]*" | head -1 | cut -d: -f2
}

prev_state=""
while true; do
  NOW=$(date +%s); ELAPSED=$(( NOW - START )); ISO=$(date +%Y-%m-%dT%H:%M:%S)

  HTTP=$(curl -s -m 4 -o /dev/null -w "%{http_code}" "http://$CYD/" 2>/dev/null)
  [ -z "$HTTP" ] && HTTP=000

  J71=$(fetch $T71); J78=$(fetch $T78)

  R71=$(peer "$J71" 98 rssi); C71=$(peer "$J71" 98 count); A71=$(peer "$J71" 98 age)
  L71=$(peer "$J71" $RLY rssi)
  R78=$(peer "$J78" 98 rssi); C78=$(peer "$J78" 98 count); A78=$(peer "$J78" 98 age)
  L78=$(peer "$J78" $RLY rssi)

  : "${R71:=}" "${C71:=}" "${A71:=}" "${L71:=}" "${R78:=}" "${C78:=}" "${A78:=}" "${L78:=}"

  # Mark the sample if either Top's "handheld" reading is indistinguishable from its relay reading.
  SUSPECT=""
  [ -n "$R71" ] && [ "$R71" = "$L71" ] && SUSPECT="${SUSPECT}71"
  [ -n "$R78" ] && [ "$R78" = "$L78" ] && SUSPECT="${SUSPECT}78"
  [ -z "$SUSPECT" ] && SUSPECT=ok

  echo "$ISO,$ELAPSED,$HTTP,$R71,$C71,$A71,$L71,$R78,$C78,$A78,$L78,$SUSPECT" >> "$CSV"

  if [ "$HTTP" = "200" ]; then wifi=up; else wifi=DOWN; fi

  lora=lost
  for a in "$A71" "$A78"; do
    if [ -n "$a" ] && [ "$a" -le 180 ] 2>/dev/null; then lora=heard; fi
  done

  # Deliberately NOT including SUSPECT in the state key. It is an exact-equality test, so it
  # toggles every time the relay's reported mean drifts a single dB - which is not a change
  # in anything real and was firing a notification each time. The CSV records both RSSI
  # columns every sample, so the relay determination is made properly at analysis time from
  # their delta; this line only exists to announce genuine wifi/lora transitions.
  state="wifi=$wifi lora=$lora"
  if [ "$state" != "$prev_state" ]; then
    prev_state="$state"
    echo "[${ELAPSED}s] wifi=$wifi lora=$lora | .71 rssi=${R71:-?} (rly ${L71:-?}) age=${A71:-?}s | .78 rssi=${R78:-?} (rly ${L78:-?}) age=${A78:-?}s | suspect=$SUSPECT"
  fi

  sleep 15
done
