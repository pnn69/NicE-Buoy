#!/usr/bin/env bash
# A/B antenna walk: the CYD and RoboLora carried together, away from the buoys.
#
# Both transmitters are at the same place at the same moment, on the same band, so every path
# effect - distance, terrain, fading - is common mode. What is left between them is the ANTENNA.
# The two Tops stay home and stay pollable, and each reports what it hears from both:
#
#   id 98        the handheld
#   id 1c3101ac  RoboLora
#
# delta = cyd_rssi - rly_rssi is the antenna advantage, read directly, once per sample.
#
# THE SUBTLETY. RoboLora repeats the handheld's beacon, and it is walking beside it. linkNote()
# files a reception against the sender id written INSIDE the frame, so a relayed copy is recorded
# as if it came from the handheld - see the attribution bug found earlier today. That does not
# ruin this experiment, it sharpens it:
#
#   linkFirstCopy() keeps the FIRST copy of a frame, and a relay is deferred at least 40 ms. So
#   while the handheld is directly receivable its own copy always wins, and cyd_rssi is genuine.
#   Only once the direct path dies does the relayed copy become the first one seen - and then
#   cyd_rssi collapses onto rly_rssi exactly.
#
#   delta ~= 0 therefore means "the handheld's direct link is DEAD and RoboLora is carrying it",
#   which is precisely the number this walk is trying to find.
#
# Full samples to CSV; stdout only on a real transition.

CSV=/c/tmp/ab_walk.csv
T71=192.168.1.71
T78=192.168.1.78
RLY=1c3101ac

echo "iso_time,elapsed_s,t71_cyd,t71_rly,t71_delta,t71_age,t78_cyd,t78_rly,t78_delta,t78_age,verdict" > "$CSV"

START=$(date +%s)
fetch() { curl -s -m 5 "http://$1/data" 2>/dev/null; }
peer() { echo "$1" | grep -o "{\"id\":\"$2\"[^}]*}" | grep -o "\"$3\":-\?[0-9]*" | head -1 | cut -d: -f2; }

prev=""
while true; do
  ISO=$(date +%Y-%m-%dT%H:%M:%S); ELAPSED=$(( $(date +%s) - START ))
  J71=$(fetch $T71); J78=$(fetch $T78)

  C71=$(peer "$J71" 98 rssi);   L71=$(peer "$J71" $RLY rssi);  A71=$(peer "$J71" 98 age)
  C78=$(peer "$J78" 98 rssi);   L78=$(peer "$J78" $RLY rssi);  A78=$(peer "$J78" 98 age)

  D71=""; D78=""
  [ -n "$C71" ] && [ -n "$L71" ] && D71=$(( C71 - L71 ))
  [ -n "$C78" ] && [ -n "$L78" ] && D78=$(( C78 - L78 ))

  # Is the handheld's own direct path still alive at either Top? delta within 3 dB of zero means
  # we are looking at RoboLora's relay, not the handheld.
  direct=no
  for d in "$D71" "$D78"; do
    if [ -n "$d" ] && { [ "$d" -lt -3 ] || [ "$d" -gt 3 ]; } 2>/dev/null; then direct=yes; fi
  done
  [ -z "$C71$C78" ] && direct=none

  rly=no
  { [ -n "$L71" ] || [ -n "$L78" ]; } && rly=yes

  echo "$ISO,$ELAPSED,${C71},${L71},${D71},${A71},${C78},${L78},${D78},${A78},cyd_direct=$direct" >> "$CSV"

  state="direct=$direct rly=$rly"
  if [ "$state" != "$prev" ]; then
    prev="$state"
    echo "[${ELAPSED}s] cyd_direct=$direct rly=$rly | .71 cyd=${C71:-?} rly=${L71:-?} d=${D71:-?} | .78 cyd=${C78:-?} rly=${L78:-?} d=${D78:-?}"
  fi

  sleep 15
done
