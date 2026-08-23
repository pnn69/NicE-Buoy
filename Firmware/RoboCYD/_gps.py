import io

p = 'src/buoy_data.cpp'
s = io.open(p, encoding='utf-8', errors='surrogateescape').read()

old = '''    // Same envelope as send_buoy_command(), but with the still-water flag in the first payload
    // field. RoboTop reads it as fields[5]; an older node that sends no payload there decodes as
    // 0, which is the current-tolerant pair-averaged mode.
    String cmdStr = buoy_id + ",98,3,89,89," + String(still_water ? 1 : 0);'''
new = '''    // Same envelope as send_buoy_command(), but with the still-water flag in the first payload
    // field. RoboTop reads it as fields[5]; an older node that sends no payload there decodes as
    // 0, which is the current-tolerant pair-averaged mode.
    //
    // ack 6 = INF, matching what RoboTop's own web page sends for this command. NOT 3 (GETACK):
    // that would enrol the packet in RoboTop's LoRa retransmit table (retry = 5, loratop.cpp) and
    // no immediate ACK is ever sent for a GETACK, so the start command would be re-sent about
    // once a second for five seconds. RoboTop's handler ignores a repeat once the run is under
    // way, so this never broke anything - it just filled the air at the worst moment.
    String cmdStr = buoy_id + ",98,6,89,89," + String(still_water ? 1 : 0);'''

assert s.count(old) == 1, s.count(old)
s = s.replace(old, new)
io.open(p, 'w', encoding='utf-8', errors='surrogateescape', newline='').write(s)
print("send_gps_fourier_calibrate now uses INF")
