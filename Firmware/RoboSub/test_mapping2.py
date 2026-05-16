import socket
import json
import time
import math

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 1001))
sock.settimeout(1.0)

print("Listening for 10 seconds. Rotate the buoy...")
start = time.time()

def vector_cross(v1, v2):
    return [
        v1[1]*v2[2] - v1[2]*v2[1],
        v1[2]*v2[0] - v1[0]*v2[2],
        v1[0]*v2[1] - v1[1]*v2[0]
    ]

def vector_dot(v1, v2):
    return sum(x*y for x, y in zip(v1, v2))

def vector_normalize(v):
    mag = math.sqrt(sum(x*x for x in v))
    if mag == 0: return [0,0,0]
    return [x/mag for x in v]

def calc_heading(m, a, f):
    east = vector_normalize(vector_cross(m, a))
    north = vector_normalize(vector_cross(a, east))
    n_dot_f = vector_dot(north, f)
    e_dot_f = vector_dot(east, f)
    if n_dot_f == 0 and e_dot_f == 0: return 0.0
    heading = math.atan2(n_dot_f, e_dot_f) * 180.0 / math.pi
    if heading < 0: heading += 360.0
    return heading

# I will use the min/max from the previous capture to do a live calibration!
hard_l_m = [(60.8 + 100.1)/2, (57.3 + 63.0)/2, (6.8 + 49.8)/2]
hard_i_m = [(-34.3 + 4.8)/2, (59.7 + 65.2)/2, (-15.6 + 29.4)/2]

while time.time() - start < 10:
    try:
        msg, _ = sock.recvfrom(1024)
        if msg.startswith(b'{'):
            data = json.loads(msg.decode('ascii'))
            
            # Raw vectors
            l_m = data['l_m']
            l_a = data['l_a']
            i_m = data['i_m']
            i_a = data['i_a']
            
            # Apply hard iron
            l_m = [l_m[0] - hard_l_m[0], l_m[1] - hard_l_m[1], l_m[2] - hard_l_m[2]]
            i_m = [i_m[0] - hard_i_m[0], i_m[1] - hard_i_m[1], i_m[2] - hard_i_m[2]]
            
            # Physical X is forward, so f = [1, 0, 0]
            f = [1, 0, 0]
            
            hdg_lsm = calc_heading(l_m, l_a, f)
            hdg_icm = calc_heading(i_m, i_a, f)
            
            print(f"LSM: {hdg_lsm:05.1f} | ICM: {hdg_icm:05.1f} | Delta: {abs(hdg_lsm-hdg_icm):05.1f}")
            
    except Exception as e:
        pass
