import socket
import json
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 1001))
sock.settimeout(1.0)

print("Listening for 15 seconds while you rotate the buoy...")
start = time.time()
data = []

while time.time() - start < 15:
    try:
        msg, _ = sock.recvfrom(1024)
        if msg.startswith(b'{'):
            data.append(json.loads(msg.decode('ascii')))
    except Exception as e:
        pass

print(f"Captured {len(data)} JSON samples.")
if data:
    for key in ['l_m', 'i_m', 'l_a', 'i_a']:
        try:
            xs = [d[key][0] for d in data]
            ys = [d[key][1] for d in data]
            zs = [d[key][2] for d in data]
            print(f"{key} X: {min(xs):.1f} to {max(xs):.1f}  |  Y: {min(ys):.1f} to {max(ys):.1f}  |  Z: {min(zs):.1f} to {max(zs):.1f}")
        except KeyError:
            pass
