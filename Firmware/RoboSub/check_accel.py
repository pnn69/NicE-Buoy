import socket
import json
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 1001))
sock.settimeout(1.0)

start = time.time()
while time.time() - start < 3:
    try:
        msg, _ = sock.recvfrom(1024)
        if msg.startswith(b'{'):
            data = json.loads(msg.decode('ascii'))
            print("LSM Accel:", data['l_a'])
            break
    except:
        pass
