import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 0))
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# Format: $IDr,IDs,ACK,cmd,status,tgDir,tgDist,...*CRC
# IDr=b7a5b578 (Sub's ID), IDs=1 (Top unit ID), ack=0, cmd=32 (CALIBRATE_MAGNETIC_COMPASS), ...
msg = "$b7a5b578,1,0,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*"

crc = 0
for c in msg[1:msg.find('*')]:
    crc ^= ord(c)

msg += f"{crc:02X}\r\n"

sock.sendto(msg.encode('ascii'), ('192.168.1.189', 1001))
print(f"Sent: {msg.strip()}")
