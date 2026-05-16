import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 0))
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# cmd=51 (SETUPDATA), IDr=b7a5b578, IDs=1 (THIS FIXES THE ROUTING!)
msg = "$b7a5b578,1,2,51,0,0,0,0,0,0,0,0,0,0,180.0,0,180.0,0,0*"

crc = 0
for c in msg[1:msg.find('*')]:
    crc ^= ord(c)

msg += f"{crc:02X}\r\n"

sock.sendto(msg.encode('ascii'), ('192.168.1.189', 1001))
print(f"Sent: {msg.strip()}")
