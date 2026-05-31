import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Command 0x99 is 153 decimal
sock.sendto(b'{"cmd":153}', ('192.168.1.189', 1001))
print("Trigger sent to 192.168.1.189:1001")
