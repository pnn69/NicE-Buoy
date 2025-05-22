import socket

UDP_PORT = 1001
BROADCAST_IP = "255.255.255.255"


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

message = "Hello, network!"
msg_bytes = message.encode('utf-8')

# Send to broadcast
sock.sendto(msg_bytes, (BROADCAST_IP, UDP_PORT))


print(f"Sent to broadcast and localhost on port {UDP_PORT}")
sock.close()
