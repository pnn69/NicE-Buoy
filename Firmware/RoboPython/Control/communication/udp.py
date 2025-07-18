import socket
from robobuoy_pid_ui.constants import BROADCAST_IP, UDP_PORT

def send_udp_message(message: str):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.sendto(message.encode(), (BROADCAST_IP, UDP_PORT))
        sock.close()
    except Exception as e:
        print(f"UDP send failed: {e}")
