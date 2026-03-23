import socket
from message_utils import generate_nmea_message

def test_command():
    # 3 = LORAGETACK, 12 = LOCKING, 7 = IDLE
    base_message = f"3,12,7"
    full_message = generate_nmea_message(base_message)
    print("Sending message:", full_message)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # Using the exact same ip and port as message_utils
        sock.sendto(full_message.encode(), ("255.255.255.255", 1001))
        print("Message sent via broadcast")
    except Exception as e:
        print("Error sending message:", e)

if __name__ == "__main__":
    test_command()
