import socket
import threading

# Define the broadcast IP address and port
UDP_BROADCAST_IP = "255.255.255.255"  # Broadcast address
UDP_PORT = 1001

# Define a message to send
MESSAGE = "Hello, this is a broadcast message!"

# Function to send a UDP broadcast message
def send_udp_broadcast():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        # Enable broadcasting mode
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.sendto(MESSAGE.encode(), (UDP_BROADCAST_IP, UDP_PORT))
        print(f"Sent broadcast message: {MESSAGE}")

# Function to receive UDP messages
def receive_udp_message():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("", UDP_PORT))  # Bind to all available network interfaces
        print(f"Listening for incoming messages on port {UDP_PORT}...")
        while True:
            data, addr = sock.recvfrom(1024)  # Buffer size of 1024 bytes
            print(f"Received message: {data.decode()} from {addr}")

# Start the receiver in a separate thread
receiver_thread = threading.Thread(target=receive_udp_message)
receiver_thread.daemon = True  # Allows the program to exit if only daemon threads are left
receiver_thread.start()

# Send a broadcast message
send_udp_broadcast()

# Keep the main thread alive to listen for messages
input("Press Enter to exit...\n")
