import serial
import socket
import threading
import time

PORT_TOP = 'COM51'
PORT_LORA = 'COM41'
BAUD = 115200
UDP_IP = "0.0.0.0"
UDP_PORT = 1001
running = True

def listen_serial(port_name, prefix):
    try:
        with serial.Serial(port_name, BAUD, timeout=1) as ser:
            print(f"[{prefix}] Connected to {port_name}")
            while running:
                line = ser.readline()
                if line:
                    decoded = line.decode('utf-8', errors='ignore').strip()
                    if decoded:
                        print(f"[{prefix}] {decoded}")
    except Exception as e:
        print(f"[{prefix}] Error: {e}")

def listen_udp():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        sock.settimeout(1.0)
        print(f"[UDP] Listening on port {UDP_PORT}")
        while running:
            try:
                data, addr = sock.recvfrom(1024)
                decoded = data.decode('utf-8', errors='ignore').strip()
                print(f"[UDP from {addr[0]}] {decoded}")
            except socket.timeout:
                pass
    except Exception as e:
        print(f"[UDP] Error: {e}")

def send_udp_msg(msg):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.sendto(msg.encode('utf-8'), ("255.255.255.255", UDP_PORT))
        print(f"[TEST RUNNER] Sent UDP broadcast: {msg}")
    except Exception as e:
        print(f"[TEST RUNNER] Failed to send UDP: {e}")

def send_serial_msg(port_name, msg):
    try:
        with serial.Serial(port_name, BAUD, timeout=1) as ser:
            ser.write((msg + "\n").encode('utf-8'))
            print(f"[TEST RUNNER] Sent to {port_name}: {msg}")
    except Exception as e:
        print(f"[TEST RUNNER] Failed to send to {port_name}: {e}")

if __name__ == '__main__':
    t_top = threading.Thread(target=listen_serial, args=(PORT_TOP, "TOP COM51"), daemon=True)
    t_lora = threading.Thread(target=listen_serial, args=(PORT_LORA, "LORA COM41"), daemon=True)
    t_udp = threading.Thread(target=listen_udp, daemon=True)

    t_top.start()
    t_lora.start()
    t_udp.start()

    time.sleep(3) # Let listeners start

    # Test 1: Send a command via UDP to Top Buoy (ID=1) to IDLE (CMD=8)
    # Format: $IDr,IDs,ACK,CMD,STATUS*CRC
    # $1,99,3,8,7*CRC  -> 1=Target ID, 99=Sender(PC), 3=LORAGETACK, 8=IDLE cmd, 7=IDLE status
    test_msg_1 = "$1,99,3,8,7*64"
    print("\n--- TEST 1: Sending UDP IDLE command ---")
    send_udp_msg(test_msg_1)
    
    time.sleep(3)

    # Test 2: Send a command via LoRa (COM41) to Top Buoy (ID=1) to IDLE (CMD=8)
    print("\n--- TEST 2: Sending LoRa IDLE command ---")
    send_serial_msg(PORT_LORA, test_msg_1)

    time.sleep(3)

    # Let listeners capture responses for 10 more seconds
    print("\n--- Waiting for responses ---")
    time.sleep(10)
    
    running = False
    print("Done testing.")