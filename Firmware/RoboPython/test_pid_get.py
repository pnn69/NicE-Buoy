import socket
import time

def compute_nmea_crc(msg):
    crc = 0
    for char in msg:
        crc ^= ord(char)
    return crc

def send_and_listen(base_msg):
    crc = compute_nmea_crc(base_msg)
    full_msg = f"${base_msg}*{crc:02X}"
    
    print(f"Sending: {full_msg}")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(2.0)
        sock.bind(("0.0.0.0", 1001)) # Bind to listen for replies
        sock.sendto(full_msg.encode(), ("255.255.255.255", 1001))
        
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                msg = data.decode('utf-8').strip()
                print(f"Received: {msg}")
                if "55" in msg or "57" in msg:
                    print("Found PID response!")
            except socket.timeout:
                print("Timeout waiting for response.")
                break
        sock.close()
    except Exception as e:
        print("Error:", e)

if __name__ == "__main__":
    target_id = "B7A5B578" # Or "1"
    
    # Test 1: Just CMD and STATUS like we did in udp_monitor.py
    print("--- TEST 1 ---")
    send_and_listen(f"{target_id},99,1,55,7")
    
    time.sleep(1)
    
    # Test 2: Pad with zeros like RoboGui.py does when sending
    print("--- TEST 2 ---")
    send_and_listen(f"{target_id},99,1,55,0,0,0,0,0,0,0")
    
    time.sleep(1)
    
    # Test 3: Broadcast ID
    print("--- TEST 3 ---")
    send_and_listen(f"1,99,1,55,0,0,0,0,0,0,0")
