import socket
import time

def compute_nmea_crc(msg):
    crc = 0
    for char in msg:
        crc ^= ord(char)
    return crc

def send_command(target_id, cmd, status):
    base_msg = f"{target_id},99,3,{cmd},{status}"
    crc = compute_nmea_crc(base_msg)
    full_msg = f"${base_msg}*{crc:02X}"
    
    print(f"Sending: {full_msg}")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.sendto(full_msg.encode(), ("255.255.255.255", 1001))
        sock.close()
    except Exception as e:
        print("Error:", e)

if __name__ == "__main__":
    # Send to the specific buoy MAC found in top_log.txt
    send_command("B7A5B578", 12, 7) # LOCKING
    
    # Also send to the universal broadcast ID defined in firmware as 1
    time.sleep(1)
    send_command("1", 12, 7)
