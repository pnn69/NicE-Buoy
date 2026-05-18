import socket
import time

UDP_IP = "192.168.1.189"  # Top IP (OTA) or Broadcast. We use broadcast for test
UDP_PORT = 1001

# The Top buoy's ID (assuming ID 1 for test, or we catch the first broadcast)
TOP_ID = "1"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(("0.0.0.0", UDP_PORT))
sock.settimeout(2.0)

def compute_crc(message):
    crc = 0
    for char in message:
        crc ^= ord(char)
    return crc

def send_cmd(cmd_str):
    crc = compute_crc(cmd_str)
    full_msg = f"${cmd_str}*{crc:02X}"
    sock.sendto(full_msg.encode(), ("255.255.255.255", UDP_PORT))
    print(f"[TX] {full_msg}")

def wait_for_response(expected_cmd, expected_ack="6"):
    start = time.time()
    while time.time() - start < 5.0:
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode().strip()
            print(f"  [DEBUG RX] {msg}")
            if f",{expected_ack},{expected_cmd}," in msg:
                return msg
        except socket.timeout:
            pass
    return None

def parse_setupdata(msg):
    # Example: $MAC,IDr,ACK,CMD,status,Kpr,Kir,Kdr,Kps,Kis,Kds,max,min,pivot,comp,icmcomp,minoff,revbb,revsb*CRC
    parts = msg.split(',')
    if len(parts) >= 11:
        try:
            return {
                'Kps': float(parts[8]) if parts[8] else 0.0,
                'Kis': float(parts[9]) if parts[9] else 0.0,
                'Kds': float(parts[10]) if parts[10] else 0.0
            }
        except ValueError:
            return None
    return None

def wait_for_valid_setupdata():
    start = time.time()
    while time.time() - start < 10.0:
        resp = wait_for_response("83", "6")
        if resp:
            params = parse_setupdata(resp)
            if params and params['Kps'] > 0: # Assuming it shouldn't be exactly 0 if loaded
                return params
    return None

print("=== STARTING FULL READ-MODIFY-STORE TEST ===")

# 1. Request current data (LORAGET = 1, CMD = 83)
print("\n1. Requesting current SETUPDATA...")
send_cmd(f"{TOP_ID},99,1,83,,,,,,,")
params1 = wait_for_valid_setupdata()

if not params1:
    print("FAIL: Could not load valid initial SETUPDATA.")
    exit(1)
print(f"Current Parameters: {params1}")

# 2. Modify a value
new_kps = params1['Kps'] + 1.5
if new_kps > 100: new_kps = 10.0 # reset if too high

print(f"\n2. Sending modified Kps: {new_kps:.2f} (CMD = 58 PIDSPEEDSET)")
# CMD 58 (PIDSPEEDSET), ACK 3 (LORAGETACK). Fields: Kps, Kis, Kds
send_cmd(f"{TOP_ID},99,3,58,,{new_kps:.2f},{params1['Kis']:.2f},{params1['Kds']:.2f},,,")

# Wait for the ack response of the SET
resp_set = wait_for_response("58", "6")
if resp_set:
    print("Received ACK for SET command.")
else:
    print("Warning: Did not see ACK for SET command, continuing anyway...")

time.sleep(1) # Give Sub a moment to save to NVS

# 3. Clear local state and re-request
print("\n3. Re-requesting SETUPDATA to verify save...")
send_cmd(f"{TOP_ID},99,1,83,,,,,,,")
resp2 = wait_for_response("83", "6")
if not resp2:
    print("FAIL: No response to second SETUPDATA request.")
    exit(1)

params2 = parse_setupdata(resp2)
print(f"Newly Loaded Parameters: {params2}")

# 4. Compare
if abs(params2['Kps'] - new_kps) < 0.01:
    print("\nSUCCESS: The parameter was successfully stored in the Sub and read back!")
else:
    print(f"\nFAIL: Parameter did not update. Expected {new_kps:.2f}, got {params2['Kps']:.2f}")

