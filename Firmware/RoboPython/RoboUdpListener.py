import socket

def calculate_checksum(s):
    """Calculate XOR checksum for string s"""
    csum = 0
    for ch in s:
        csum ^= ord(ch)
    return csum

# Setup UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 1001))

print("Listening on UDP port 1001 (IPv4)...")

while True:
    data, addr = sock.recvfrom(1024)
    msg = data.decode(errors='ignore').strip()

    # Validate format
    if '*' not in msg or not msg.startswith('$'):
        continue

    try:
        # Split payload and checksum
        payload, received_chk_str = msg[1:].split('*', 1)  # Exclude initial '$'
        received_chk_str = received_chk_str.strip()

        if len(received_chk_str) != 2:
            continue

        # Calculate checksum on the payload
        calculated_chk = calculate_checksum(payload)
        received_chk = int(received_chk_str, 16)

        if calculated_chk != received_chk:
            continue

        # Parse fields
        parts = payload.split(',')

        if len(parts) < 4:
            continue

        IDr, IDs, ack, msg_field = parts[0], parts[1], parts[2], parts[3]
        data_fields = parts[4:]

        # Filter: only show if IDs == 00ab0b34
        if IDs.lower() != "ab0b34":
            continue

        # Output decoded result
        print("Decoded:")
        print(f"  IDr = {IDr}")
        print(f"  IDs = {IDs}")
        print(f"  ack = {ack}")
        print(f"  msg = {msg_field}")
        print(f"  Number of data fields = {len(data_fields)}")
        print(f"  data fields = {data_fields}")

    except Exception:
        continue
