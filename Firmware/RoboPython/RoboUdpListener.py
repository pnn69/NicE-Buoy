# Given:
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
    print(f"Received from {addr}: {msg}")

    # Validate format
    if '*' not in msg or not msg.startswith('$'):
        print("Invalid format: missing '$' or '*' for checksum separation")
        continue

    try:
        # Split payload and checksum
        payload, received_chk_str = msg[1:].split('*', 1)  # Exclude initial '$'
        received_chk_str = received_chk_str.strip()

        if len(received_chk_str) != 2:
            print("Invalid checksum length")
            continue

        # Calculate checksum on the payload
        calculated_chk = calculate_checksum(payload)
        received_chk = int(received_chk_str, 16)

        if calculated_chk != received_chk:
            print(f"Checksum mismatch: calculated {calculated_chk:02X}, received {received_chk_str}")
            continue

        # Parse fields
        parts = payload.split(',')

        if len(parts) < 4:
            print(f"Invalid payload parts count, expected at least 4 fields but got {len(parts)}")
            continue

        IDr, IDs, ack, msg_field = parts[0], parts[1], parts[2], parts[3]
        data_fields = parts[4:]
        data_count = len(data_fields)

        # Output decoded result
        print("Decoded:")
        print(f"  IDr = {IDr}")
        print(f"  IDs = {IDs}")
        print(f"  ack = {ack}")
        print(f"  msg = {msg_field}")
        print(f"  Number of data fields = {data_count}")
        print(f"  data fields = {data_fields}")

        # → At this point, you can process IDr, IDs, ack, msg_field, data_fields further as needed.

    except Exception as e:
        print(f"Error processing message: {e}")
# write the script that
# decode incomming data to IDr IDs ack msg data if the checksum is correct
    #
