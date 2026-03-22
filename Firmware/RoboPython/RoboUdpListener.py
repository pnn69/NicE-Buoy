import socket
import sys
from IDS import MsgType

def calculate_checksum(s):
    """Calculate XOR checksum for string s"""
    csum = 0
    for ch in s:
        csum ^= ord(ch)
    return csum

def get_msg_name(msg_id):
    """Return the name of the message type if known"""
    try:
        msg_int = int(msg_id)
        for msg in MsgType:
            if msg.value == msg_int:
                return msg.name
    except (ValueError, TypeError):
        pass
    return f"UNKNOWN({msg_id})"

def main():
    # Configuration
    UDP_IP = "0.0.0.0"
    UDP_PORT = 1001
    
    # Target source ID (IDs) to filter for. 
    # Use hex string format (e.g., "ab0b34"). Set to None to see all.
    TARGET_ID_STR = "ab0b34"  
    TARGET_ID = int(TARGET_ID_STR, 16) if TARGET_ID_STR else None

    # Setup UDP socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        print(f"Listening on UDP port {UDP_PORT} (IPv4)...")
        if TARGET_ID is not None:
            print(f"Filtering for source ID: 0x{TARGET_ID:02x}")
        else:
            print("Showing all received messages.")
    except Exception as e:
        print(f"Failed to bind socket: {e}")
        sys.exit(1)

    while True:
        try:
            data, addr = sock.recvfrom(2048)
            msg = data.decode(errors='ignore').strip()

            # Validate format
            if not msg.startswith('$') or '*' not in msg:
                continue

            # Split payload and checksum
            try:
                payload, received_chk_str = msg[1:].split('*', 1)
                received_chk_str = received_chk_str.strip()
            except ValueError:
                continue

            # Validate checksum hex format
            if len(received_chk_str) != 2:
                continue

            try:
                received_chk = int(received_chk_str, 16)
            except ValueError:
                continue

            # Calculate and verify checksum
            calculated_chk = calculate_checksum(payload)
            if calculated_chk != received_chk:
                print(f"Checksum mismatch: calc={calculated_chk:02X}, recv={received_chk:02X} in {msg}")
                continue

            # Parse fields
            parts = payload.split(',')
            if len(parts) < 4:
                continue

            # IDr, IDs, ack, cmd, status, data...
            try:
                IDr = int(parts[0], 16)
                IDs = int(parts[1], 16)
            except ValueError:
                # If they aren't valid hex, they don't match our protocol
                continue

            # Filter by source ID (IDs)
            if TARGET_ID is not None and IDs != TARGET_ID:
                continue

            ack_str = parts[2]
            cmd_id = parts[3]
            
            # Output decoded result
            print("-" * 40)
            print(f"From: {addr[0]}:{addr[1]}")
            print(f"  IDr (Dest): 0x{IDr:02x}")
            print(f"  IDs (Src):  0x{IDs:02x}")
            print(f"  ACK:        {ack_str}")
            print(f"  Command:    {cmd_id} ({get_msg_name(cmd_id)})")
            
            if len(parts) > 4:
                status = parts[4]
                data_fields = parts[5:]
                print(f"  Status:     {status}")
                if data_fields:
                    print(f"  Data:       {', '.join(data_fields)}")

        except KeyboardInterrupt:
            print("\nStopping listener...")
            break
        except Exception as e:
            print(f"Error processing packet: {e}")
            continue

if __name__ == "__main__":
    main()
