import socket
import serial
import serial.tools.list_ports

# Constants
UDP_PORT = 1001
# COMPORT = "COM60"
# COMPORT = "COM40"
BROADCAST_IP = '255.255.255.255'
SETTINGS_FILE = "RobobuoyConfig.json"
PIDRUDDERSET = 56
PIDSPEEDSET = 58
LOCKING = 12
IDELING = 8
IDLE = 7
DOCKING = 15
REMOTE = 25
BUOYIDALL = 0xb7a5b58c
ME = 0x99
LORAGETACK = 3
LORAINF = 6
STORE_DECLINATION = 31
MAXMINPWR = 68
MAXMINPWRSET = 69
DIRDIST = 47
# data
serial_data_by_ids = {}


def open_serial_port(port_name, baudrate=115200, timeout=0.1):
    try:
        ser = serial.Serial(port=port_name, baudrate=baudrate, timeout=timeout)
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port {port_name}: {e}")
        return None


def read_serial_line(ser):
    try:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        return line
    except Exception as e:
        print(f"Error reading from serial port: {e}")
        return None


def list_serial_ports():
    return [port.device for port in serial.tools.list_ports.comports()]


def compute_nmea_crc(sentence: str) -> str:
    crc = 0
    for char in sentence:
        crc ^= ord(char)
    return f"{crc:02X}"


def parse_value(val):
    """Convert to int or float if possible, else return original string."""
    try:
        if '.' in val:
            return float(val)
        else:
            return int(val)
    except ValueError:
        return val


def calculate_checksum(nmea_str):
    """Calculate XOR checksum for characters in the string."""
    checksum = 0
    for char in nmea_str:
        checksum ^= ord(char)
    return checksum


def parse_serial_line(line):
    if not line.startswith('$') or '*' not in line:
        print("Invalid start or missing '*'")
        return None

    try:
        body, checksum_str = line[1:].split('*', 1)
        calculated_checksum = calculate_checksum(body)
        expected_checksum = int(checksum_str.strip(), 16)
    except ValueError as e:
        print(f"Checksum parsing error: {e}")
        return None

    if calculated_checksum != expected_checksum:
        print(
            f"Checksum mismatch: {calculated_checksum:02X} != {expected_checksum:02X}")
        return None

    fields = body.split(',')
    if len(fields) < 5:
        print(f"Too few fields: {fields}")
        return None

    try:
        IDr = int(fields[0], 16)       # <-- base 16 for hex string
        IDs = int(fields[1], 16)
        ACK = int(fields[2])
        command = int(fields[3])
        status = int(fields[4])
        data_fields = [parse_value(f) for f in fields[5:]]
    except ValueError as e:
        print(f"Data parsing error: {e}")
        return None

    decoded = {
        'IDr': IDr,
        'IDs': IDs,
        'ACK': ACK,
        'command': command,
        'status': status,
        'data': data_fields,
        'checksum': f"{expected_checksum:02X}"
    }

    serial_data_by_ids[IDs] = decoded
    return decoded


def generate_nmea_message(base_message: str) -> str:
    """
    Generate full NMEA message string with CRC.
    $IDr,IDs,ack,msg,data*chk
    """
    base_message = f"{BUOYIDALL:06x},{ME:x}," + base_message
    base_message = base_message.lstrip()
    crc = compute_nmea_crc(base_message)
    return f"${base_message}*{crc}"


def send_message(full_message: str, serial_port=None):
    """
    Send the full message string via UDP broadcast and optionally via serial.
    """
    print("Sending message:", full_message)

    # Send UDP
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.sendto(full_message.encode(), (BROADCAST_IP, UDP_PORT))
        sock.close()
    except Exception as e:
        print(f"UDP send failed: {e}")

    # Send via serial
    try:
        if serial_port and serial_port.is_open:
            # serial_port.write((full_message + '\n').encode())
            serial_port.write(full_message.encode())  # <-- No \n
        else:
            print("Serial port not available or not open.")
    except serial.SerialException as e:
        print(f"Serial send failed: {e}")
