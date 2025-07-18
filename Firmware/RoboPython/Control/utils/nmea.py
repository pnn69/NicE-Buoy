from .helpers import parse_value, calculate_checksum
from robobuoy_pid_ui.constants import BUOYIDALL, ME

serial_data_by_ids = {}

def compute_nmea_crc(sentence: str) -> str:
    crc = 0
    for char in sentence:
        crc ^= ord(char)
    return f"{crc:02X}"


def parse_serial_line(line):
    if not line.startswith('$') or '*' not in line:
        return None
    try:
        body, checksum_str = line[1:].split('*', 1)
        calculated_checksum = calculate_checksum(body)
        expected_checksum = int(checksum_str.strip(), 16)
    except ValueError:
        return None

    if calculated_checksum != expected_checksum:
        return None

    fields = body.split(',')
    if len(fields) < 5:
        return None

    try:
        IDr = int(fields[0])
        IDs = fields[1]
        ACK = int(fields[2])
        command = int(fields[3])
        status = int(fields[4])
        data_fields = [parse_value(f) for f in fields[5:]]
    except ValueError:
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
    base_message = f"{BUOYIDALL:06x},{ME:x}," + base_message.strip()
    crc = compute_nmea_crc(base_message)
    return f"${base_message}*{crc}"
