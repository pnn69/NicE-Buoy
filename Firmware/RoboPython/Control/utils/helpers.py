def float_to_str(f):
    return format(f, '.10g')


def parse_value(val):
    try:
        if '.' in val:
            return float(val)
        else:
            return int(val)
    except ValueError:
        return val


def calculate_checksum(nmea_str):
    checksum = 0
    for char in nmea_str:
        checksum ^= ord(char)
    return checksum


def log(widget, msg):
    widget.serial_display.append(f"[LOG] {msg}")
