import serial
import serial.tools.list_ports

def setup_serial_port(port_name):
    try:
        port = serial.Serial(port=port_name, baudrate=115200, timeout=0.1)
        print(f"Serial port {port_name} opened.")
        return port
    except serial.SerialException as e:
        print(f"Failed to open serial port {port_name}: {e}")
        return None


def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]
