# serial_worker.py
# serial_handler.py
from message_utils import parse_serial_line, send_message
from PyQt5.QtCore import QObject, QThread, pyqtSignal
import serial
import serial.tools.list_ports


class SerialWorker(QObject):
    data_received = pyqtSignal(str)
    ip_ir_updated = pyqtSignal(float, float)

    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self._is_running = True
        self.serial_conn = None

    def start(self):
        try:
            self.serial_conn = serial.Serial(
                self.port, self.baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return

        while self._is_running and self.serial_conn.is_open:
            try:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.data_received.emit(line)

                    # Example: expecting line like "IP:1.23 IR:4.56"
                    if "IP:" in line and "IR:" in line:
                        try:
                            parts = line.split()
                            ip = float(parts[0].split(":")[1])
                            ir = float(parts[1].split(":")[1])
                            self.ip_ir_updated.emit(ip, ir)
                        except (ValueError, IndexError):
                            pass

            except serial.SerialException:
                break

    def stop(self):
        self._is_running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
