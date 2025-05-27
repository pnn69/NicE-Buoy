import sys
import socket
import os
import json
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QGroupBox, QTextEdit
)
from PyQt5.QtWidgets import QRadioButton, QButtonGroup
from PyQt5.QtWidgets import QSlider
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QTextCursor
import serial  # Add at the top if not already present
import serial.tools.list_ports

UDP_PORT = 1001
BROADCAST_IP = '255.255.255.255'
SETTINGS_FILE = "RobobuoyConfig.json"
PIDRUDDERSET = 56
PIDSPEEDSET = 58


def float_to_str(f):
    return format(f, '.10g')  # Strips trailing zeros and keeps precision


def compute_nmea_crc(sentence: str) -> str:
    crc = 0
    for char in sentence:
        crc ^= ord(char)
    return f"{crc:02X}"  # 2-digit uppercase hex


def send_udp_broadcast(values, id, group_name, serial_port=None):
    ordered_keys = ["Kps", "Kis", "Kds"] if group_name == "SpeedPid" else [
        "Kpr", "Kir", "Kdr"]
    try:
        ordered_values = [values[k] for k in ordered_keys]
    except KeyError:
        print("Error: Missing values for", group_name)
        return

    value_strs = [float_to_str(v) for v in ordered_values]
    base_message = f"1,2,6,{id},0," + ",".join(value_strs)+",0,0,0"
    crc_hex = compute_nmea_crc(base_message)
    full_message = f"${base_message}*{crc_hex}"
    print("Sending (UDP):", full_message)

    # Send UDP
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
#        sock.sendto(full_message.encode('utf-8'), (BROADCAST_IP, UDP_PORT))
        sock.close()
    except Exception as e:
        print(f"UDP send failed: {e}")

    # Send to COM60
    try:
        if serial_port and serial_port.is_open:
            serial_port.write((full_message + '\n').encode())
            print("Sent to COM60.")
        else:
            print("Serial port not available or not open.")
    except serial.SerialException as e:
        print(f"Failed to send to COM60: {e}")


def save_values(group_values):
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(group_values, f, indent=4)
    except Exception as e:
        print("Error saving values:", e)


def load_values():
    if not os.path.isfile(SETTINGS_FILE):
        return {}
    try:
        with open(SETTINGS_FILE, 'r') as f:
            return json.load(f)
    except Exception as e:
        print("Error loading saved values:", e)
        return {}


class PIDSender(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID UDP Broadcaster")
        self.setMinimumWidth(400)

        self.saved_values = load_values()
        layout = QVBoxLayout()

        # Group 1
        self.SpeedPid_inputs = self.create_input_group(
            "Speed (Kp, Ki, Kd)", ["Kps", "Kis", "Kds"], self.send_SpeedPid)
        layout.addWidget(self.SpeedPid_inputs["box"])

        # Group 2
        self.RudderPid_inputs = self.create_input_group(
            "Rudder (Kp, Ki, Kd)", ["Kpr", "Kir", "Kdr"], self.send_RudderPid)
        layout.addWidget(self.RudderPid_inputs["box"])

        self.setLayout(layout)

        # Load stored values
        self.restore_values()

        self.serial_display = QTextEdit()
        self.serial_display.setReadOnly(True)
        self.serial_display.setMinimumHeight(150)
        layout.addWidget(QLabel("Incoming Serial Data:"))
        layout.addWidget(self.serial_display)

        self.serial_port = None
        self.setup_serial_port("COM60")

        # Timer to poll serial input
        self.serial_timer = QTimer()
        self.serial_timer.timeout.connect(self.read_serial_data)
        self.serial_timer.start(200)  # Poll every 200ms

# Add sliders for Rudder and Speed
        layout.addWidget(QLabel("Rudder Control:"))
        self.rudder_slider, self.rudder_label = self.create_slider("Rudder")
        layout.addLayout(self.rudder_slider)

        layout.addWidget(QLabel("Speed Control:"))
        self.speed_slider, self.speed_label = self.create_slider("Speed")
        layout.addLayout(self.speed_slider)

 # Add radio buttons for Lock, Remote, and Idle
        mode_group_box = QGroupBox("Control Mode")
        mode_layout = QHBoxLayout()

        self.lock_radio = QRadioButton("Lock")
        self.remote_radio = QRadioButton("Remote")
        self.idle_radio = QRadioButton("Idle")

        self.radio_group = QButtonGroup()
        self.radio_group.addButton(self.lock_radio)
        self.radio_group.addButton(self.remote_radio)
        self.radio_group.addButton(self.idle_radio)

        mode_layout.addWidget(self.lock_radio)
        mode_layout.addWidget(self.remote_radio)
        mode_layout.addWidget(self.idle_radio)

        self.lock_radio.setChecked(True)  # Default selection

        mode_group_box.setLayout(mode_layout)
        layout.addWidget(mode_group_box)


    def get_selected_mode(self):
            if self.lock_radio.isChecked():
                return "Lock"
            elif self.remote_radio.isChecked():
                return "Remote"
            elif self.idle_radio.isChecked():
                return "Idle"
            return None

    def create_input_group(self, title, var_names, send_callback):
        box = QGroupBox(title)
        vbox = QVBoxLayout()
        inputs = {}

        for var in var_names:
            hbox = QHBoxLayout()
            label = QLabel(var)
            field = QLineEdit()
            field.setPlaceholderText("Enter float")
            hbox.addWidget(label)
            hbox.addWidget(field)
            vbox.addLayout(hbox)
            inputs[var] = field

        send_button = QPushButton("Send")
        send_button.clicked.connect(send_callback)
        vbox.addWidget(send_button, alignment=Qt.AlignRight)

        box.setLayout(vbox)
        return {"box": box, "inputs": inputs}

    def get_values(self, input_fields):
        values = {}
        for key, field in input_fields.items():
            try:
                values[key] = float(field.text())
            except ValueError:
                values[key] = 0.0  # Default value
        return values

    def restore_values(self):
        for group_name, fields in [("SpeedPid", self.SpeedPid_inputs["inputs"]),
                                   ("RudderPid", self.RudderPid_inputs["inputs"])]:
            group_vals = self.saved_values.get(group_name, {})
            for key, field in fields.items():
                if key in group_vals:
                    field.setText(str(group_vals[key]))

    def send_SpeedPid(self):
        values = self.get_values(self.SpeedPid_inputs["inputs"])
        send_udp_broadcast(values, PIDSPEEDSET, "SpeedPid", self.serial_port)
        self.saved_values["SpeedPid"] = values
        save_values(self.saved_values)

    def send_RudderPid(self):
        values = self.get_values(self.RudderPid_inputs["inputs"])
        send_udp_broadcast(values, PIDRUDDERSET, "RudderPid", self.serial_port)
        self.saved_values["RudderPid"] = values
        save_values(self.saved_values)

    def closeEvent(self, event):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        event.accept()

    def setup_serial_port(self, port_name):
        try:
            self.serial_port = serial.Serial(
                port=port_name, baudrate=115200, timeout=0.1)
            print(f"Serial port {port_name} opened for reading.")
        except serial.SerialException as e:
            print(f"Failed to open serial port {port_name}: {e}")
            self.serial_port = None
    def create_slider(self, name):
            hbox = QHBoxLayout()
            label = QLabel("0")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-100)
            slider.setMaximum(100)
            slider.setValue(0)
            slider.setTickInterval(10)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.valueChanged.connect(lambda val: label.setText(str(val)))
            hbox.addWidget(slider)
            hbox.addWidget(label)
            return hbox, label

    def read_serial_data(self):
        if self.serial_port and self.serial_port.in_waiting:
            try:
                data = self.serial_port.read(
                    self.serial_port.in_waiting).decode(errors='ignore')
                if data:
                    self.serial_display.moveCursor(QTextCursor.End)
                    self.serial_display.insertPlainText(data)
                    self.serial_display.verticalScrollBar().setValue(
                        self.serial_display.verticalScrollBar().maximum())
            except Exception as e:
                print(f"Error reading serial data: {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PIDSender()
    window.show()
    sys.exit(app.exec_())
