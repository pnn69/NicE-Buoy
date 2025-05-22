# make a script with 6 input fields containg  float's
# Make two groups
# variable names group 1: Kps Kis Kds
# variable namesgroup 2: Kpr Kir Kdr
# For each group, make a  sendbutton
# If the button is pressed send a udp braodast with the values of the group
import sys
import socket
import os
import json
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QGroupBox
)
from PyQt5.QtCore import Qt

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


def send_udp_broadcast(values, id, group_name):
    ordered_keys = ["Kps", "Kis", "Kds"] if group_name == "SpeedPid" else [
        "Kpr", "Kir", "Kdr"]
    try:
        ordered_values = [values[k] for k in ordered_keys]
    except KeyError:
        print("Error: Missing values for", group_name)
        return

    # Convert to stripped float strings
    value_strs = [float_to_str(v) for v in ordered_values]
    base_message = f"1,2,6,{id},0," + ",".join(value_strs)+",0,0,0"

    # Compute NMEA-style CRC
    crc_hex = compute_nmea_crc(base_message)

    # Final message with *CRC
    full_message = f"${base_message}*{crc_hex}"
    print("Sending:", full_message)

    # Send broadcast
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.sendto(full_message.encode('utf-8'), (BROADCAST_IP, UDP_PORT))
    sock.close()


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
        send_udp_broadcast(values, PIDSPEEDSET, "SpeedPid")
        self.saved_values["SpeedPid"] = values
        save_values(self.saved_values)

    def send_RudderPid(self):
        values = self.get_values(self.RudderPid_inputs["inputs"])
        send_udp_broadcast(values, PIDRUDDERSET, "RudderPid")
        self.saved_values["RudderPid"] = values
        save_values(self.saved_values)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PIDSender()
    window.show()
    sys.exit(app.exec_())
