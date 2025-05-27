import sys
import os
import json
import socket
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QGroupBox, QTextEdit,
    QSlider, QRadioButton, QButtonGroup
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QTextCursor

# Constants
UDP_PORT = 1001
BROADCAST_IP = '255.255.255.255'
SETTINGS_FILE = "RobobuoyConfig.json"
PIDRUDDERSET = 56
PIDSPEEDSET = 58
LOCKING = 12
IDELING = 8
DOCKING = 15
REMOTE = 25
BUOYIDALL = 1
LORAGETACK = 3
LORAINF = 6

# Utility Functions


def float_to_str(f):
    return format(f, '.10g')


def compute_nmea_crc(sentence: str) -> str:
    crc = 0
    for char in sentence:
        crc ^= ord(char)
    return f"{crc:02X}"


def generate_nmea_message(base_message: str) -> str:
    """
    Generate full NMEA message string with CRC.
    $IDr,IDs,ack,msg,data*chk
    """
    base_message = f"{BUOYIDALL},1," + base_message
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
            serial_port.write((full_message + '\n').encode())
        else:
            print("Serial port not available or not open.")
    except serial.SerialException as e:
        print(f"Serial send failed: {e}")


def send_udp_broadcast(values, pid_id, group_name, serial_port=None):
    ordered_keys = ["Kps", "Kis", "Kds"] if group_name == "SpeedPid" else [
        "Kpr", "Kir", "Kdr"]
    try:
        ordered_values = [values[k] for k in ordered_keys]
    except KeyError:
        print("Error: Missing values for", group_name)
        return

    value_strs = [float_to_str(v) for v in ordered_values]
    base_message = f"{LORAINF},{pid_id},0," + ",".join(value_strs) + ",0,0,0"
    full_message = generate_nmea_message(base_message)
    send_message(full_message, serial_port)


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


# Main Widget
class PIDSender(QWidget):
    def __init__(self):
        self.last_rudder_value = None
        self.last_speed_value = None
        self.debounce_timer = QTimer()
        self.debounce_timer.setSingleShot(True)
        self.debounce_timer.timeout.connect(self.send_control_update)

        super().__init__()
        self.setWindowTitle("PID UDP Broadcaster")
        # Get screen geometry
        screen = QApplication.primaryScreen()
        screen_rect = screen.availableGeometry()
        # Calculate half screen width
        half_width = screen_rect.width() // 2
        height = screen_rect.height() // 2  # You can set height as you want
        self.resize(half_width, height)  # Set window size
        self.setWindowTitle("PID UDP Broadcaster")

        self.saved_values = load_values()
        self.serial_port = None
        layout = QVBoxLayout()
        # PID input groups
        self.SpeedPid_inputs = self.create_pid_input_group(
            "Speed (Kp, Ki, Kd)", ["Kps", "Kis", "Kds"], self.send_SpeedPid)
        layout.addWidget(self.SpeedPid_inputs["box"])

        self.RudderPid_inputs = self.create_pid_input_group(
            "Rudder (Kp, Ki, Kd)", ["Kpr", "Kir", "Kdr"], self.send_RudderPid)
        layout.addWidget(self.RudderPid_inputs["box"])

        # Restore saved values
        self.restore_values()

        # Serial display
        layout.addWidget(QLabel("Incoming Serial Data:"))
        self.serial_display = QTextEdit(readOnly=True)
        self.serial_display.setMinimumHeight(150)
        layout.addWidget(self.serial_display)

        # Rudder and Speed sliders
        layout.addWidget(QLabel("Rudder Control:"))
        self.rudder_slider_layout, self.rudder_label, self.rudder_slider = self.create_slider(
            "Rudder", self.on_slider_change)
        layout.addLayout(self.rudder_slider_layout)

        layout.addWidget(QLabel("Speed Control:"))
        self.speed_slider_layout, self.speed_label, self.speed_slider = self.create_slider(
            "Speed", self.on_slider_change)
        layout.addLayout(self.speed_slider_layout)

        # Control mode radio buttons
        layout.addWidget(self.create_mode_radio_buttons())

        self.setLayout(layout)

        self.setup_serial_port("COM60")

        # Serial read timer
        self.serial_timer = QTimer()
        self.serial_timer.timeout.connect(self.read_serial_data)
        self.serial_timer.start(200)

    def create_pid_input_group(self, title, var_names, callback):
        box = QGroupBox(title)
        vbox = QVBoxLayout()
        inputs = {}

        for var in var_names:
            hbox = QHBoxLayout()
            hbox.addWidget(QLabel(var))
            field = QLineEdit()
            field.setPlaceholderText("Enter float")
            hbox.addWidget(field)
            vbox.addLayout(hbox)
            inputs[var] = field

        send_button = QPushButton("Send")
        send_button.clicked.connect(callback)
        vbox.addWidget(send_button, alignment=Qt.AlignRight)

        box.setLayout(vbox)
        return {"box": box, "inputs": inputs}

    def create_slider(self, name, on_change):
        layout = QHBoxLayout()
        label = QLabel("0")
        slider = QSlider(Qt.Horizontal)
        slider.setRange(-100, 100)
        slider.setValue(0)
        slider.setTickInterval(10)
        slider.setTickPosition(QSlider.TicksBelow)

        slider.valueChanged.connect(lambda val: (
            label.setText(str(val)),
            on_change(name, val)
        ))

        layout.addWidget(slider)
        layout.addWidget(label)
        return layout, label, slider

    def on_slider_change(self, name, value):
        if name == "Rudder":
            if value != self.last_rudder_value:
                self.last_rudder_value = value
                self.debounce_timer.start(200)  # Reset debounce timer
        elif name == "Speed":
            if value != self.last_speed_value:
                self.last_speed_value = value
                self.debounce_timer.start(200)  # Reset debounce timer

    def create_mode_radio_buttons(self):
        box = QGroupBox("Control Mode")
        layout = QHBoxLayout()

        self.lock_radio = QRadioButton("Lock")
        self.remote_radio = QRadioButton("Remote")
        self.idle_radio = QRadioButton("Idle")
        self.docking_radio = QRadioButton("Docking")
        self.idle_radio.setChecked(True)

        self.mode_button_group = QButtonGroup(self)
        self.mode_button_group.addButton(self.lock_radio)
        self.mode_button_group.addButton(self.remote_radio)
        self.mode_button_group.addButton(self.idle_radio)
        self.mode_button_group.addButton(self.docking_radio)

        layout.addWidget(self.lock_radio)
        layout.addWidget(self.remote_radio)
        layout.addWidget(self.idle_radio)
        layout.addWidget(self.docking_radio)

        # Connect only once here
        self.mode_button_group.buttonClicked.connect(
            self.on_mode_button_clicked)

        box.setLayout(layout)
        return box

    def get_selected_mode(self):
        if self.lock_radio.isChecked():
            return "Lock"
        elif self.remote_radio.isChecked():
            return "Remote"
        elif self.idle_radio.isChecked():
            return "Idle"
        return None

    def restore_values(self):
        for group, fields in [("SpeedPid", self.SpeedPid_inputs["inputs"]), ("RudderPid", self.RudderPid_inputs["inputs"])]:
            values = self.saved_values.get(group, {})
            for key, field in fields.items():
                if key in values:
                    field.setText(str(values[key]))

    def get_values(self, inputs):
        return {k: float(v.text()) if v.text() else 0.0 for k, v in inputs.items()}

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

    def setup_serial_port(self, port_name):
        try:
            self.serial_port = serial.Serial(
                port=port_name, baudrate=115200, timeout=0.1)
            print(f"Serial port {port_name} opened.")
        except serial.SerialException as e:
            print(f"Failed to open serial port {port_name}: {e}")
            self.serial_port = None

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

    def send_control_update(self):
        if not self.remote_radio.isChecked():
            print("Control update skipped: mode is not Remote.")
            return

        rudder = self.last_rudder_value if self.last_rudder_value is not None else 0
        speed = self.last_speed_value if self.last_speed_value is not None else 0
        speedBb = speed+rudder
        if speedBb > 100:
            speedBb = 100
        elif speedBb < -100:
            speedBb = -100
        speedSb = speed-rudder
        if speedSb > 100:
            speedSb = 100
        elif speedSb < -100:
            speedSb = -100
        base_message = f"{LORAINF},{REMOTE},{speedBb},{speedSb}"
        full_message = generate_nmea_message(base_message)
        send_message(full_message, self.serial_port)

    def on_mode_button_clicked(self, button):
        # button is the QRadioButton clicked
        if button == self.lock_radio:
            base_message = f"{LORAGETACK},{LOCKING}"
        elif button == self.idle_radio:
            base_message = f"{LORAGETACK},{IDELING}"
        elif button == self.docking_radio:
            base_message = f"{LORAGETACK},{DOCKING}"
        else:
            return  # No action for other buttons (like Remote)

        full_message = generate_nmea_message(base_message)
        send_message(full_message, self.serial_port)

    def closeEvent(self, event):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PIDSender()
    window.show()
    sys.exit(app.exec_())
