import sys
import os
import json
import socket
import serial
import serial.tools.list_ports
import tempfile
from PyQt5.QtCore import QObject, pyqtSignal, QTimer
from PyQt5.QtCore import QThread
from serial_handler import SerialWorker
from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen
from PyQt5.QtWidgets import QWidget


# self.ip_display.setText(str(new_ip_value))
# self.ir_display.setText(str(new_ir_value))

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QGroupBox, QTextEdit,
    QSlider, QRadioButton, QButtonGroup
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QTextCursor
from PyQt5.QtGui import QFont

# Constants
UDP_PORT = 1001
COMPORT = "COM60"
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
# BUOYIDALL = 0xb7a50840
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

# Utility Functions



class VerticalCenteredProgressBar(QWidget):
    def __init__(self, width=30, height=150, parent=None):
        super().__init__(parent)
        self._value = 0
        self.setFixedSize(width, height)

    def setValue(self, value):
        self._value = max(-100, min(100, value))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect()
        w = rect.width()
        h = rect.height()

        painter.setBrush(QBrush(Qt.lightGray))
        painter.setPen(QPen(Qt.darkGray))
        painter.drawRect(rect)

        mid_y = h / 2
        if self._value == 0:
            return

        length = (abs(self._value) / 100) * (h / 2)

        if self._value > 0:
            fill_rect = QRectF(0, mid_y - length, w, length)
            color = QColor(0, 180, 0)
        else:
            fill_rect = QRectF(0, mid_y, w, length)
            color = QColor(180, 0, 0)

        painter.setBrush(QBrush(color))
        painter.setPen(Qt.NoPen)
        painter.drawRect(fill_rect)











def float_to_str(f):
    return format(f, '.10g')


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
        IDr = int(fields[0])
        IDs = fields[1]
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

def decode(result):
    print(result)

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


def log(self, msg):
    self.serial_display.append(f"[LOG] {msg}")


def save_values(group_values):
    try:
        with tempfile.NamedTemporaryFile('w', delete=False, dir='.') as tf:
            json.dump(group_values, tf, indent=4)
            temp_name = tf.name
        os.replace(temp_name, SETTINGS_FILE)
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

telemetry_records = {}
class TelemetryRecord:
    def __init__(self, IDs):
        self.IDs = IDs
        
        # Fields from type 1 (command 51)
        self.dirMag = None
        self.dirGPS = None
        self.tgDir = None
        self.tgDist = None
        self.wDir = None
        self.wStd = None
        self.speedBB = None
        self.speedSb = None
        self.ip = None
        self.ir = None
        self.subAccuV = None
        self.subAccuP = None
        self.lat = None
        self.lng = None
        self.gpsFix = None
        self.gpsSat = None
        self.topAccuP = None  # field from type 2
        self.status = None
        self.checksum = None

    def update_type51(self, data, status, checksum):
        try:
            self.dirMag   = int(data[0])
            self.dirGPS   = int(data[1])
            self.tgDir    = int(data[2])
            self.tgDist   = float(data[3])
            self.wDir     = int(data[4])
            self.wStd     = float(data[5])
            self.speedBB  = float(data[6])
            self.speedSb  = float(data[7])
            self.ip       = float(data[8])
            self.ir       = float(data[9])
            self.subAccuV = float(data[10])
            self.subAccuP = float(data[11])
            self.lat      = float(data[12])
            self.lng      = float(data[13])
            self.gpsFix   = int(data[14])
            self.gpsSat   = int(data[15])
        except (ValueError, TypeError, IndexError) as e:
            print(f"[update_type51] Error parsing data: {e}")

        self.status = status
        self.checksum = checksum

    def update_type19(self, data, status, checksum):
        self.lat = data[0]
        self.lng = data[1]
        self.dirMag = data[2]
        self.wDir = data[3]
        self.wStd = data[4]
        self.topAccuP = data[5]
        self.subAccuP = data[6]
        self.gpsFix = data[7]
        self.gpsSat = data[8]
        self.status = status
        self.checksum = checksum

def update_fields(self, values: dict, group: str):
    """
    Update QLineEdit fields for given PID group with new values.
    `group` should be "SpeedPid" or "RudderPid".
    """
    if group == "SpeedPid":
        inputs = self.SpeedPid_inputs["inputs"]
    elif group == "RudderPid":
        inputs = self.RudderPid_inputs["inputs"]
    else:
        return

    for key, val in values.items():
        if key in inputs:
            inputs[key].setText(str(val))



def process_input(input_dict):
    IDs = input_dict.get('IDs')
    if not IDs:
        print("No IDs field found in input.")
        return
    
    # Create new record if ID not stored yet
    if IDs not in telemetry_records:
        telemetry_records[IDs] = TelemetryRecord(IDs)
    
    record = telemetry_records[IDs]
    
    command = input_dict.get('command')
    data = input_dict.get('data')
    status = input_dict.get('status')
    checksum = input_dict.get('checksum')
    
    if command == 51:
        record.update_type51(data, status, checksum)
        print(f"Updated record for ID {IDs} with command 51")
    elif command == 19:
        record.update_type19(data, status, checksum)
        print(f"Updated record for ID {IDs} with command 19")
    else:
        print(f"Unknown command {command} for ID {IDs}")




# Main Widget
class PIDSender(QWidget):
    def __init__(self):
        super().__init__()


        self.last_rudder_value = None
        self.last_speed_value = None
        self.debounce_timer = QTimer()
        self.debounce_timer.setSingleShot(True)
        self.debounce_timer.timeout.connect(self.send_control_update)

        self.serial_buffer = ""
        self.setWindowTitle("PID UDP Broadcaster")

        # Screen setup
        screen = QApplication.primaryScreen()
        screen_rect = screen.availableGeometry()
        half_width = screen_rect.width() // 2
        height = screen_rect.height() // 2
        self.resize(half_width, height)

        self.saved_values = load_values()
        self.serial_port = None

        # Main layout
        layout = QVBoxLayout()
        # Top horizontal layout for PID groups + IP/IR inputs
        top_layout = QHBoxLayout()

        # Extra fields for Speed PID inputs
        self.SpeedPid_inputs = self.create_pid_input_group(
            "Speed (Kp, Ki, Kd)", 
            ["Kps", "Kis", "Kds"], 
            self.send_SpeedPid,
            extra_fields=[("TGdist", "float"), ("Dir", "int")]
        )
        top_layout.addWidget(self.SpeedPid_inputs["box"])

        # BB/SB Progress Bars with Labels
        self.bb_bar = VerticalCenteredProgressBar(height=self.SpeedPid_inputs["box"].sizeHint().height())
        self.sb_bar = VerticalCenteredProgressBar(height=self.SpeedPid_inputs["box"].sizeHint().height())
        bb_label = QLabel("BB")
        bb_label.setAlignment(Qt.AlignCenter)
        sb_label = QLabel("SB")
        sb_label.setAlignment(Qt.AlignCenter)

        bb_layout = QVBoxLayout()
        bb_layout.addWidget(self.bb_bar)
        bb_layout.addWidget(bb_label)

        sb_layout = QVBoxLayout()
        sb_layout.addWidget(self.sb_bar)
        sb_layout.addWidget(sb_label)

        bars_layout = QHBoxLayout()
        bars_layout.addLayout(bb_layout)
        bars_layout.addSpacing(10)
        bars_layout.addLayout(sb_layout)

        top_layout.addLayout(bars_layout)

        # Extra fields for Rudder PID inputs
        self.RudderPid_inputs = self.create_pid_input_group(
            "Rudder (Kp, Ki, Kd)", 
            ["Kpr", "Kir", "Kdr"], 
            self.send_RudderPid,
            extra_fields=[("mDir", "int"), ("Vbatt", "float")]
        )
        top_layout.addWidget(self.RudderPid_inputs["box"]) 

        # IP/IR inputs group
        ip_ir_group = QGroupBox("IP / IR Values")
        ip_ir_layout = QVBoxLayout()

        ip_layout = QHBoxLayout()
        ip_ir_layout.addWidget(QLabel("IP:"))
        self.ip_display = QLabel("0.0")
        ip_ir_layout.addWidget(self.ip_display)

        ip_ir_layout.addSpacing(20)

        ip_ir_layout.addWidget(QLabel("IR:"))
        self.ir_display = QLabel("0.0")
        ip_ir_layout.addWidget(self.ir_display)

        ip_ir_group.setLayout(ip_ir_layout)
        top_layout.addWidget(ip_ir_group)

        layout.addLayout(top_layout)

        self.setLayout(layout)
        self.restore_values()


        # Power and Declination layout
        power_declination_layout = QHBoxLayout()

        power_declination_layout.addWidget(QLabel("Max Power (-100 to 100):"))
        self.max_power_input = QLineEdit()
        self.max_power_input.setPlaceholderText("Enter max power")
        power_declination_layout.addWidget(self.max_power_input)

        power_declination_layout.addSpacing(20)

        power_declination_layout.addWidget(QLabel("Min Power (-100 to 100):"))
        self.min_power_input = QLineEdit()
        self.min_power_input.setPlaceholderText("Enter min power")
        power_declination_layout.addWidget(self.min_power_input)

        power_declination_layout.addSpacing(20)

        power_declination_layout.addWidget(QLabel("Declination (°):"))
        self.declination_input = QLineEdit()
        self.declination_input.setPlaceholderText("Enter declination float")
        power_declination_layout.addWidget(self.declination_input)

        layout.addLayout(power_declination_layout)

        # Load saved values
        self.max_power_input.setText(
            str(self.saved_values.get("MaxPower", 0.0)))
        self.min_power_input.setText(
            str(self.saved_values.get("MinPower", 0.0)))
        self.declination_input.setText(
            str(self.saved_values.get("Declination", 0.0)))

        # Buttons layout for Send Power Limits and Send Declination buttons
        buttons_layout = QHBoxLayout()

        self.send_power_button = QPushButton("Send Power Limits")
        self.send_power_button.clicked.connect(self.send_power_limits)
        buttons_layout.addWidget(
            self.send_power_button, alignment=Qt.AlignLeft)

        buttons_layout.addStretch(1)

        self.send_declination_button = QPushButton("Send Declination")
        self.send_declination_button.clicked.connect(self.send_declination)
        buttons_layout.addWidget(
            self.send_declination_button, alignment=Qt.AlignRight)

        layout.addLayout(buttons_layout)

        # Serial display
        layout.addWidget(QLabel("Incoming Serial Data:"))
        self.serial_display = QTextEdit(readOnly=True)
        self.serial_display.setMinimumHeight(150)
        layout.addWidget(self.serial_display)

        # Rudder slider
        layout.addWidget(QLabel("Rudder Control:"))
        self.rudder_slider_layout, self.rudder_label, self.rudder_slider = self.create_slider(
            "Rudder", self.on_slider_change)
        layout.addLayout(self.rudder_slider_layout)

        # Speed slider
        layout.addWidget(QLabel("Speed Control:"))
        self.speed_slider_layout, self.speed_label, self.speed_slider = self.create_slider(
            "Speed", self.on_slider_change)
        layout.addLayout(self.speed_slider_layout)

        # Distance and Direction inputs with send button
        distance_direction_layout = QHBoxLayout()

        distance_direction_layout.addWidget(QLabel("Distance:"))
        self.distance_input = QLineEdit("10")  # Default value
        distance_direction_layout.addWidget(self.distance_input)

        distance_direction_layout.addWidget(QLabel("Direction:"))
        self.direction_input = QLineEdit("180")  # Default value
        distance_direction_layout.addWidget(self.direction_input)

        self.send_distance_direction_button = QPushButton(
            "Send Distance/Direction")
        self.send_distance_direction_button.clicked.connect(
            self.send_distance_direction)
        distance_direction_layout.addWidget(
            self.send_distance_direction_button)

        layout.addLayout(distance_direction_layout)

        # Control mode radio buttons
        layout.addWidget(self.create_mode_radio_buttons())

        self.setLayout(layout)

        self.setup_serial_port(COMPORT)
        # Create and start serial worker in a thread
        self.serial_thread = QThread()
        self.serial_worker = SerialWorker(COMPORT)
        self.serial_worker.moveToThread(self.serial_thread)
        self.serial_thread.started.connect(self.serial_worker.start)
        # Connect signals to update display
        self.serial_worker.data_received.connect(self.update_serial_display)
        self.serial_worker.ip_ir_updated.connect(self.update_ip_ir_display)
        self.serial_thread.start()

        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            print(p.device)

        self.serial_timer = QTimer()
        self.serial_timer.timeout.connect(self.read_serial_data)
        self.serial_timer.start(200)

    def update_thruster_bars(self, bb_value: float, sb_value: float):
        self.bb_bar.setValue(int(bb_value))
        self.sb_bar.setValue(int(sb_value))

    def update_ip_ir_display(self, IDs):
        if IDs in telemetry_records:
            record = telemetry_records[IDs]
            # Defensive: check if fields are not None
            ip_val = record.ip if record.ip is not None else 0.0
            ir_val = record.ir if record.ir is not None else 0.0
            
            self.ip_display.setText(str(ip_val))
            self.ir_display.setText(str(ir_val))
        else:
            # No data for this ID, show default
            self.ip_display.setText("0.0")
            self.ir_display.setText("0.0")

    def update_serial_display(self, text):
        self.serial_display.append(text)

    def update_ip_ir_display(self, ip, ir):
        self.ip_display.setText(f"{ip:.2f}")
        self.ir_display.setText(f"{ir:.2f}")

    def create_pid_input_group(self, title, var_names, callback, extra_fields=None):
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

        if extra_fields:
            extra_hbox = QHBoxLayout()
            for label, placeholder in extra_fields:
                extra_hbox.addWidget(QLabel(label))
                field = QLineEdit()
                field.setPlaceholderText(placeholder)
                extra_hbox.addWidget(field)
                inputs[label] = field  # add to same dict
            send_button = QPushButton("Send")
            send_button.clicked.connect(callback)
            extra_hbox.addWidget(send_button)
            vbox.addLayout(extra_hbox)
        else:
            send_button = QPushButton("Send")
            send_button.clicked.connect(callback)
            vbox.addWidget(send_button, alignment=Qt.AlignRight)

        box.setLayout(vbox)
        return {"box": box, "inputs": inputs}
    

    def create_slider(self, name, on_change):
        def format_slider_value(val):
            if val < 0:
                return f"-{abs(val):02d}"
            else:
                return f"{val:02d}"

        layout = QHBoxLayout()
        label = QLabel("00")  # default as two digits
        slider = QSlider(Qt.Horizontal)
        slider.setRange(-100, 100)
        slider.setValue(0)
        slider.setTickInterval(10)
        slider.setTickPosition(QSlider.TicksBelow)

        slider.valueChanged.connect(lambda val: (
            label.setText(format_slider_value(val)),
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

    def update_new_fields_from_record(self, record):
        # Extract only the new fields from record for Speed
        speed_new_values = {
            "target_distance": getattr(record, "target_distance", None),
            "direction": getattr(record, "direction", None),
        }
        speed_new_values = {k: v for k, v in speed_new_values.items() if v is not None}

        # Update Speed inputs
        for key, val in speed_new_values.items():
            if key in self.SpeedPid_inputs["inputs"]:
                self.SpeedPid_inputs["inputs"][key].setText(str(val))

        # Extract only the new fields from record for Rudder
        rudder_new_values = {
            "mDir": getattr(record, "mDir", None),
            "Vbatt": getattr(record, "Vbatt", None),
        }
        rudder_new_values = {k: v for k, v in rudder_new_values.items() if v is not None}

        # Update Rudder inputs
        for key, val in rudder_new_values.items():
            if key in self.RudderPid_inputs["inputs"]:
                self.RudderPid_inputs["inputs"][key].setText(str(val))




    def update_extra_fields(self, tgdist, direction, mdir, bat):
        def safe_float(value, digits=1):
            try:
                return f"{float(value):.{digits}f}"
            except (TypeError, ValueError):
                return "—"

        def safe_int(value):
            try:
                return str(int(value))
            except (TypeError, ValueError):
                return "—"

        # Speed section fields
        if "TGdist" in self.SpeedPid_inputs["inputs"]:
            self.SpeedPid_inputs["inputs"]["TGdist"].setText(safe_float(tgdist, 1))
        if "Dir" in self.SpeedPid_inputs["inputs"]:
            self.SpeedPid_inputs["inputs"]["Dir"].setText(safe_int(direction))

        # Rudder section fields
        if "mDir" in self.RudderPid_inputs["inputs"]:
            self.RudderPid_inputs["inputs"]["mDir"].setText(safe_int(mdir))
        if "Vbatt" in self.RudderPid_inputs["inputs"]:
            self.RudderPid_inputs["inputs"]["Vbatt"].setText(safe_float(bat, 1))














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
                    self.serial_buffer += data  # append to buffer

                    while '\n' in self.serial_buffer:
                        line, self.serial_buffer = self.serial_buffer.split(
                            '\n', 1)
                        line = line.strip()

                        if line:
                            self.serial_display.moveCursor(QTextCursor.End)
                            self.serial_display.insertPlainText(line + '\n')
                            self.serial_display.verticalScrollBar().setValue(
                                self.serial_display.verticalScrollBar().maximum())
                            while ',,' in line:
                                        line = line.replace(',,', ',0,')
                            # Handle special case if line ends with a comma
                            if line.endswith(','):
                                line += '0'




                            result = parse_serial_line(line)
                            if result is None:
                                print(f"Error: {line}")
                            else:
                                process_input(result)
                                record = telemetry_records.get(result['IDs'])
                                if record is not None:
                                    pid_sender_instance.update_ip_ir_display(record.ip, record.ir)
                                    pid_sender_instance.update_thruster_bars(record.speedBB, record.speedSb)
                                    pid_sender_instance.update_extra_fields(
                                        tgdist=record.tgDist,      # Replace with actual field name
                                        direction=record.tgDir,      # Replace with actual field name
                                        mdir=record.dirMag,          # Replace with actual field name
                                        bat=record.subAccuV             # Replace with actual field name
                                    )


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

    def send_power_limits(self):
        try:
            max_power = int(self.max_power_input.text())
            min_power = int(self.min_power_input.text())
        except ValueError:
            print("Invalid max or min power value.")
            return

        # Clamp values between -100 and 100
        max_power = max(-100, min(100, max_power))
        min_power = max(-100, min(100, min_power))

        # Save to config
        self.saved_values["MaxPower"] = max_power
        self.saved_values["MinPower"] = min_power
        save_values(self.saved_values)

        # Prepare message and send
        base_message = f"{LORAINF},{MAXMINPWRSET},0,{max_power},{min_power},0,0"
        full_message = generate_nmea_message(base_message)
        send_message(full_message, self.serial_port)

    def send_declination(self):
        try:
            declination = float(self.declination_input.text())
        except ValueError:
            print("Invalid declination value.")
            return

        self.saved_values["Declination"] = declination
        save_values(self.saved_values)

        base_message = f"{LORAINF},{STORE_DECLINATION},{IDLE},{float_to_str(declination)}"
        full_message = generate_nmea_message(base_message)
        send_message(full_message, self.serial_port)
        print(f"Declination sent: {declination}")

    def on_mode_button_clicked(self, button):
        # button is the QRadioButton clicked
        if button == self.lock_radio:
            base_message = f"{LORAGETACK},{LOCKING},{IDLE}"
        elif button == self.idle_radio:
            base_message = f"{LORAGETACK},{IDELING},{IDLE}"
        elif button == self.docking_radio:
            base_message = f"{LORAGETACK},{DOCKING},{IDLE}"
        else:
            return  # No action for other buttons (like Remote)

        full_message = generate_nmea_message(base_message)
        send_message(full_message, self.serial_port)

    def send_distance_direction(self):
        try:
            distance = float(self.distance_input.text())
            direction = float(self.direction_input.text())
        except ValueError:
            print("Invalid distance or direction input.")
            return

        # Construct and send the message
        base_message = f"{LORAINF},{DIRDIST},{IDLE},{direction},{distance}"
        full_message = generate_nmea_message(base_message)
        send_message(full_message, self.serial_port)
        print(f"Distance & Direction sent: {direction},{distance}")

    def closeEvent(self, event):
        if self.serial_worker:
            self.serial_worker.stop()
        if self.serial_thread:
            self.serial_thread.quit()
            self.serial_thread.wait()
        event.accept()


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)

    # Set a larger global font before creating the window
    font = QFont("Arial", 12)
    app.setFont(font)

    pid_sender_instance = PIDSender()
    pid_sender_instance.show()

    sys.exit(app.exec_())
