from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from ui.ui_elements import create_slider, create_labeled_input
from core.udp_sender import UDPSender
from core.serial_sender import SerialSender


class PIDSender(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robobuoy PID Controller")

        self.sender = UDPSender("192.168.1.100", 5005)
        # self.serial_sender = SerialSender("/dev/ttyUSB0", 115200)
        self.serial_sender = SerialSender("COM60", 115200)

        main_layout = QVBoxLayout()

        self.rudder_layout, self.rudder_slider, _ = create_slider(
            "Rudder", -100, 100, 0)
        self.speed_layout, self.speed_slider, _ = create_slider(
            "Speed", -100, 100, 0)
        main_layout.addLayout(self.rudder_layout)
        main_layout.addLayout(self.speed_layout)

        self.kp_layout, self.kp_input = create_labeled_input("Kp", "1.0")
        self.ki_layout, self.ki_input = create_labeled_input("Ki", "0.1")
        self.kd_layout, self.kd_input = create_labeled_input("Kd", "0.01")
        main_layout.addLayout(self.kp_layout)
        main_layout.addLayout(self.ki_layout)
        main_layout.addLayout(self.kd_layout)

        send_btn = QPushButton("Send Settings")
        send_btn.clicked.connect(self.send_settings)
        main_layout.addWidget(send_btn)

        self.setLayout(main_layout)

    def send_settings(self):
        kp = self.kp_input.text()
        ki = self.ki_input.text()
        kd = self.kd_input.text()
        rudder = self.rudder_slider.value()
        speed = self.speed_slider.value()

        message = f"Kp:{kp},Ki:{ki},Kd:{kd},Rudder:{rudder},Speed:{speed}"
        self.sender.send(message)
        self.serial_sender.send(message)
