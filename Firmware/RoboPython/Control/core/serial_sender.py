import serial

class SerialSender:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate)

    def send(self, message):
        self.ser.write(message.encode())

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
