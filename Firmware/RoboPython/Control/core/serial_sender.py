import serial

class SerialSender:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate)

    def send(self, message):
        self.ser.write(message.encode())
