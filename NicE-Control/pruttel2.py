import serial
import re

# Open COM7 port
ser = serial.Serial('COM7', 115200)  # Adjust baud rate as per your requirement

def decode_message(message):
    # Regular expression pattern to match the message format: <sender><status><msgId><ack><msg>
    pattern = r'<(\d+)><(\d+)><(\d+)><(\d+)><(.+)>'
    # Match the pattern in the message
    match = re.match(pattern, message)
    if match:
        sender = match.group(1)
        status = match.group(2)
        msg_id = match.group(3)
        ack = match.group(4)
        msg = match.group(5)
        print("Sender:", sender)
        print("Status:", status)
        print("Message ID:", msg_id)
        print("Acknowledge:", ack)
        print("Message:", msg)
    else:
        print("Invalid message format:", message)

try:
    while True:
        if ser.in_waiting > 0:
            received_data = ser.readline().decode().strip()
            print("Received:", received_data)
            decode_message(received_data)
except KeyboardInterrupt:
    ser.close()
