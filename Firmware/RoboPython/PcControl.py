from tkinter import *
import time
import math
import webbrowser
import serial
import threading
import re
import compass
import socket
import IDS
from dataclasses import dataclass

        #print("IDs (Hex):", hex(field1))
        #print("IDr (Hex):", hex(field2))
        #print("Ack", field3)

@dataclass
class Robostruct:
    mac: int = 0
    IDs: int = 0
    IDr: int = 0
    cmd: int = 0
    ack: int = 0
    msg: int = 0
    status: int = 0
    lat: float = 0.0
    lng: float = 0.0
    tgLat: float = 0.0
    tgLng: float = 0.0
    dirSet: int = 0
    dirGps: int = 0
    wDir: float = 0.0
    wStd: float = 0.0
    dirMag: int = 0
    tgDir: int = 0
    trackPos: int = 0
    speed: int = 0
    speedBb: int = 0
    speedSb: int = 0
    speedSet: int = 0
    tgDist: float = 0.0
    subAccuV: float = 0.0
    topAccuV: float = 0.0
    subAccuP: int = 0
    topAccuP: int = 0
    cmd: int = 0
    minOfsetDist: int = 0
    maxOfsetDist: int = 0
    minSpeed: int = 0
    maxSpeed: int = 0
    compassOffset: int = 0
    lastLoraIn: int = 0
    lastLoraOut: int = 0
    lastUdpOut: int = 0
    lastUdpIn: int = 0
    gpsFix: bool = FALSE
    gpsSat: int = 0
    
   
font_settings = ("Arial", 8)  # Change the font name and size as needed

windowh = 350
windoww = 500


# Open COM7 port
ser = serial.Serial('COM60', 115200)  # Adjust baud rate as per your requirement

   
#***************************************************************************************************
#  Store data in struct
#***************************************************************************************************
def decode(robostruct,data):
    if robostruct.msg == 19 and len(data) >= 9: #52.32053467, 4.96557883, 0.0, 0.0, 0.0, 100, 0, 1, 9
        robostruct.lat = data[0]
        robostruct.lng = data[1]
        robostruct.mDir = round(data[2])
        robostruct.wDir = round(data[3])
        robostruct.wStd = data[4]
        robostruct.topAccuP = data[5]
        robostruct.subAccuP = data[6]
        robostruct.gpsFix = data[7]
        robostruct.gpsSat = data[8]    
        return
        
    if robostruct.msg == 21 and len(data) >= 4: #52.3204735, 4.965538333, 0.0, 0.0
        robostruct.tgLat = data[0]
        robostruct.tgLng = data[1]
        robostruct.wDir = data[2]
        robostruct.wStd = data[3]
        return

    if robostruct.msg == 44 and len(data) >= 2: #233.36, 0.93
        robostruct.tgDir = data[0]
        robostruct.tgDist = data[1]
        return

    if robostruct.msg == 55 and len(data) >= 2: #51,52.32048950,4.96559850,5.8,0.4
        robostruct.tgLat = data[0]
        robostruct.tgLng = data[1]
        return
    
    if robostruct.msg == 56:#80.0, 0, 0, 0, 0.0
        robostruct.speedSet = data[0]
        robostruct.speed = data[1]
        robostruct.speedBb = data[2]
        robostruct.speedSb = data[3]
        robostruct.subAccuV = data[4]
        return
    #54,80.00,80,80,-80,0.00
    if robostruct.msg == 56 and len(data) >= 5: 
        robostruct.speedSet = data[0]
        robostruct.speed = data[1]
        robostruct.speedBb = data[2]
        robostruct.speedSb = data[3]
        robostruct.subAccuV = data[4]
        return

    print("Unknown data ")

#***************************************************************************************************
#  Checkum validation
#***************************************************************************************************
def validate_and_extract_data(input_string):
    #print(f"String in: {input_string}")
    if '*' not in input_string:
        return -1
    data, checksum_str = input_string.split('*')
    # Remove the initial '$' from data if present
    if data.startswith('$'):
        data = data[1:]
    try:
        checksum = int(checksum_str, 16)  # Interpret checksum as hexadecimal
    except ValueError:
        return -1  # Invalid checksum format
    calculated_checksum = 0
    for char in data:
        calculated_checksum ^= ord(char)
    #print(f"Calculated checksum (hex): {hex(calculated_checksum)}")
    #print(f"Provided checksum (hex): {hex(checksum)}")
    if calculated_checksum == checksum:
        return data  # Checksum matches, return data
    else:
        return -1  # Checksum does not match

#***************************************************************************************************
#  Decoder version 2
#  IDr,IDs,ack,msg,data
#***************************************************************************************************
def decode_V2(input_str):
    # Remove angle brackets and split by commas
    input_str = input_str.strip("<>")
    fields = input_str.split(",")
    
    try:
        # Try to decode the first four fields
        field1 = int(fields[0], 16)  # IDr
        field2 = int(fields[1], 16)  # IDs
        field3 = int(fields[2])      # Ack
        field4 = int(fields[3])      # msg

        # Convert the remaining fields to `data`
        data = [float(f.strip()) if '.' in f.strip() else int(f.strip()) for f in fields[5:]]
        #print("IDr (Hex):", hex(field1))
        #print("IDs (Hex):", hex(field2))
        #print("Ack", field3)
        #print("msg", field4)
        #print("Data:", data)
        #print("Nr of data fields:",  len(data))

        if robostructs[0].IDs == 0 or robostructs[0].IDs == field2:
            robostructs[0].IDs = field2
            robostructs[0].ack = field3
            robostructs[0].msg = field4
            decode(robostructs[0], data)
            #return
        elif robostructs[1].IDs == 0 or robostructs[1].IDs == field2:
            robostructs[1].IDs = field2
            robostructs[1].ack = field3
            robostructs[1].msg = field4
            decode(robostructs[1], data)
            #return
        elif robostructs[2].IDs == 0 or robostructs[2].IDs == field2:
            robostructs[2].IDs = field2
            robostructs[2].ack = field3
            robostructs[2].msg = field4
            decode(robostructs[2], data)
        return
    except ValueError as e:
        # Print an error message and return None or some default values
        print(f"Error decoding input: {e}")
        return None


def read_serial():
    while True:
        if ser.in_waiting > 0:
            try:
                data_in = ser.readline().decode().strip()
            except UnicodeDecodeError as e:
                data_in = "troep"
            if data_in != "troep":
                received_data = validate_and_extract_data(data_in)
                if received_data != -1:
                    print(received_data)
                    decode_V2(received_data)
                else:
                    print(data_in, ' # No valid data: ') 

def start_serial_thread():
    serial_thread = threading.Thread(target=read_serial)
    serial_thread.daemon = True
    serial_thread.start()

#def start_udp_thread():
#    while True:
#        data = sock.recv(2048)
#        print(data)

    
   
def start_compass_thread():
    serial_thread = threading.Thread(target=write_compass)
    serial_thread.daemon = True
    serial_thread.start()


def write_compass():
    compass.root.mainloop()
# Thread to run the compass
def start_compass_thread():
    compass_thread = threading.Thread(target=write_compass)
    compass_thread.daemon = True
    compass_thread.start()


root = Tk()
root.title("Nice buoy control!")
frame = Frame(root, width=windoww, height=windowh)
frame.pack()
statuslb = Label(text="STATUS:")
statuslb.place(x=windoww/2-20, y=10)
status_txt = Label(text="?")
status_txt.place(x=windoww/2+30, y=10)
dist_txt = Label(text="")
dist_txt.place(x=windoww/2-20, y=30)

# Initialize the list with four Robostruct instances
robostructs = [
    Robostruct(),
    Robostruct(),
    Robostruct(),
    Robostruct(),  # Fourth instance with default values
]
start_serial_thread()  # Start the serial reading thread
root.mainloop()
