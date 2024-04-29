import serial
import bluetooth

esp32 = "NicE_Buoy_Control"
address = ""

devices = bluetooth.discover_devices()

for addr in devices:
    if esp32 == bluetooth.lookup_name(addr):
        address = addr
        break
    
port = 1
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((address,port))
sock.send("Hey")
sock.close
