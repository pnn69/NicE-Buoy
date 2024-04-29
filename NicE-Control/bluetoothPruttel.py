import bluetooth
import socket

target_name = "0 lite"
target_name1 = "NicE_Buoy_Control"
target_address = None
target_address1 = None

nearby_devices = bluetooth.discover_devices(lookup_names=True,lookup_class=True)
print(nearby_devices)
for btaddr, btname, btclass in nearby_devices:
    if target_name == btname:
        target_address = btaddr
    if target_name1 == btname:
        target_address = btaddr


if target_address1 is not None:
    target_name = target_name1
    target_address = target_address1

if target_address is not None:
    print("found target {} bluetooth device with address {} ".format(target_name,target_address))
    
    serverMACAddress = target_address
    port = 1
    try:
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        s.connect((serverMACAddress,port))
        print("connected to {}".format(target_name))
        while 1:
            text = input()
            if text == "quit":
                break
            s.send(bytes(text, 'UTF-8'))
            data = s.recv(1024)
            if data:
                print(data)
        s.close()
    except TimeoutError:
        print("No port open")


else:
    print("could not find target bluetooth device nearby")