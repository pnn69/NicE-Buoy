import os
import random
from bluetooth import *

def read_temp():
    return random.random()

server_sock = BluetoothSocket(RFCOMM)
server_sock.bind(("", PORT_ANY))
server_sock.listen(1)
port = server_sock.getsockname()[1]

uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
advertise_service(server_sock, "TestServer", service_id=uuid,
                  service_classes=[uuid, SERIAL_PORT_CLASS],
                  profiles=[SERIAL_PORT_PROFILE])

while True:
    print(f"Waiting for connection on RFCOMM channel {port}")
    client_sock, client_info = server_sock.accept()
    print("Accepted connection from", client_info)

    try:
        data = client_sock.recv(1024)
        if len(data) == 0:
            break
        print(data)
        if data == b'temp':
            data = str(read_temp()) + '!'
            client_sock.send(data)
        else:
            data = 'WTF!'
            client_sock.send(data)

        print(f"sending [{data}]")
    except IOError:
        pass
    except KeyboardInterrupt:
        print("disconnected")
        client_sock.close()
        server_sock.close()
        print("all done")
        break
