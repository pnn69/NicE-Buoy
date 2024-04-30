from bluetooth import *
def hallo():
    print("Halllo!")

def read_bleutooth():
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
            while True:
                # Receive data from the client
                data = client_sock.recv(1024)
                if not data:
                    break
                
                # Print the received data
                print("Received:", data.decode("utf-8"))
        except KeyboardInterrupt:
            pass
        finally:
            # Close the client socket
            client_sock.close()
            # Close the server socket
            server_sock.close()
