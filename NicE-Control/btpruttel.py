import serial
import bluetooth

# Function to discover nearby Bluetooth devices
def discover_devices():
    nearby_devices = bluetooth.discover_devices(duration=8, lookup_names=True, flush_cache=True, lookup_class=False)
    return nearby_devices

# Function to connect to a Bluetooth device
def connect_to_device(device_address):
    port = 1  # RFCOMM port for serial communication
    socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    socket.connect((device_address, port))
    return socket

# Function to read data from the serial terminal
def read_from_serial(socket):
    while True:
        data = socket.recv(1024)  # Adjust buffer size as needed
        print(data.decode('utf-8'))  # Decode data and print

# Function to write data to the serial terminal
def write_to_serial(socket, data):
    socket.send(data.encode('utf-8'))  # Encode data and send

if __name__ == "__main__":
    # Discover nearby Bluetooth devices
    devices = discover_devices()
    for device_address, device_name in devices:
        print("Found:", device_name, "(", device_address, ")")
    
    # Select a device to connect
    selected_device_address = input("Enter the Bluetooth address of the device to connect: ")

    # Connect to the selected device
    try:
        socket = connect_to_device(selected_device_address)
        print("Connected to", selected_device_address)
        
        # Read data from the serial terminal in a separate thread or process
        # Example usage:
        # import threading
        # reader_thread = threading.Thread(target=read_from_serial, args=(socket,))
        # reader_thread.start()
        
        # Write data to the serial terminal
        while True:
            user_input = input("Enter data to send (or 'exit' to quit): ")
            if user_input.lower() == 'exit':
                break
            write_to_serial(socket, user_input)
        
        socket.close()  # Close the Bluetooth socket when done
    except Exception as e:
        print("Error:", e)
