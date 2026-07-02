import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
if not ports:
    print("No serial ports found.")
else:
    print("Available serial ports:")
    for port in ports:
        print(f"  - {port.device}: {port.description}")
