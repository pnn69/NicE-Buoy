
import serial
import time
import os

def calculate_crc(data):
    crc = 0
    for char in data:
        crc ^= ord(char)
    return crc

def send_command(ser, command):
    crc = calculate_crc(command)
    full_command = f"${command}*{crc:02X}" + os.linesep
    print(f"Sending: {full_command.strip()}")
    ser.write(full_command.encode())

def main():
    port = "COM30"
    buoy_idr = "1"
    buoy_ids = "63" # 99 in hex
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        print(f"Connected to {port}")
    except serial.SerialException as e:
        print(f"Error opening port {port}: {e}")
        return

    # Send SETUPDATA command with correct hex IDs and enough parameters
    setup_command = f"{buoy_idr},{buoy_ids},1,84,0,0,0,0,0,0,0,0,0,0,0,0"
    send_command(ser, setup_command)

    time.sleep(1) # Wait for a response

    # Read response
    response = ser.readline().decode('latin-1').strip()
    if response:
        print(f"Received: {response}")
        # Basic validation
        if response.startswith('$') and '*' in response:
            content, received_crc_str = response[1:].split('*')
            
            try:
                received_crc = int(received_crc_str, 16)
                calculated_crc = calculate_crc(content)
                
                if received_crc == calculated_crc:
                    print("CRC check PASSED")
                else:
                    print(f"CRC check FAILED. Received: {received_crc}, Calculated: {calculated_crc}")
                    
            except ValueError:
                print("Error parsing received CRC.")
                
            fields = content.split(',')
            if len(fields) >= 14 and fields[3] == "84":
                print("SETUPDATA response received.")
                # Further checks can be added here
            else:
                print("Did not receive a valid SETUPDATA response.")
        else:
            print("Invalid response format.")
    else:
        print("No response received.")

    ser.close()
    print(f"Disconnected from {port}")

if __name__ == "__main__":
    main()
