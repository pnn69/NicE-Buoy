import serial
import time

try:
    ser = serial.Serial('COM15', 115200, timeout=1)
    print("Reading serial on COM15 for 12 seconds...")
    print("Please press the Reset button on your CYD board now to capture the boot progression!")
    
    # Read serial data continuously for 12 seconds
    start_time = time.time()
    while time.time() - start_time < 12:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(data.decode('utf-8', errors='ignore'), end='', flush=True)
        time.sleep(0.1)
        
    ser.close()
except Exception as e:
    print("\nError opening serial port:", e)
