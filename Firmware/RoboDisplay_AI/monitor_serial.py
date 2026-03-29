#!/usr/bin/env python3
import serial
import sys
import time

try:
    ser = serial.Serial('COM7', 115200, timeout=1)
    time.sleep(2)  # Wait for connection
    
    print("Connected to COM7, waiting for device output...")
    print("=" * 60)
    
    for i in range(300):  # Read for ~30 seconds
        try:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').rstrip()
                if line:
                    print(line)
        except Exception as e:
            print(f"Error: {e}")
            break
    
    ser.close()
    print("=" * 60)
    print("Monitor terminated")
    
except Exception as e:
    print(f"Failed to open serial port: {e}")
    sys.exit(1)
