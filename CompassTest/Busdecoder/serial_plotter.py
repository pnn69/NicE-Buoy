import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse
from collections import deque
import sys
import threading
import queue

# Set up command line arguments
parser = argparse.ArgumentParser(description='Serial Port Plotter')
parser.add_argument('--port', type=str, required=True, help='Serial port (e.g., COM3 or /dev/ttyUSB0)')
parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate (default: 115200)')
parser.add_argument('--max_points', type=int, default=200, help='Max points to display on the X-axis')
args = parser.parse_args()

try:
    ser = serial.Serial(args.port, args.baudrate, timeout=0.1)
    print(f"Connected to {args.port} at {args.baudrate} baud.")
except Exception as e:
    print(f"Error opening serial port: {e}")
    sys.exit(1)

# Thread-safe queue to pass data from serial thread to GUI
data_queue = queue.Queue()

# We will dynamically create buffers and lines based on the number of CSV columns
data_buffers = []
line_objects = []
fig, ax = plt.subplots()
plt.title(f"Serial Data on {args.port}")
plt.xlabel("Samples")
plt.ylabel("Value")

is_running = True

def serial_read_thread():
    """Reads lines from the serial port in a background thread."""
    global is_running
    while is_running:
        try:
            if ser.in_waiting:
                line_bytes = ser.readline()
                try:
                    line_str = line_bytes.decode('utf-8', errors='ignore').strip()
                    if line_str:
                        # Try parsing as comma-separated floats
                        values = [float(v) for v in line_str.split(',')]
                        data_queue.put(values)
                except ValueError:
                    # Ignore lines that don't match the expected CSV format
                    pass
        except serial.SerialException:
            is_running = False
            break

# Start the serial reader thread
thread = threading.Thread(target=serial_read_thread, daemon=True)
thread.start()

def update(frame):
    """Called periodically by FuncAnimation to update the plot."""
    global data_buffers, line_objects
    
    updated = False
    
    # Process all available data in the queue
    while not data_queue.empty():
        values = data_queue.get()
        
        # Initialize buffers on the first valid line of data
        if not data_buffers:
            for i in range(len(values)):
                data_buffers.append(deque([0.0]*args.max_points, maxlen=args.max_points))
                lobj, = ax.plot([], [], label=f"Value {i+1}")
                line_objects.append(lobj)
            ax.legend(loc='upper right')

        # Append new values if the count matches
        if len(values) == len(data_buffers):
            for i, val in enumerate(values):
                data_buffers[i].append(val)
            updated = True

    if updated:
        x_data = list(range(len(data_buffers[0])))
        for i, lobj in enumerate(line_objects):
            lobj.set_data(x_data, data_buffers[i])
        
        ax.relim()
        ax.autoscale_view()
        
    return line_objects

# Create the animation
ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

try:
    print("Starting plot. Close the window to stop.")
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    is_running = False
    ser.close()
    print("Exited.")