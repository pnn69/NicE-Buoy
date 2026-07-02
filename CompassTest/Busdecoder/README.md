# Serial Plotter Instructions

I have created a Python script that can monitor and plot data from a serial port in real-time.

## Prerequisites
Ensure you have the required Python libraries installed:
```cmd
pip install pyserial matplotlib
```

## Step 1: Update Firmware
Based on the reference code in `PIOCompass`, the device does not continuously output data over the Serial port. You need to update your ESP32 firmware (`main.cpp`) to output comma-separated values. 

Add the following line inside the `loop()` function (e.g., right before `strip.show();`):
```cpp
Serial.printf("%.2f,%.2f,%.2f,%.2f\n", g_data.magX, g_data.magY, g_data.magZ, g_data.heading);
```

## Step 2: Find your COM Port
Run the helper script to list available COM ports on your system:
```cmd
python list_ports.py
```

## Step 3: Run the Plotter
Run the plotter script with your COM port:
```cmd
python serial_plotter.py --port COM3
```
*(Replace `COM3` with your actual COM port).*

The script will automatically detect the number of comma-separated values coming over the serial port and plot each as a separate line.
