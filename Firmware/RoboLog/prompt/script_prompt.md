The user wants a Python script to do the following:
1.  **Receive UDP messages** from `RobobuoyTop`.
2.  **Display wind data in a windrose format** (a basic representation of wind direction on a circular canvas).
3.  **Display other parameters** (IDr, IDs, ACK, CMD, Status, Magnetic Dir (mDir), GPS Dir (gDir), Target Dir (tgDir), Target Dist (tgDist), Wind Dir (wDir), Wind StdDev (wStd), Bow Thruster (BB), Stern Thruster (SB), PID I-term (IP), PID R-term (IR), Sub Battery V, Sub Battery %, Latitude (Lat), Longitude (Lon), GPS Fix, GPS Satellites (GpsSat), Checksum) from `RobobuoyTop`'s UDP output. Note: IP and IR are typically from the sub buoy, but the request implied all parameters output by the top.
4.  **Use a Tkinter GUI** for display.
5.  **Store the script in a new directory `RoboLog`**.
6.  **Put this prompt in a subdirectory `/prompt`**.

The script `udp_monitor.py` will:
-   Listen for UDP packets on port 1001.
-   Parse incoming messages using regular expressions to extract relevant data points based on the format observed in `RoboCompute.cpp` (comma-separated with checksum).
-   Update a Tkinter GUI with the parsed numerical and text data.
-   Draw a simplified windrose on a Tkinter Canvas, showing the wind direction as a red arrow. The canvas will include cardinal directions (N, E, S, W).
-   Handle graceful shutdown when the GUI window is closed.