import tkinter as tk
from tkinter import Frame
from tkhtmlview import HTMLLabel

# URL of the HTML file to display
url = 'file:///C:/tmp/NicE-Buoy/Firmware/RoboPython/openstreetmap_with_wind_arrow.html'

# Create the main application window
root = tk.Tk()
root.title("OpenStreetMap Display")
root.geometry("800x600")  # Set the window size

# Create a frame for the HTML display
frame = Frame(root)
frame.pack(fill=tk.BOTH, expand=True)

# Create an HTMLLabel to display the HTML content
html_label = HTMLLabel(frame, html=f'<iframe src="{url}" width="100%" height="100%" frameborder="0"></iframe>')
html_label.pack(fill=tk.BOTH, expand=True)

# Start the application
root.mainloop()
