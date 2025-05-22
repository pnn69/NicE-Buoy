import folium
import tkinter as tk
from tkinterweb import HtmlFrame
import os  # Add this import
import webbrowser

# Coordinates for the midpoint
latitude, longitude = 52.290500405282, 4.929717537261

# Create a map centered around the midpoint
m = folium.Map(location=[latitude, longitude], zoom_start=13)

# Add a marker at the specified coordinates
folium.Marker([latitude, longitude], popup="Midpoint").add_to(m)

# Save the map to an HTML file
map_file = "map.html"
m.save(map_file)
print(f"Saved map to: {os.path.abspath(map_file)}")
print(f"File exists: {os.path.isfile(map_file)}")
# Create the Tkinter window
root = tk.Tk()
root.title("OpenStreetMap with Marker")

# Create an HtmlFrame widget to display the map
frame = HtmlFrame(root, horizontal_scrollbar="auto")
frame.pack(fill="both", expand=True)

# Open the map in the default web browser
webbrowser.open("file://" + os.path.abspath(map_file))

# Run the Tk
