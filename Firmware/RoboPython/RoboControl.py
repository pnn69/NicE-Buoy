import folium
import math
import webbrowser
import os  # Add this import

# Coordinates
head = (52.28926899869115, 4.926260253375628)
port = (52.28969490726184, 4.934093649446976)
starboard = (52.29136300973761, 4.932715551989979)

# Create a map centered around the first coordinate
m = folium.Map(location=[52.2900, 4.9300], zoom_start=16)

# Add the coordinates as markers with names
folium.Marker(head, popup='Head (52.2893, 4.9263)').add_to(m)
folium.Marker(port, popup='Port (52.2897, 4.9341)').add_to(m)
folium.Marker(starboard, popup='Starboard (52.2914, 4.9327)').add_to(m)

# Function to add an arrow for wind direction
def add_wind_arrow(map_obj, location, direction, length=0.0005, arrow_length=0.0002):
    # Convert wind direction to radians
    rad = math.radians(direction)

    # Calculate the end point of the arrow
    lat, lon = location
    delta_lat = length * math.cos(rad)
    delta_lon = length * math.sin(rad)
    arrow_end = (lat + delta_lat, lon + delta_lon)

    # Add the main line of the arrow
    folium.PolyLine([location, arrow_end], color='blue', weight=5).add_to(map_obj)

    # Calculate the left wing of the arrow
    left_wing_direction = math.radians(direction + 135)
    left_wing_lat = arrow_end[0] + arrow_length * math.cos(left_wing_direction)
    left_wing_lon = arrow_end[1] + arrow_length * math.sin(left_wing_direction)
    left_wing_end = (left_wing_lat, left_wing_lon)

    # Calculate the right wing of the arrow
    right_wing_direction = math.radians(direction - 135)
    right_wing_lat = arrow_end[0] + arrow_length * math.cos(right_wing_direction)
    right_wing_lon = arrow_end[1] + arrow_length * math.sin(right_wing_direction)
    right_wing_end = (right_wing_lat, right_wing_lon)

    # Add the wings of the arrow
    folium.PolyLine([arrow_end, left_wing_end], color='blue', weight=5).add_to(map_obj)
    folium.PolyLine([arrow_end, right_wing_end], color='blue', weight=5).add_to(map_obj)

# Calculate the midpoint among "Head," "Port," and "Starboard"
midpoint = (
    (head[0] + port[0] + starboard[0]) / 3,
    (head[1] + port[1] + starboard[1]) / 3
)

# Add wind arrow at the midpoint with a direction of 190 degrees
add_wind_arrow(m, midpoint, 190)

# Save the map to an HTML file
m.save("openstreetmap_with_wind_arrow.html")

# If you want to directly view it in a notebook (if running in Jupyter)
# m  # Uncomment this line if running in Jupyter
# Open the map in the default web browser
webbrowser.open("file://" + os.path.abspath("openstreetmap_with_wind_arrow.html"))
