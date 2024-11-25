import folium
import math
import webbrowser
import os
import ctypes

# dir= 80
STARBOARD = (52.320421667000,4.965367500000)
PORT = (52.320433000000,4.965470333000)
HEAD = (0.000000000000,0.000000000000)
midpoint =(52.320427333500,4.965418916500)
winddir =(108.9)
#Send track info
PORTn = (52.320457546570,4.965435839743)
#Lora out<$0,2,3,20,0,0.0000000000,0.0000000000*03>
STARBOARDn = (52.320397120430,4.965401993257)
HEADn = (0.000000000000,0.000000000000)






m = folium.Map(location=midpoint, zoom_start=16)
# Create a map centered around the first coordinate


# Add the coordinates as markers with names
folium.Marker(HEAD, popup='Head' + str(HEAD),icon=folium.Icon( color="darkblue")).add_to(m)
folium.Marker(PORT, popup='Port' + str(PORT),icon=folium.Icon(color="darkred")).add_to(m)
folium.Marker(STARBOARD, popup='Starboard\r' + str(STARBOARD),icon=folium.Icon(color="darkgreen")).add_to(m)
#folium.Marker(midpoint).add_to(m)
#folium.Marker(startlincp,icon=folium.Icon(color="black")).add_to(m)
folium.Marker(HEADn, popup='HEAD' + str(HEADn),icon=folium.Icon(icon ='flag' , color="lightblue")).add_to(m)
folium.Marker(PORTn, popup='PORT' + str(PORTn),icon=folium.Icon(icon ='flag' , color="lightred")).add_to(m)
folium.Marker(STARBOARDn, popup='STARBOURD' + str(STARBOARDn),icon=folium.Icon(icon ='flag' , color="lightgreen")).add_to(m)

# Invert the wind direction
inverted_winddir = (winddir + 180) % 360  # Add 180 degrees and wrap around if necessary
old_windir = (250 + 180) %360
# Function to add an arrow for wind direction
def add_wind_arrow(map_obj, location, direction, length=0.0005, arrow_length=0.0002):
    # Convert wind direction to radians
    rad = math.radians(direction)

    # Calculate the end point of the arrow based on the wind direction
    lat, lon = location
    delta_lat = length * math.cos(rad)  # Correctly adjusting latitude
    delta_lon = length * math.sin(rad)  # Correctly adjusting longitude
    arrow_end = (lat + delta_lat, lon + delta_lon)

    # Add the main line of the arrow
    folium.PolyLine([location, arrow_end], color='blue', weight=5).add_to(map_obj)

    # Calculate the left wing of the arrow
    left_wing_direction = rad + math.radians(135)  # Maintain the angle for left wing
    left_wing_lat = arrow_end[0] + arrow_length * math.cos(left_wing_direction)
    left_wing_lon = arrow_end[1] + arrow_length * math.sin(left_wing_direction)
    left_wing_end = (left_wing_lat, left_wing_lon)

    # Calculate the right wing of the arrow
    right_wing_direction = rad - math.radians(135)  # Maintain the angle for right wing
    right_wing_lat = arrow_end[0] + arrow_length * math.cos(right_wing_direction)
    right_wing_lon = arrow_end[1] + arrow_length * math.sin(right_wing_direction)
    right_wing_end = (right_wing_lat, right_wing_lon)

    # Add the wings of the arrow
    folium.PolyLine([arrow_end, left_wing_end], color='blue', weight=5).add_to(map_obj)
    folium.PolyLine([arrow_end, right_wing_end], color='blue', weight=5).add_to(map_obj)

# Calculate the midpoint among "Head," "Port," and "Starboard"
#midpoint = (52.29092383,4.92974225)
    #(head[0] + port[0] + starboard[0]) / 3,
    #(head[1] + port[1] + starboard[1]) / 3
#)

def add_wind_arrow_old(map_obj, location, direction, length=0.0005, arrow_length=0.0002):
    # Convert wind direction to radians
    rad = math.radians(direction)

    # Calculate the end point of the arrow based on the wind direction
    lat, lon = location
    delta_lat = length * math.cos(rad)  # Correctly adjusting latitude
    delta_lon = length * math.sin(rad)  # Correctly adjusting longitude
    arrow_end = (lat + delta_lat, lon + delta_lon)

    # Add the main line of the arrow
    folium.PolyLine([location, arrow_end], color='grey', weight=5).add_to(map_obj)

    # Calculate the left wing of the arrow
    left_wing_direction = rad + math.radians(135)  # Maintain the angle for left wing
    left_wing_lat = arrow_end[0] + arrow_length * math.cos(left_wing_direction)
    left_wing_lon = arrow_end[1] + arrow_length * math.sin(left_wing_direction)
    left_wing_end = (left_wing_lat, left_wing_lon)

    # Calculate the right wing of the arrow
    right_wing_direction = rad - math.radians(135)  # Maintain the angle for right wing
    right_wing_lat = arrow_end[0] + arrow_length * math.cos(right_wing_direction)
    right_wing_lon = arrow_end[1] + arrow_length * math.sin(right_wing_direction)
    right_wing_end = (right_wing_lat, right_wing_lon)

    # Add the wings of the arrow
    folium.PolyLine([arrow_end, left_wing_end], color='grey', weight=5).add_to(map_obj)
    folium.PolyLine([arrow_end, right_wing_end], color='grey', weight=5).add_to(map_obj)

# Calculate the midpoint among "Head," "Port," and "Starboard"
#midpoint = (52.29092383,4.92974225)
    #(head[0] + port[0] + starboard[0]) / 3,
    #(head[1] + port[1] + starboard[1]) / 3
#)


# Add wind arrow at the midpoint with the inverted wind direction
add_wind_arrow_old(m, midpoint, old_windir)
add_wind_arrow(m, midpoint, inverted_winddir)

# Save the map to an HTML file
output_file = "openstreetmap_with_wind_arrow.html"
m.save(output_file)

# Open the map in the default web browser
webbrowser.open('file://' + os.path.realpath(output_file))
