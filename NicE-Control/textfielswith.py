from tkinter import *
from math import sin, cos, radians

# Function to update the compass pointers
def update_pointers(buoy_heading, target_heading, gps_heading):
    canvas.delete("pointers")  # Clear previous pointers
    
    # Draw buoy heading pointer (red)
    draw_pointer(buoy_heading, "red")
    
    # Draw target heading pointer (blue)
    draw_pointer(target_heading, "blue")
    
    # Draw GPS heading pointer (green)
    draw_pointer(gps_heading, "green")

# Function to draw a pointer at a specific angle and color
def draw_pointer(angle, color):
    x_center = 150  # X-coordinate of the center of the compass
    y_center = 150  # Y-coordinate of the center of the compass
    length = 100    # Length of the pointer
    pointer_width = 2  # Width of the pointer
    arrow_size = 8  # Size of the arrowhead
    
    # Calculate the endpoint of the pointer based on the angle
    x_end = x_center + length * sin(radians(angle))
    y_end = y_center - length * cos(radians(angle))
    
    # Draw the pointer line on the canvas
    canvas.create_line(x_center, y_center, x_end, y_end, width=pointer_width, fill=color, tags="pointers")
    
    # Calculate the coordinates for the arrowhead
    x1 = x_end - arrow_size * sin(radians(angle + 150))
    y1 = y_end + arrow_size * cos(radians(angle + 150))
    x2 = x_end - arrow_size * sin(radians(angle - 150))
    y2 = y_end + arrow_size * cos(radians(angle - 150))
    
    # Draw the arrowhead on the canvas
    canvas.create_line(x_end, y_end, x1, y1, width=pointer_width, fill=color, tags="pointers")
    canvas.create_line(x_end, y_end, x2, y2, width=pointer_width, fill=color, tags="pointers")

# Create a Tkinter window
root = Tk()
root.title("Compass Rose")

# Create a Canvas widget to draw the compass
canvas = Canvas(root, width=300, height=300, bg="white")
canvas.pack()

# Draw the compass rose
canvas.create_oval(50, 50, 250, 250)  # Outer circle
canvas.create_oval(100, 100, 200, 200)  # Inner circle
canvas.create_line(150, 50, 150, 250)  # North-South line
canvas.create_line(50, 150, 250, 150)  # East-West line

# Example angles for the pointers (replace with your actual values)
buoy_heading = 45
target_heading = 135
gps_heading = 225

# Update the pointers initially
update_pointers(buoy_heading, target_heading, gps_heading)

# Example usage: Change the pointer angles and update
buoy_heading = 90
target_heading = 180
gps_heading = 270


# Run the Tkinter event loop
root.mainloop()
