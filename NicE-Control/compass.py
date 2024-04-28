# compass.py

from tkinter import *
from math import sin, cos, radians

# Define global variables to store the heading angles
buoy_heading = 45
target_heading = 135
gps_heading = 225
speed_bb = IntVar
speed_sb = IntVar  

def update_bars(bb_var, sb_var):
    #speed_sb.set(sb_var)
    #speed_bb.set(bb_var)
    print(bb_var)
    print(sb_var)

def draw_compass(canvas):
    canvas.delete("all")  # Clear the canvas before redrawing
    canvas.create_oval(50, 50, 250, 250)  # Outer circle
    canvas.create_oval(100, 100, 200, 200)  # Inner circle
    canvas.create_line(150, 50, 150, 250)  # North-South line
    canvas.create_line(50, 150, 250, 150)  # East-West line
    # Add left scale bar
    #left_bar = Scale(canvas, from_=-100, to=100, orient=VERTICAL)
    #left_bar.place(x=0, y=50)
    
    #scale_bb = Scale(canvas, from_=-100, to=100, variable=speed_bb, orient=VERTICAL)
    #scale_bb.place(x=0, y=100)
    
    #scale_sb = Scale(canvas, from_=-100, to=100, variable=speed_sb, orient=VERTICAL)
    #scale_sb.place(x=251, y=100)
    
    # Add right scale bar
    #right_bar = Scale(canvas, from_=-100,  to=100, value=-10, orient=VERTICAL)
    #right_bar.place(x=251, y=50)



# Define functions to update the heading angles
def draw_pointer(angle, canvas, color):
    x_center = 150  # X-coordinate of the center of the compass
    y_center = 150  # Y-coordinate of the center of the compass
    length = 100    # Length of the pointer
    pointer_width = 2  # Width of the pointer
    # Clear the previous pointer by deleting all items with the given color
    if color == "green": 
        print(angle)
    if color == "black":
        canvas.delete("red")    
    else:
        canvas.delete(color)
    # Calculate the endpoint of the pointer based on the angle
        x_end = x_center + length * sin(radians(angle))
        y_end = y_center - length * cos(radians(angle))
        # Draw the pointer line on the canvas and save its ID to delete it later
        pointer_id = canvas.create_line(x_center, y_center, x_end, y_end, width=pointer_width, fill=color, tags=color)
   
    #return pointer_id  # Return the ID of the pointer item for further use

def create_compass(root):
    # Create a Canvas widget to draw the compass
    canvas = Canvas(root, width=300, height=300, bg="white")
    canvas.pack()
    # Draw the compass rose
    draw_compass(canvas)
    return canvas  # Return the canvas object for further use
