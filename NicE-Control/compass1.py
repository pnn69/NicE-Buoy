from tkinter import *
from math import sin, cos, radians

def draw_compass(canvas):
    canvas.create_oval(50, 50, 250, 250)  # Outer circle
    canvas.create_oval(100, 100, 200, 200)  # Inner circle
    canvas.create_line(150, 50, 150, 250)  # North-South line
    canvas.create_line(50, 150, 250, 150)  # East-West line

def draw_pointer(canvas, angle, color):
    x_center = 150  # X-coordinate of the center of the compass
    y_center = 150  # Y-coordinate of the center of the compass
    length = 100    # Length of the pointer
    pointer_width = 2  # Width of the pointer
    
    # Calculate the endpoint of the pointer based on the angle
    x_end = x_center + length * sin(radians(angle))
    y_end = y_center - length * cos(radians(angle))
    
    # Draw the pointer line on the canvas
    canvas.create_line(x_center, y_center, x_end, y_end, width=pointer_width, fill=color)

def create_compass(root):
    # Create a Canvas widget to draw the compass
    canvas = Canvas(root, width=300, height=300, bg="white")
    canvas.pack()

    # Draw the compass rose
    draw_compass(canvas)

    # Draw the pointers
    draw_pointer(canvas, 45, "red")    # Buoy heading
    draw_pointer(canvas, 135, "blue")  # Target heading
    draw_pointer(canvas, 225, "green") # GPS heading
