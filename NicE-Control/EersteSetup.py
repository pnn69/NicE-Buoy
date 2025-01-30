import tkinter as tk
from tkinter import ttk
import math

class Entry(ttk.Frame):
    def __init__(self, parent, label_text, button_text, label_background, wind_direction=None):
        super().__init__(parent)
        
        # Add label
        label = ttk.Label(self, text=label_text, background=label_background, anchor='center')
        label.pack(expand=True, fill='both')
        
        # Add wind rose canvas
        self.canvas = tk.Canvas(self, width=200, height=200, bg="white")
        self.canvas.pack(expand=True, fill='both', pady=10)
        
        # Draw wind rose on the canvas
        self.draw_wind_rose(
            canvas=self.canvas, 
            center_x=100, 
            center_y=100, 
            outer_radius=80, 
            inner_radius=50, 
            directions=[
                ("N", 90), ("NE", 45), ("E", 0), ("SE", -45),
                ("S", -90), ("SW", -135), ("W", 180), ("NW", 135)
            ], 
            wind_direction=wind_direction
        )
        
        # Add button
        button = ttk.Button(self, text=button_text)
        button.pack(expand=True, fill='both', pady=10)
        
        self.pack(side='left', expand=True, fill='both', padx=20, pady=20)
    
    def draw_wind_rose(self, canvas, center_x, center_y, outer_radius, inner_radius, directions, wind_direction=None):
        """
        Draws a wind rose with two concentric circles, labeled directions, and a wind direction line.
        :param canvas: The Tkinter canvas to draw on.
        :param center_x: X-coordinate of the wind rose center.
        :param center_y: Y-coordinate of the wind rose center.
        :param outer_radius: Radius of the outer circle.
        :param inner_radius: Radius of the inner circle.
        :param directions: List of tuples with direction labels and angles (in degrees).
        :param wind_direction: The wind direction in degrees (0° = East, 90° = North, etc.).
        """
        # Draw outer circle
        canvas.create_oval(
            center_x - outer_radius, center_y - outer_radius,
            center_x + outer_radius, center_y + outer_radius,
            outline="black", width=2
        )
        
        # Draw inner circle
        canvas.create_oval(
            center_x - inner_radius, center_y - inner_radius,
            center_x + inner_radius, center_y + inner_radius,
            outline="black", width=2
        )
        
        # Add the direction labels
        for direction, angle in directions:
            # Convert angle to radians (correcting for Tkinter's coordinate system)
            radian_angle = math.radians(-angle)
            
            # Calculate position for the labels
            label_x = center_x + (outer_radius + 15) * math.cos(radian_angle)
            label_y = center_y + (outer_radius + 15) * math.sin(radian_angle)
            canvas.create_text(label_x, label_y, text=direction, fill="blue", font=("Arial", 8, "bold"))
        
        # Draw the wind direction line if provided
        if wind_direction is not None:
            # Convert wind direction to radians (correcting for Tkinter's coordinate system)
            radian_angle = math.radians(-wind_direction)
            end_x = center_x + (outer_radius * 0.9) * math.cos(radian_angle)
            end_y = center_y + (outer_radius * 0.9) * math.sin(radian_angle)
            canvas.create_line(center_x, center_y, end_x, end_y, fill="red", width=2, arrow=tk.LAST)

# Main Application
root = tk.Tk()
root.title("Wind Rose with Entry Class")

# Add an instance of the Entry class with a wind rose
entry = Entry(
    root, 
    label_text="Wind Rose Example", 
    button_text="OK", 
    label_background="lightgray", 
    wind_direction=135  # Example wind direction
)

root.mainloop()
