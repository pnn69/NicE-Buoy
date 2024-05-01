# compass.py

from tkinter import *

from math import sin, cos, radians

# Define global variables to store the heading angles
buoy_heading = 45
target_heading = 135
gps_heading = 225
speed_bb = 0
speed_sb = 0 
bb_text_cp = None
sb_text_cp = None
battperc = 0
once = False
def draw_compass(canvas):
    canvas.delete("all")  # Clear the canvas before redrawing
    canvas.create_oval(50, 50, 250, 250)  # Outer circle
    canvas.create_oval(100, 100, 200, 200)  # Inner circle
    canvas.create_line(150, 50, 150, 250)  # North-South line
    canvas.create_line(50, 150, 250, 150)  # East-West line
    # Define the dimensions of the progress bar
    bar_width = 20
    bar_height = 200
    x_bb = 10
    x_sb =270
    y_start = 50 
    canvas.create_rectangle(x_bb, 50, x_bb + bar_width, 50 + 200, outline="black")
    canvas.create_rectangle(x_sb, y_start, x_sb + bar_width, y_start + bar_height, outline="black")
    canvas.create_rectangle(50, 300, 50 + 200, 280,  outline="black")



# Define functions to update the heading angles
def draw_pointer(angle, canvas, color):
    global Tghdg_text, Mhdg_text
    x_center = 150  # X-coordinate of the center of the compass
    y_center = 150  # Y-coordinate of the center of the compass
    length = 100    # Length of the pointer
    pointer_width = 2  # Width of the pointer
    # Clear the previous pointer by deleting all items with the given color
    if color == "black":
        canvas.delete("red")    
    else:
        canvas.delete(color)
    # Calculate the endpoint of the pointer based on the angle
        x_end = x_center + length * sin(radians(angle))
        y_end = y_center - length * cos(radians(angle))
        # Draw the pointer line on the canvas and save its ID to delete it later
        pointer_id = canvas.create_line(x_center, y_center, x_end, y_end, width=pointer_width, fill=color, tags=color)
    if  color == "green":
            Mhdg_text.config(text=f"Magentic heading: {angle}")
    if  color == "blue":
            Tghdg_text.config(text=f"Target heading: {angle}")
   
    #return pointer_id  # Return the ID of the pointer item for further use
def draw_barr(bb,sb,canvas):
    global bb_text_cp , sb_text_cp , once , battperc
    canvas.delete("progress_bar")  # Clear the canvas before redrawing
    # Define the canvas size
    canvas_width = canvas.winfo_width()
    canvas_height = canvas.winfo_height()

    # Define the dimensions of the progress bar
    bar_width = 20
    bar_height = 200
    x_bb = 10
    x_sb =270
    x_progress_bar = 100
    y_progress_bar = 270
    y_start = (canvas_height - bar_height) / 2

    if bb < 0 & bb >= -100:
        canvas.create_rectangle(x_bb,canvas_height/2-bb, x_bb  + bar_width, canvas_height/2, fill="red", tags="progress_bar")
    if bb >= 0 & bb <= 100:
        canvas.create_rectangle(x_bb,canvas_height/2-bb, x_bb  + bar_width, canvas_height/2, fill="green", tags="progress_bar")
    if sb < 0 & sb >= -100:
        canvas.create_rectangle(x_sb,canvas_height/2-sb, x_sb  + bar_width, canvas_height/2, fill="red", tags="progress_bar")
    if sb >= 0 & sb <= 100:
        canvas.create_rectangle(x_sb,canvas_height/2-sb, x_sb  + bar_width, canvas_height/2, fill="green", tags="progress_bar")
    canvas.create_rectangle(50, 300, 50 + 200*battperc/100, 280,  fill="green", tags="progress_bar")
    if bb_text_cp:
        bb_text_cp.config(text=f"{bb}%")
    if sb_text_cp:
        sb_text_cp.config(text=f"{sb}%")
        
def draw_Vbatbarr(perc,canvas):
    global battperc
    battperc = perc
    canvas.create_rectangle(50, 300, 50 + 200*battperc/100, 280,  fill="green", tags="progress_bar")
    batt_text.config(text=f"Battery: {perc}%")

def create_compass(root):
    global bb_text_cp , sb_text_cp , batt_text , Mhdg_text , Tghdg_text  # Access the global label
    # Create a Canvas widget to draw the compass
    canvas = Canvas(root, width=300, height=300, bg="white")
    canvas.pack()
    # Draw the compass rose
    draw_compass(canvas)
    bb_text_cp = Label(canvas, text="?%", bg="white")
    bb_text_cp.place(x=10, y=29)
    bb_text = Label(canvas, text="BBpwr", bg="white")
    bb_text.place(x=8, y=255)
    sb_text_cp = Label(canvas, text="?%", bg="white")
    sb_text_cp.place(x=270, y=29)
    sb_text = Label(canvas, text="SBpwr", bg="white")
    sb_text.place(x=255, y=255)
    batt_text = Label(canvas, text="Battery", bg="white")
    batt_text.place(x=110, y=255)
    Mhdg_text = Label(canvas, text="Magentic heading: ", bg="white", fg = "green")
    Mhdg_text.place(x=10, y=10)
    Tghdg_text = Label(canvas, text="Target heading: ", bg="white", fg = "blue")
    Tghdg_text.place(x=190, y=10)

    return canvas  # Return the canvas object for further use
