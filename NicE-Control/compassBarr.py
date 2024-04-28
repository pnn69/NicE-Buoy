from tkinter import *

def draw_progress_bar(canvas, value):
    canvas.delete("progress_bar")  # Clear the canvas before redrawing
    
    # Define the canvas size
    canvas_width = canvas.winfo_width()
    canvas_height = canvas.winfo_height()

    # Define the dimensions of the progress bar
    bar_width = 20
    bar_height = 200
    x_start = (canvas_width - bar_width) / 2
    y_start = (canvas_height - bar_height) / 2

    # Calculate the width of the filled portion based on the value
    filled_width = (bar_width / 200) * (value)

    # Draw the outline of the progress bar
    canvas.create_rectangle(x_start, canvas_width/2, x_start + bar_width, y_start + bar_height, outline="black")
    canvas.create_rectangle(x_start, y_start, x_start + bar_width, y_start + bar_height/2, outline="black")

    # Draw the outline of the progress bar
    #canvas.create_rectangle(x_start, y_start, x_start + bar_width, y_start + bar_height, outline="black")
    #canvas.create_rectangle(x_start, canvas_height/2, x_start + bar_width, y_start + bar_height/2, outline="black")




    # Draw the filled portion of the progress bar
    if value < 0:
        canvas.create_rectangle(x_start,canvas_height/2-value, x_start  + bar_width, canvas_height/2, fill="red", tags="progress_bar")
    else:
        canvas.create_rectangle(x_start,canvas_height/2-value, x_start  + bar_width, canvas_height/2, fill="green", tags="progress_bar")

# Example usage:
def update_progress():
    for value in range(-100, 101, 10):
        canvas.after(500, draw_progress_bar, canvas, 50)
        canvas.update()

root = Tk()
root.title("Progress Bar")

canvas = Canvas(root, width=100, height=300, bg="white")
canvas.pack()

update_progress()

root.mainloop()
