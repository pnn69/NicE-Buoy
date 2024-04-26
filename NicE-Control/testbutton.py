from tkinter import *
import compass
import random
# Create a Tkinter window
root = Tk()
root.title("Main Script")

# Call the function from the compass module to create the compass rose
compass.create_compass(root)

# Function to update headings dynamically
def update_headings():
    # Example: Update headings (replace with your logic)
    t =random.randint(0, 359)
    print(t)
    compass.update_buoy_heading(t)
    t =random.randint(0, 359)
    print(t)
    compass.update_target_heading(t)
    t =random.randint(0, 359)
    print(t)
    compass.update_gps_heading(t)

# Button to update headings
update_button = Button(root, text="Update Headings", command=update_headings)
update_button.pack()

# Run the Tkinter event loop
root.mainloop()
