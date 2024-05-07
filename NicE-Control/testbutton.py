from tkinter import *
import compass
import random
import webbrowser

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

def update_googlemaps():
        #url = f"https://www.google.com/maps/?q=52.292533,4.931970"
        url = f"https://www.google.nl/maps/52.29308075283747,4.932570409845357,1000m"
        # Open the URL in a web browser
        webbrowser.open(url)     
# Button to update headings
update_button = Button(root, text="Update Google Maps", command=update_googlemaps)
update_button.pack()
url = f"https://www.google.com/maps/?q=52.292533,4.931970"
#url = f"https://www.google.nl/maps/52.29308075283747,4.932570409845357,1000m"
webbrowser.open(url)     
# Run the Tkinter event loop
#root.mainloop()
