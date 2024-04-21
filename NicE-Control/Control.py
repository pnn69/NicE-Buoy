import tkinter as tk
import serial

# Open COM3 port
ser = serial.Serial('COM3', 115200)  # Adjust baud rate as per your requirement


def radio_button_selected():
    selected_option = radio_var.get()
    print("Selected Radio Button:", selected_option)
    
#<buoynr><sender><status><msgID><>msg
def submit():
    output_field.delete(1.0, tk.END)
    output_field.insert(tk.END, "<1>")
    output_field.insert(tk.END,"<255>")
    selected_option = radio_var.get()
    output_field.insert(tk.END,"<" + selected_option + ">")
    if selected_option == "30":
        output_field.insert(tk.END,"<" + entry1.get()+ ">")
    if selected_option == "29":
        output_field.insert(tk.END,"<" + entry2.get()+ ">")
    if selected_option == "31":
        output_field.insert(tk.END,"<" + entry3.get()+ ">")
    ser.write(output_field)

        

# Create main window
root = tk.Tk()
root.title("GUI with Radio Buttons, Input Fields, and Output Field")

var = tk.StringVar()
# Create input fields


# Create radio buttons
radio_var = tk.StringVar()
radio_var.set(None)  # Set initial value to None
radio_button1 = tk.Radiobutton(root, text="PID_RUDDER", variable=radio_var, value="30", command=radio_button_selected)
radio_button1.grid(row=0, column=0)

radio_button2 = tk.Radiobutton(root, text="PID_SPEED", variable=radio_var, value="29", command=radio_button_selected)
radio_button2.grid(row=0, column=1)

radio_button3 = tk.Radiobutton(root, text="POS_DIR", variable=radio_var, value="31", command=radio_button_selected)
radio_button3.grid(row=0, column=2)

entry1 = tk.Entry(root)
entry1.insert(tk.END, "20,0.1,01")  # Pre-fill entry2
entry1.grid(row=1, column=0)

entry2 = tk.Entry(root)
entry2.insert(tk.END, "05,0.02,01")  # Pre-fill entry2
entry2.grid(row=1, column=1)

entry3 = tk.Entry(root)
entry3.grid(row=1, column=2)
entry3.insert(tk.END, "0,100")  # Pre-fill entry2


# Create output field
output_field = tk.Text(root, height=5, width=30)
output_field.grid(row=2, columnspan=3)

# Create submit button
submit_button = tk.Button(root, text="Submit", command=submit)
submit_button.grid(row=3, columnspan=3)

selected_label = tk.Label(root, text="lala")
selected_label.grid()

# Run the main event loop
root.mainloop()

