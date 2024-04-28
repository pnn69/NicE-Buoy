import tkinter as tk
import serial
import threading
import re

# Open COM3 port
ser = serial.Serial('COM7', 115200)  # Adjust baud rate as per your requirement

def radio_button_selected():
    selected_option = radio_var.get()
    print("Selected Radio Button:", selected_option)

def submit():
    output_field.delete(1.0, tk.END)
    output_field.insert(tk.END, "1")
    selected_option = radio_var.get()
    output_field.insert(tk.END,"^" + selected_option)
    output_field.insert(tk.END,"^SET")
    if selected_option == "PID_RUDDER_PARAMETERS":
        output_field.insert(tk.END,"^" + entry1.get())
    if selected_option == "PID_SPEED_PARAMETERS":
        output_field.insert(tk.END,"^" + entry2.get())
    if selected_option == "CHANGE_POS_DIR_DIST":
        output_field.insert(tk.END,"^" + entry3.get())
    output_field.insert(tk.END,"")
    output_data = output_field.get("1.0", tk.END).strip()
    ser.write(output_data.encode())
    print(output_data)

def read_serial():
    while True:
        
        if ser.in_waiting > 0:
            received_data = ser.readline().decode().strip()
            received_text.config(state=tk.NORMAL)
            received_text.insert(tk.END, received_data + '\n')
            received_text.config(state=tk.DISABLED)
            received_text.see(tk.END)
            #input_string = "<1><23><2><2234,34.3>"
            # Regular expression pattern to match the fragments enclosed in <>
            pattern = r'<(.*?)>'
            # Find all matches of the pattern in the input string
            matches = re.findall(pattern, received_data)
            # Initialize an empty buffer array
            buffer_array = []
            # Add matches to the buffer array
            buffer_array.extend(matches)
            received_data = ""
            #<sender><status><msgID><ACK><MSG>
            # 0       1       2      3    4
            if len(buffer_array) > 2:
                if buffer_array[2] == "29":
                    buf29.delete(0,tk.END)
                    buf29.insert(tk.END,buffer_array[4])  # Pre-fill entry3
                if buffer_array[2] == "18":
                    buf23.delete(0,tk.END)
                    buf23.insert(tk.END,buffer_array[4])  # Pre-fill entry3


# Create main window
root = tk.Tk()
root.title("GUI with Radio Buttons, Input Fields, and Output Field")

# Create input fields
entry1 = tk.Entry(root).insert(tk.END, "1,0.05,00")  # Pre-fill entry1
entry1
entry1.grid(row=1, column=0)

entry2 = tk.Entry(root)
entry2.insert(tk.END, "20,0.4,0")  # Pre-fill entry2
entry2.grid(row=1, column=1)

entry3 = tk.Entry(root)
entry3.grid(row=1, column=2)
entry3.insert(tk.END, "0,100")  # Pre-fill entry3

buf29 = tk.Entry(root)
buf29.grid(row=1, column=4)
buf29.insert(tk.END, "")  # Pre-fill entry3

buf23 = tk.Entry(root,width=50)
buf23.grid(row=2, column=4)
buf23.insert(tk.END, "")  # Pre-fill entry3


# Create radio buttons
radio_var = tk.StringVar()
radio_var.set(None)  # Set initial value to None

radio_button1 = tk.Radiobutton(root, text="PID_RUDDER", variable=radio_var, value="PID_RUDDER_PARAMETERS", command=radio_button_selected)
radio_button1.grid(row=0, column=0)

radio_button2 = tk.Radiobutton(root, text="PID_SPEED", variable=radio_var, value="PID_SPEED_PARAMETERS", command=radio_button_selected)
radio_button2.grid(row=0, column=1)

radio_button3 = tk.Radiobutton(root, text="POS_DIR", variable=radio_var, value="CHANGE_POS_DIR_DIST", command=radio_button_selected)
radio_button3.grid(row=0, column=2)

# Create output field
output_field = tk.Text(root, height=1, width=50)
output_field.grid(row=2, columnspan=3)

# Create received data text widget
received_text = tk.Text(root, height=30, width=100)  # Wider size
received_text.grid(row=50, columnspan=3)
received_text.config(state=tk.DISABLED)

# Create submit button
submit_button = tk.Button(root, text="Submit", command=submit)
submit_button.grid(row=2, columnspan=1)

# Create and start thread for reading serial data
serial_thread = threading.Thread(target=read_serial)
serial_thread.daemon = True
serial_thread.start()

# Run the main event loop
root.mainloop()
