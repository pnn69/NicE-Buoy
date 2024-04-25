import tkinter as tk
from tkinter import messagebox

def update_counter(counter_var, text_var):
    counter_var.set(counter_var.get() + 1)

    # Change text every second
    if text_var.get() == "Hello":
        text_var.set("Goodbye")
    else:
        text_var.set("Hello")

    root.after(1000, update_counter, counter_var, text_var)

def send(input_entries, radio_selection):
    # Get input values and selected radio button
    inputs = [entry.get() for entry in input_entries]
    selected_radio = radio_selection.get()

    # Display input values and selected radio button in a message box
    message = f"Inputs: {inputs}\nSelected Radio: {selected_radio}"
    messagebox.showinfo("Input Values", message)

# Create the main Tkinter window
root = tk.Tk()
root.title("Input Form")

# Variables to store input values
inputs = [tk.StringVar() for _ in range(5)]

# Counter variable
counter = tk.IntVar()
counter.set(0)

# Text variable for changing text
text_var = tk.StringVar()
text_var.set("Hello")

# Create input fields
input_entries = []
for i in range(5):
    label = tk.Label(root, text=f"Input {i+1}:")
    label.grid(row=i, column=0, sticky=tk.W)
    entry = tk.Entry(root, textvariable=inputs[i])
    entry.grid(row=i, column=1)
    input_entries.append(entry)

# Label to display the value of Input 1
input1_value_label = tk.Label(root, textvariable=inputs[0])
input1_value_label.grid(row=0, column=2,columnspan=200,sticky=tk.W)
#input1_value_label.grid(row=0, column=2, sticky=tk.W)
# Label to display the value of Input 1
input2_value_label = tk.Label(root,  textvariable=inputs[1])
input2_value_label.grid(row=1, column=2, sticky=tk.W)

# Create radio buttons
radio_selection = tk.StringVar()
radio_selection.set("Option 1")
radio_buttons = []
for i in range(5):
    radio_button = tk.Radiobutton(root, text=f"Option {i+1}", variable=radio_selection, value=f"Option {i+1}")
    radio_button.grid(row=i, column=3)
    radio_buttons.append(radio_button)

# Create counter label
counter_label = tk.Label(root, textvariable=counter)
counter_label.grid(row=5, columnspan=4)

# Create text changing label
text_label = tk.Label(root, textvariable=text_var)
text_label.grid(row=6, columnspan=4)

# Create send button
send_button = tk.Button(root, text="Send", command=lambda: send(input_entries, radio_selection))
send_button.grid(row=7, columnspan=4)

# Start counter
update_counter(counter, text_var)

root.mainloop()
