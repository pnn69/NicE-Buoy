import tkinter as tk
from tkinter import messagebox
import time

class App:
    def __init__(self, master):
        self.master = master
        master.title("Input Form")

        # Variables to store input values
        self.inputs = [tk.StringVar() for _ in range(5)]

        # Counter variable
        self.counter = tk.IntVar()
        self.counter.set(0)

        # Text variable for changing text
        self.text_var = tk.StringVar()
        self.text_var.set("Hello")

        # Create input fields
        self.input_entries = []
        for i in range(5):
            label = tk.Label(master, text=f"Input {i+1}:")
            label.grid(row=i, column=0, sticky=tk.W)
            entry = tk.Entry(master, textvariable=self.inputs[i])
            entry.grid(row=i, column=1)
            self.input_entries.append(entry)

        # Create radio buttons
        self.radio_selection = tk.StringVar()
        self.radio_selection.set("Option 1")
        self.radio_buttons = []
        for i in range(5):
            radio_button = tk.Radiobutton(master, text=f"Option {i+1}", variable=self.radio_selection, value=f"Option {i+1}")
            radio_button.grid(row=i, column=2)
            self.radio_buttons.append(radio_button)

        # Create counter label
        self.counter_label = tk.Label(master, textvariable=self.counter)
        self.counter_label.grid(row=5, columnspan=3)

        # Create text changing label
        self.text_label = tk.Label(master, textvariable=self.text_var)
        self.text_label.grid(row=6, columnspan=3)

        # Create send button
        self.send_button = tk.Button(master, text="Send", command=self.send)
        self.send_button.grid(row=7, columnspan=3)

        # Start counter
        self.update_counter()

    def update_counter(self):
        self.counter.set(self.counter.get() + 1)
        self.master.after(1000, self.update_counter)

        # Change text every second
        if self.text_var.get() == "Hello":
            self.text_var.set("Goodbye")
        else:
            self.text_var.set("Hello")

    def send(self):
        # Get input values and selected radio button
        inputs = [entry.get() for entry in self.input_entries]
        selected_radio = self.radio_selection.get()

        # Display input values and selected radio button in a message box
        message = f"Inputs: {inputs}\nSelected Radio: {selected_radio}"
        messagebox.showinfo("Input Values", message)

# Create the main Tkinter window
root = tk.Tk()
app = App(root)
root.mainloop()
