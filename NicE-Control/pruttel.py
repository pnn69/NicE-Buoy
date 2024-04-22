import tkinter as tk
import re


# Create main window
root = tk.Tk()
root.title("GUI with Radio Buttons, Input Fields, and Output Field")

entry1 = tk.Entry(root)
entry1.insert(tk.END, "20,0.1,01")  # Pre-fill entry2
entry1.grid(row=1, column=0)

entry2 = tk.Entry(root)
entry2.insert(tk.END, "05,0.02,01")  # Pre-fill entry2
entry2.grid(row=1, column=2)


bbTxt = tk.Label(root,text="Bb")
bbTxt.config(justify=tk.RIGHT)
bbTxt.grid(row=3, column=0)

bbPwr = tk.Label(root,text="0")
bbPwr.grid(row=3, column=1)
buf29 = tk.Entry(root)
buf29.grid(row=5, column=0)
buf29.insert(tk.END, "LEEG")  # Pre-fill entry3

bufAck = tk.Entry(root)
bufAck.grid(row=6, column=0)
bufAck.insert(tk.END, "ACK")  # Pre-fill entry3


sbTxt = tk.Label(root,text="Bs")
sbTxt.grid(row=3, column=2)
sbPwr = tk.Label(root,text="0")
sbPwr.grid(row=3, column=3)

# Your input string
input_string = "<1><23><29><1><2234,34.3>"
#<sender><status><msgID><ACK><MSG>
# Regular expression pattern to match the fragments enclosed in <>
#input_string = "<1><23><2><2234,34.3>"
# Regular expression pattern to match the fragments enclosed in <>
pattern = r'<(.*?)>'
# Find all matches of the pattern in the input string
matches = re.findall(pattern, input_string)
# Initialize an empty buffer array
buffer_array = []
# Add matches to the buffer array
buffer_array.extend(matches)
received_data = ""
#<sender><status><msgID><ACK><MSG>
# 0       1       2      3    4
if buffer_array[2] == "29":
    buf29.delete(0,tk.END)
    buf29.insert(tk.END,buffer_array[4])  # Pre-fill entry3
if buffer_array[3] == "1":
    bufAck.delete(0,tk.END)
    bufAck.insert(tk.END,buffer_array[4])  # Pre-fill entry3

# Print the buffer array
print(buffer_array)
selected_label = tk.Label(root, text=buffer_array[3])
selected_label.grid()

# Data for the bars
data = [80, -30]  # Values for the two columns
bbPWR = tk.Label(root,text=data[0])
bbPWR.grid(row=3, column=1)
sbPWR = tk.Label(root,text=data[1])
sbPWR.grid(row=3, column=3)

# Run the main event loop
root.mainloop()

