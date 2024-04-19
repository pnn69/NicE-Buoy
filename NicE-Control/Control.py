from tkinter import *
import matplotlib.pyplot as plt
import numpy as np

#with serial.Serial('COM3', 19200, timeout=1) as ser:
#     x = ser.read()          # read one byte
#     s = ser.read(10)        # read up to ten bytes (timeout)
#     line = ser.readline()   # read a '\n' terminated line
count: int = 1

def click():
     global count
     count+=1
     print(count)
     print("You clicked the button")
 
 
msg = "hello world"
print(msg)
     
window = Tk()
     
button = Button(window,
                     text="click me",
                     command=click,
                     font=("Comic Sans",30),
                     fg="#00ff00",
                     bg="black",
                     activeforeground="green",
                     activebackground="black",
                     state=ACTIVE)
button.pack()
     
window.mainloop()