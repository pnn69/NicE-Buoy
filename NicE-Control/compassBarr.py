from tkinter import *
from math import sin, cos, radians

def update_bars(sb, bb, value=None):
    if value is not None:
        sb.set(value)
        bb.set(value)

def main():
    root = Tk()
    root.geometry("200x300")

    sb = IntVar()  # Initialize sb as IntVar for integer values
    bb = IntVar()  # Initialize bb as IntVar for integer values

    scale_sb = Scale(root, from_=-100, to=100, variable=sb, orient=VERTICAL)
    scale_sb.pack(side=LEFT)

    scale_bb = Scale(root, from_=-100, to=100, variable=bb, orient=VERTICAL)
    scale_bb.pack(side=RIGHT)

    update_button = Button(root, text="Update Bars", command=lambda: update_bars(sb, bb, 50))
    update_button.pack()

    root.mainloop()

if __name__ == "__main__":
    main()
