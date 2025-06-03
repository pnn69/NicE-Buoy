import tkinter as tk
from math import sin, cos, radians

class CompassDisplay:
    def __init__(self, root):
        self.canvas = tk.Canvas(root, width=300, height=320, bg="white")
        self.canvas.pack()
        self._init_labels()
        self._draw_static_elements()
        self.battperc = 0
        self.circle_visible = True

    def _init_labels(self):
        self.bb_text = tk.Label(self.canvas, text="?%", bg="white")
        self.bb_text.place(x=10, y=29)
        tk.Label(self.canvas, text="BBpwr", bg="white").place(x=8, y=255)

        self.sb_text = tk.Label(self.canvas, text="?%", bg="white")
        self.sb_text.place(x=270, y=29)
        tk.Label(self.canvas, text="SBpwr", bg="white").place(x=255, y=255)

        self.batt_text = tk.Label(self.canvas, text="?", bg="white")
        self.batt_text.place(x=255, y=280)

        self.mhdg_label = tk.Label(self.canvas, text="Magnetic heading: ", bg="white", fg="green")
        self.mhdg_label.place(x=10, y=10)

        self.tghdg_label = tk.Label(self.canvas, text="Target heading: ", bg="white", fg="blue")
        self.tghdg_label.place(x=190, y=10)

        self.gpshdg_label = tk.Label(self.canvas, text="GPS heading: ", bg="white", fg="red")
        self.gpshdg_label.place(x=110, y=255)

    def _draw_static_elements(self):
        self.canvas.create_oval(50, 50, 250, 250)  # Outer circle
        self.canvas.create_oval(100, 100, 200, 200)  # Inner circle
        self.canvas.create_line(150, 50, 150, 250)  # NS line
        self.canvas.create_line(50, 150, 250, 150)  # EW line
        self.canvas.create_rectangle(10, 50, 30, 250, outline="black")  # BB Power bar
        self.canvas.create_rectangle(270, 50, 290, 250, outline="black")  # SB Power bar
        self.canvas.create_rectangle(50, 280, 250, 300, outline="black")  # Battery bar

        self.circle = self.canvas.create_oval(15, 285, 25, 295, fill="", outline="black")

    def draw_pointer(self, angle, color):
        center_x, center_y = 150, 150
        length = 100
        self.canvas.delete(color)
        end_x = center_x + length * sin(radians(angle))
        end_y = center_y - length * cos(radians(angle))
        self.canvas.create_line(center_x, center_y, end_x, end_y, width=2, fill=color, tags=color)

        if color == "green":
            self.mhdg_label.config(text=f"Magnetic heading: {angle}")
        elif color == "blue":
            self.tghdg_label.config(text=f"Target heading: {angle}")
        elif color == "red":
            self.gpshdg_label.config(text=f"GPS heading: {angle}")

    def draw_power_bars(self, bb, sb):
        self.canvas.delete("progress_bar")
        canvas_mid = 150

        def draw_bar(x, value):
            height = abs(value)
            color = "green" if value >= 0 else "red"
            top = canvas_mid - height if value >= 0 else canvas_mid
            bottom = canvas_mid if value >= 0 else canvas_mid + height
            self.canvas.create_rectangle(x, top, x + 20, bottom, fill=color, tags="progress_bar")

        if -100 <= bb <= 100: draw_bar(10, bb)
        if -100 <= sb <= 100: draw_bar(270, sb)

        self.bb_text.config(text=f"{bb}%")
        self.sb_text.config(text=f"{sb}%")

    def draw_battery(self, percent):
        self.canvas.delete("progress_bar_battery")
        self.battperc = max(0, min(percent, 100))
        self.canvas.create_rectangle(50, 280, 50 + 200 * self.battperc / 100, 300,
                                     fill="green", tags="progress_bar_battery")
        self.batt_text.config(text=f"{self.battperc}%")

    def toggle_circle(self):
        if self.circle_visible:
            self.canvas.itemconfig(self.circle, fill="green")
        else:
            self.canvas.itemconfig(self.circle, fill="")
        self.circle_visible = not self.circle_visible


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Compass Display")

    compass = CompassDisplay(root)

    # Example values
    buoy_heading = 45
    target_heading = 120
    gps_heading = 270

    # Draw example indicators
    compass.draw_pointer(buoy_heading, "green")
    compass.draw_pointer(target_heading, "blue")
    compass.draw_pointer(gps_heading, "red")
    compass.draw_power_bars(60, -40)
    compass.draw_battery(80)

    root.mainloop()
