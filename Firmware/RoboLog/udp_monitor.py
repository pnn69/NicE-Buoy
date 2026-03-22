import tkinter as tk
from tkinter import ttk
import socket
import threading
import re
import math
from datetime import datetime

class RoboMonitor:
    def __init__(self, master):
        self.master = master
        master.title("Robobuoy Monitor - 3 Buoys")
        master.geometry("1000x900")
        master.resizable(False, False)

        self.udp_ip = "0.0.0.0"  # Listen on all available interfaces
        self.udp_port = 1001     # Port RobobuoyTop sends UDP messages to

        self.buoy_frames = []
        self.create_widgets()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))

        self.running = True
        self.udp_thread = threading.Thread(target=self.udp_listener)
        self.udp_thread.daemon = True
        self.udp_thread.start()

        self.update_gui()

    def create_widgets(self):
        self.columns_frame = ttk.Frame(self.master)
        self.columns_frame.pack(expand=True, fill="both", padx=10, pady=5)

        for i in range(3):
            frame = ttk.LabelFrame(self.columns_frame, text=f"Buoy {i+1} (Waiting...)")
            frame.grid(row=0, column=i, sticky="nsew", padx=5, pady=5)
            self.columns_frame.columnconfigure(i, weight=1)
            
            # Windrose and Bars Area
            visual_frame = tk.Frame(frame)
            visual_frame.pack(padx=5, pady=5)
            
            # Left Bar: BB Power
            bb_frame = tk.Frame(visual_frame)
            bb_frame.pack(side="left", fill="y", padx=(0, 5))
            tk.Label(bb_frame, text="BB", font=("Arial", 8)).pack()
            bb_bar = tk.Canvas(bb_frame, width=20, height=180, bg="lightgray", highlightthickness=1, highlightbackground="gray")
            bb_bar.pack(expand=True, fill="y")
            bb_bar.create_line(0, 90, 20, 90, fill="black", width=2) # Center line
            bb_val_label = tk.Label(bb_frame, text="0%", font=("Arial", 8, "bold"), width=5)
            bb_val_label.pack()
            
            # Center: Windrose + Voltage
            center_frame = tk.Frame(visual_frame)
            center_frame.pack(side="left")
            
            windrose_canvas = tk.Canvas(center_frame, width=200, height=200, bg="white")
            windrose_canvas.pack()
            self.draw_windrose_background(windrose_canvas)
            
            volt_frame = tk.Frame(center_frame)
            volt_frame.pack(fill="x", pady=(5, 0))
            tk.Label(volt_frame, text="17V", font=("Arial", 7)).pack(side="left")
            volt_bar = ttk.Progressbar(volt_frame, orient="horizontal", length=150, mode="determinate", maximum=8.2)
            volt_bar.pack(side="left", expand=True, fill="x", padx=2)
            tk.Label(volt_frame, text="25V", font=("Arial", 7)).pack(side="left")
            
            # Right Bar: SB Power
            sb_frame = tk.Frame(visual_frame)
            sb_frame.pack(side="left", fill="y", padx=(5, 0))
            tk.Label(sb_frame, text="SB", font=("Arial", 8)).pack()
            sb_bar = tk.Canvas(sb_frame, width=20, height=180, bg="lightgray", highlightthickness=1, highlightbackground="gray")
            sb_bar.pack(expand=True, fill="y")
            sb_bar.create_line(0, 90, 20, 90, fill="black", width=2) # Center line
            sb_val_label = tk.Label(sb_frame, text="0%", font=("Arial", 8, "bold"), width=5)
            sb_val_label.pack()
            
            # Legend for the vectors
            legend_frame = ttk.Frame(frame)
            legend_frame.pack(fill="x", padx=5, pady=(0, 5))
            tk.Label(legend_frame, text="■ Wind", fg="red", font=("Arial", 8)).pack(side="left", expand=True)
            tk.Label(legend_frame, text="■ Target", fg="blue", font=("Arial", 8)).pack(side="left", expand=True)
            tk.Label(legend_frame, text="■ Mag", fg="green", font=("Arial", 8)).pack(side="left", expand=True)
            
            # Control Buttons
            btn_frame = ttk.Frame(frame)
            btn_frame.pack(fill="x", padx=5, pady=5)
            
            lock_btn = ttk.Button(btn_frame, text="LOCK", command=lambda idx=i: self.on_lock_click(idx))
            lock_btn.pack(side="left", expand=True, fill="x", padx=1)
            
            dock_btn = ttk.Button(btn_frame, text="DOCK", command=lambda idx=i: self.on_dock_click(idx))
            dock_btn.pack(side="left", expand=True, fill="x", padx=1)
            
            setup_btn = ttk.Button(btn_frame, text="SETUP", command=lambda idx=i: self.on_setup_click(idx))
            setup_btn.pack(side="left", expand=True, fill="x", padx=1)

            # Parameters below
            params_frame = ttk.Frame(frame)
            params_frame.pack(expand=True, fill="both", padx=5, pady=5)
            
            labels = self.create_params_widgets(params_frame)
            
            self.buoy_frames.append({
                'frame': frame,
                'windrose_canvas': windrose_canvas,
                'bb_bar': bb_bar,
                'sb_bar': sb_bar,
                'bb_val_label': bb_val_label,
                'sb_val_label': sb_val_label,
                'volt_bar': volt_bar,
                'lock_btn': lock_btn,
                'dock_btn': dock_btn,
                'wind_arrow': None,
                'tg_arrow': None,
                'mag_arrow': None,
                'params_frame': params_frame,
                'labels': labels,
                'id': None,
                'data': {}.fromkeys(labels.keys(), "N/A")
            })

        # Incoming Data Log at the bottom
        self.log_frame = ttk.LabelFrame(self.master, text="Incoming Raw Data")
        self.log_frame.pack(expand=False, fill="both", padx=15, pady=10)
        
        self.log_text = tk.Text(self.log_frame, height=8, bg="black", fg="lime", font=("Consolas", 9))
        self.scrollbar = ttk.Scrollbar(self.log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=self.scrollbar.set)
        
        self.log_text.pack(side="left", expand=True, fill="both")
        self.scrollbar.pack(side="right", fill="y")

    def on_lock_click(self, idx):
        b = self.buoy_frames[idx]
        if not b['id']: return
        
        current_status = str(b['data'].get("Status", "0")).strip()
        # Action-oriented: if currently locked (12 or 13), send IDLE (7), otherwise send LOCKED (13)
        cmd = 7 if current_status in ["12", "13", "14"] else 13
        self.send_udp_command(b['id'], cmd)

    def on_dock_click(self, idx):
        b = self.buoy_frames[idx]
        if not b['id']: return
        # Send DOCKING (15)
        self.send_udp_command(b['id'], 15)

    def on_setup_click(self, idx):
        b = self.buoy_frames[idx]
        if not b['id']: return
        # Send STOREASDOC (49)
        self.send_udp_command(b['id'], 49)

    def send_udp_command(self, target_id, cmd_id):
        # IDs (sender) = 99, ACK = 0
        base_msg = f"{target_id},99,0,{cmd_id}"
        
        # Calculate CRC
        crc = 0
        for char in base_msg:
            crc ^= ord(char)
        full_msg = f"${base_msg}*{crc:02X}"
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                s.sendto(full_msg.encode(), ("255.255.255.255", 1001))
            self.log_message(f"SENT TO {target_id}: {full_msg}")
        except Exception as e:
            self.log_message(f"SEND ERROR: {e}")

    def log_message(self, message):
        """Thread-safe logging of raw messages to the text widget"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.log_text.insert("end", log_entry)
        self.log_text.see("end")
        
        if float(self.log_text.index("end-1c")) > 50:
            self.log_text.delete("1.0", "2.0")

    def draw_windrose_background(self, canvas):
        center_x, center_y = 100, 100
        radius = 80
        canvas.create_oval(center_x - radius, center_y - radius,
                           center_x + radius, center_y + radius,
                           outline="black")
        
        for angle_deg, label in [(0, "N"), (90, "E"), (180, "S"), (270, "W")]:
            angle_rad = math.radians(angle_deg - 90) # Adjust for canvas Y-axis inverted
            x = center_x + radius * math.cos(angle_rad)
            y = center_y + radius * math.sin(angle_rad)
            canvas.create_text(x, y, text=label, fill="black", font=("Arial", 10, "bold"))

    def create_params_widgets(self, frame):
        labels = {}
        param_names = [
            "Timestamp", "IDs",
            "Target Dist (tgDist)", "Target Dir (tgDir)", "Magnetic Dir (mDir)",
            "GPS Dir (gpsDir)", "Wind Dir (wDir)", "Wind StdDev (wStd)",
            "Bow Thruster (BB)", "Stern Thruster (SB)",
            "PID I-term (IP)", "PID R-term (IR)",
            "Sub Battery V",
            "Latitude (Lat)", "Longitude (Lon)", "GPS Fix", "GPS Satellites (GpsSat)"
        ]
        for i, name in enumerate(param_names):
            ttk.Label(frame, text=f"{name}:").grid(row=i, column=0, sticky="w", padx=2, pady=1)
            labels[name] = ttk.Label(frame, text="N/A", foreground="blue")
            labels[name].grid(row=i, column=1, sticky="w", padx=2, pady=1)
        return labels

    def udp_listener(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                message = data.decode('utf-8').strip()
                self.log_message(message)
                self.parse_message(message)
            except socket.timeout:
                pass
            except Exception as e:
                print(f"UDP Error: {e}")

    def update_buoy_data(self, buoy_id, data):
        for b in self.buoy_frames:
            if b['id'] == buoy_id:
                b['data'].update(data)
                return
        
        for b in self.buoy_frames:
            if b['id'] is None:
                b['id'] = buoy_id
                b['frame'].config(text=f"Buoy: {buoy_id}")
                b['data'].update(data)
                return

    def parse_message(self, message):
        current_timestamp = datetime.now().strftime("%H:%M:%S")
        comma_sep_pattern = r"^\$([^,]*),([^,]*),([^,]*),([^,]*),"
        initial_match = re.match(comma_sep_pattern, message)
        
        if initial_match:
            cmd = initial_match.group(4)
            buoy_id = initial_match.group(2)
            data = {"Timestamp": current_timestamp}

            if cmd == "51": # TOPDATA
                topdata_regex = re.match(
                    r"^\$([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),"
                    r"([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),"
                    r"([^,]*),([^,]*),([^,]*),([^,]*),"
                    r"([^,]*),([^,]*),"
                    r"([^,]*),([^,]*),"
                    r"([^,]*),([^\*]*)\*([0-9A-F]+)$",
                    message
                )
                if topdata_regex:
                    groups = topdata_regex.groups()
                    data.update({
                        "IDr": groups[0], "IDs": groups[1], "ACK": groups[2], "CMD": groups[3], "Status": groups[4],
                        "Magnetic Dir (mDir)": groups[5], "GPS Dir (gpsDir)": groups[6],
                        "Target Dir (tgDir)": groups[7], "Target Dist (tgDist)": groups[8],
                        "Wind Dir (wDir)": groups[9], "Wind StdDev (wStd)": groups[10],
                        "Bow Thruster (BB)": groups[11], "Stern Thruster (SB)": groups[12],
                        "PID I-term (IP)": groups[13], "PID R-term (IR)": groups[14],
                        "Sub Battery V": groups[15], "Sub Battery %": groups[16],
                        "Latitude (Lat)": groups[17], "Longitude (Lon)": groups[18],
                        "GPS Fix": groups[19], "GPS Satellites (GpsSat)": groups[20]
                    })
                    self.update_buoy_data(buoy_id, data)

            elif cmd == "47": # DIRDIST
                dirdist_regex = re.match(r"^\$([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^\*]*)\*([0-9A-F]+)$", message)
                if dirdist_regex:
                    groups = dirdist_regex.groups()
                    data.update({
                        "IDr": groups[0], "IDs": groups[1], "ACK": groups[2], "CMD": groups[3], "Status": groups[4],
                        "Target Dir (tgDir)": groups[5], "Target Dist (tgDist)": groups[6]
                    })
                    self.update_buoy_data(buoy_id, data)

            elif cmd == "7": # IDLE
                idle_regex = re.match(r"^\$([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^\*]*)\*([0-9A-F]+)$", message)
                if idle_regex:
                    groups = idle_regex.groups()
                    data.update({
                        "IDr": groups[0], "IDs": groups[1], "ACK": groups[2], "CMD": groups[3], "Status": groups[4]
                    })
                    self.update_buoy_data(buoy_id, data)

            elif cmd == "19": # BUOYPOS
                buoypos_regex = re.match(r"^\$([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^\*]*)\*([0-9A-F]+)$", message)
                if buoypos_regex:
                    groups = buoypos_regex.groups()
                    data.update({
                        "IDr": groups[0], "IDs": groups[1], "ACK": groups[2], "CMD": groups[3], "Status": groups[4],
                        "Latitude (Lat)": groups[5], "Longitude (Lon)": groups[6],
                        "Magnetic Dir (mDir)": groups[7], "Wind Dir (wDir)": groups[8], "Wind StdDev (wStd)": groups[9],
                        "Bow Thruster (BB)": groups[10], "Stern Thruster (SB)": groups[11],
                        "GPS Fix": groups[12], "GPS Satellites (GpsSat)": groups[13]
                    })
                    self.update_buoy_data(buoy_id, data)

    def update_gui(self):
        for b in self.buoy_frames:
            if b['id'] is not None:
                data = b['data']
                labels = b['labels']
                
                # Update Lock Button Text (Action-oriented)
                current_status = str(data.get("Status", "0")).strip()
                if current_status in ["12", "13", "14"]: # LOCKED
                    b['lock_btn'].config(text="IDLE")
                elif current_status == "7": # IDLE
                    b['lock_btn'].config(text="LOCKED")
                
                # Update Dock Button Text
                if current_status in ["15", "16", "17"]: # DOCKING
                    b['dock_btn'].config(text="DOCKING")
                else:
                    b['dock_btn'].config(text="DOCK")
                
                tg_dir = data.get("Target Dir (tgDir)", "N/A")
                w_dir = data.get("Wind Dir (wDir)", "N/A")
                if current_status == "7": # Hide in IDLE
                    tg_dir = "N/A"
                    w_dir = "N/A"
                
                self.update_windrose(b, w_dir, tg_dir, data.get("Magnetic Dir (mDir)", "N/A"))
                
                for name, label_widget in labels.items():
                    if name in ["Bow Thruster (BB)", "Stern Thruster (SB)"]:
                        val = data.get(name, "N/A")
                        canvas = b['bb_bar'] if name == "Bow Thruster (BB)" else b['sb_bar']
                        label = b['bb_val_label'] if name == "Bow Thruster (BB)" else b['sb_val_label']
                        if val != "N/A":
                            try:
                                v = float(val)
                                canvas.delete("bar")
                                bar_height = abs(v) * 0.9
                                if v >= 0:
                                    canvas.create_rectangle(0, 90 - bar_height, 20, 90, fill="green", outline="green", tags="bar")
                                else:
                                    canvas.create_rectangle(0, 90, 20, 90 + bar_height, fill="red", outline="red", tags="bar")
                                label.config(text=f"{val}%")
                            except ValueError: pass
                        label_widget.config(text=f"{val}%" if val != "N/A" else val)
                    elif name == "Sub Battery V":
                        val = data.get(name, "N/A")
                        if val != "N/A":
                            try:
                                v = float(val)
                                b['volt_bar']['value'] = max(0, min(8.2, v - 17.0))
                            except ValueError: pass
                        label_widget.config(text=val)
                    else:
                        label_widget.config(text=data.get(name, "N/A"))

        self.master.after(500, self.update_gui)

    def update_windrose(self, b, w_dir_str, tg_dir_str, m_dir_str):
        canvas = b['windrose_canvas']
        def draw_arrow(dir_str, arrow_key, color, length, width):
            if b.get(arrow_key): canvas.delete(b[arrow_key])
            b[arrow_key] = None
            if dir_str != "N/A" and dir_str != "nan":
                try:
                    angle_rad = math.radians(float(dir_str) - 90)
                    end_x = 100 + length * math.cos(angle_rad)
                    end_y = 100 + length * math.sin(angle_rad)
                    b[arrow_key] = canvas.create_line(100, 100, end_x, end_y, arrow=tk.LAST, width=width, fill=color)
                except ValueError: pass
        draw_arrow(w_dir_str, 'wind_arrow', 'red', 70, 2)
        draw_arrow(tg_dir_str, 'tg_arrow', 'blue', 60, 2)
        draw_arrow(m_dir_str, 'mag_arrow', 'green', 50, 3)

    def on_closing(self):
        self.running = False
        self.sock.close()
        self.udp_thread.join()
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = RoboMonitor(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
