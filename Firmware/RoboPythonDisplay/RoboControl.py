import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import socket
import threading
import time
import re
import math
import webbrowser
from datetime import datetime
import serial
import serial.tools.list_ports
import sys
import os

class MsgType:
    IDLE = 7
    IDELING = 8
    PING = 9
    PONG = 10
    ERROR = 11
    LOCKING = 12
    LOCKED = 13
    LOCK_POS = 14
    DOCKING = 15
    DOCKED = 16
    DOC = 17
    STOREASDOC = 18
    BUOYPOS = 19
    SETLOCKPOS = 20
    LOCKPOS = 21
    SETDOCKPOS = 22
    DOCKPOS = 23
    UNLOCK = 24
    REMOTE = 25
    REMOTEING = 26
    DIRDIST = 47
    TOPDATA = 51
    PIDRUDDER = 55
    PIDRUDDERSET = 56
    PIDSPEED = 57
    PIDSPEEDSET = 58
    MAXMINPWR = 68
    MAXMINPWRSET = 69
    STORE_COMPASS_OFFSET = 75
    INFIELD_CALIBRATE = 77
    INFIELD_OFFSET_CALIBRATE = 78
    RESET_RUDDER_PID = 79
    RESET_SPEED_PID = 80
    RESET_SPEED_RUD_PID = 81
    WAKEUP = 82
    SETUPDATA = 83

class RoboMonitor:
    def __init__(self, master):
        self.master = master
        master.title("Robobuoy Monitor - 3 Buoys")
        master.geometry("1400x950")
        master.resizable(True, False)

        self.udp_ip = "0.0.0.0"
        self.udp_port = 1001
        
        self.data_lock = threading.Lock()
        self.networking_ok = True
        self.serial_conn = None
        self.serial_thread = None
        self.running = True
        self.running_serial = False

        self.buoy_frames = []
        self.create_widgets()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)
        try:
            self.sock.bind((self.udp_ip, self.udp_port))
        except Exception as e:
            print(f"Error binding UDP: {e}")
            self.networking_ok = False
            self.log_message(f"CRITICAL: Failed to bind UDP port {self.udp_port}.")

        if self.networking_ok:
            self.udp_thread = threading.Thread(target=self.udp_listener)
            self.udp_thread.daemon = True
            self.udp_thread.start()

        self.update_gui()

    def calculate_crc(self, content):
        crc = 0
        for char in content:
            crc ^= ord(char)
        return crc

    def create_widgets(self):
        self.serial_bar = ttk.Frame(self.master)
        self.serial_bar.pack(fill="x", padx=10, pady=5)
        
        tk.Label(self.serial_bar, text="LoRa Controller Port:", font=("Arial", 9, "bold")).pack(side="left", padx=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(self.serial_bar, textvariable=self.port_var, width=15)
        self.port_combo.pack(side="left", padx=5)
        
        self.refresh_btn = ttk.Button(self.serial_bar, text="↻", width=3, command=self.refresh_ports)
        self.refresh_btn.pack(side="left", padx=2)
        self.refresh_ports()
        
        self.connect_btn = ttk.Button(self.serial_bar, text="Connect LoRa", command=self.toggle_serial)
        self.connect_btn.pack(side="left", padx=5)
        self.serial_status_label = tk.Label(self.serial_bar, text="Disconnected", fg="red")
        self.serial_status_label.pack(side="left", padx=10)

        self.columns_frame = ttk.Frame(self.master)
        self.columns_frame.pack(expand=True, fill="both", padx=10, pady=5)

        for i in range(3):
            frame = ttk.LabelFrame(self.columns_frame, text=f"Buoy {i+1} (Waiting...)")
            frame.grid(row=0, column=i, sticky="nsew", padx=5, pady=5)
            self.columns_frame.columnconfigure(i, weight=1)
            
            ip_header_label = tk.Label(frame, text="", font=("Arial", 8), fg="gray")
            ip_header_label.place(relx=1.0, x=-5, y=-15, anchor="ne")
            
            network_frame = tk.Frame(frame)
            network_frame.place(relx=1.0, x=-5, y=5, anchor="ne")
            udp_indicator = tk.Label(network_frame, text="UDP: --", font=("Arial", 8, "bold"))
            udp_indicator.pack(anchor="e")
            lora_indicator = tk.Label(network_frame, text="LoRa: --", font=("Arial", 8, "bold"))
            lora_indicator.pack(anchor="e")
            sync_indicator = tk.Label(network_frame, text="Data: --", font=("Arial", 8, "bold"))
            sync_indicator.pack(anchor="e")
            
            visual_frame = tk.Frame(frame)
            visual_frame.pack(padx=5, pady=5)
            
            bb_frame = tk.Frame(visual_frame)
            bb_frame.pack(side="left", fill="y", padx=(0, 5))
            tk.Label(bb_frame, text="BB", font=("Arial", 8)).pack()
            bb_bar = tk.Canvas(bb_frame, width=20, height=180, bg="lightgray", highlightthickness=1)
            bb_bar.pack(expand=True, fill="y")
            bb_bar.create_line(0, 90, 20, 90, fill="black", width=2)
            bb_val_label = tk.Label(bb_frame, text="0%", font=("Arial", 8, "bold"), width=5)
            bb_val_label.pack()
            is_label = tk.Label(bb_frame, text="Is: -", font=("Arial", 8), fg="purple")
            is_label.pack()
            
            center_frame = tk.Frame(visual_frame)
            center_frame.pack(side="left")
            status_label = tk.Label(center_frame, text="UNKNOWN", font=("Arial", 12, "bold"), fg="darkblue")
            status_label.pack(pady=(0, 5))
            
            windrose_canvas = tk.Canvas(center_frame, width=200, height=200, bg="white")
            windrose_canvas.pack()
            self.draw_windrose_background(windrose_canvas)
            dist_text = windrose_canvas.create_text(2, 2, anchor="nw", text="-", font=("Arial", 12, "bold"), fill="red")
            wind_text = windrose_canvas.create_text(198, 2, anchor="ne", text="-", font=("Arial", 10, "bold"), fill="blue")
            tg_dir_text = windrose_canvas.create_text(2, 198, anchor="sw", text="", font=("Arial", 11, "bold"), fill="red")
            mag_dir_text = windrose_canvas.create_text(198, 198, anchor="se", text="Mag:-", font=("Arial", 11, "bold"), fill="green")
            
            volt_frame = tk.Frame(center_frame)
            volt_frame.pack(fill="x", pady=(5, 0))
            tk.Label(volt_frame, text="17V", font=("Arial", 7)).pack(side="left")
            
            volt_bar_container = tk.Frame(volt_frame)
            volt_bar_container.pack(side="left", expand=True, fill="x", padx=2)
            
            volt_bar = ttk.Progressbar(volt_bar_container, orient="horizontal", mode="determinate", maximum=8.2)
            volt_bar.pack(expand=True, fill="both")
            
            volt_val_label = tk.Label(volt_bar_container, text="0.0V", font=("Arial", 8, "bold"))
            volt_val_label.place(relx=0.5, rely=0.5, anchor="center")
            
            tk.Label(volt_frame, text="25V", font=("Arial", 7)).pack(side="left")
            
            curr_frame = tk.Frame(center_frame)
            curr_frame.pack(fill="x", pady=(2, 0))
            tk.Label(curr_frame, text="-5A", font=("Arial", 7)).pack(side="left")
            
            curr_bar_container = tk.Frame(curr_frame)
            curr_bar_container.pack(side="left", expand=True, fill="x", padx=2)
            
            curr_bar = ttk.Progressbar(curr_bar_container, orient="horizontal", mode="determinate", maximum=25.0)
            curr_bar.pack(expand=True, fill="both")
            
            curr_val_label = tk.Label(curr_bar_container, text="0.0A", font=("Arial", 8, "bold"))
            curr_val_label.place(relx=0.5, rely=0.5, anchor="center")
            
            tk.Label(curr_frame, text="20A", font=("Arial", 7)).pack(side="left")
            
            sb_frame = tk.Frame(visual_frame)
            sb_frame.pack(side="left", fill="y", padx=(5, 0))
            tk.Label(sb_frame, text="SB", font=("Arial", 8)).pack()
            sb_bar = tk.Canvas(sb_frame, width=20, height=180, bg="lightgray", highlightthickness=1)
            sb_bar.pack(expand=True, fill="y")
            sb_bar.create_line(0, 90, 20, 90, fill="black", width=2)
            sb_val_label = tk.Label(sb_frame, text="0%", font=("Arial", 8, "bold"), width=5)
            sb_val_label.pack()
            ir_label = tk.Label(sb_frame, text="Ir: -", font=("Arial", 8), fg="purple")
            ir_label.pack()
            
            legend_frame = ttk.Frame(frame)
            legend_frame.pack(fill="x", padx=5, pady=(0, 5))
            tk.Label(legend_frame, text="■ Target", fg="red", font=("Arial", 8)).pack(side="left", expand=True)
            tk.Label(legend_frame, text="■ Mag", fg="green", font=("Arial", 8)).pack(side="left", expand=True)
            tk.Label(legend_frame, text="■ Wind", fg="blue", font=("Arial", 8)).pack(side="left", expand=True)
            tk.Label(legend_frame, text="■ GPS", fg="black", font=("Arial", 8)).pack(side="left", expand=True)
            
            btn_frame = ttk.Frame(frame)
            btn_frame.pack(fill="x", padx=5, pady=5)
            lock_btn = ttk.Button(btn_frame, text="LOCK", command=lambda idx=i: self.on_lock_click(idx))
            lock_btn.pack(side="left", expand=True, fill="x", padx=1)
            dock_btn = ttk.Button(btn_frame, text="DOCK", command=lambda idx=i: self.on_dock_click(idx))
            dock_btn.pack(side="left", expand=True, fill="x", padx=1)
            setup_btn = ttk.Button(btn_frame, text="SETUP", command=lambda idx=i: self.on_setup_click(idx))
            setup_btn.pack(side="left", expand=True, fill="x", padx=1)

            dirdist_frame = ttk.Frame(frame)
            dirdist_frame.pack(fill="x", padx=5, pady=2)
            tk.Label(dirdist_frame, text="Dir:", font=("Arial", 8)).pack(side="left")
            dir_entry = ttk.Entry(dirdist_frame, width=8)
            dir_entry.pack(side="left", padx=(0, 5))
            tk.Label(dirdist_frame, text="Dist:", font=("Arial", 8)).pack(side="left")
            dist_entry = ttk.Entry(dirdist_frame, width=8)
            dist_entry.pack(side="left", padx=(0, 5))
            def send_dirdist(buoy_idx=i, de=dir_entry, dste=dist_entry):
                self.on_dirdist_click(buoy_idx, de.get(), dste.get())
            dirdist_send_btn = ttk.Button(dirdist_frame, text="Send", width=6, command=send_dirdist)
            dirdist_send_btn.pack(side="left", padx=2)
            map_btn = ttk.Button(dirdist_frame, text="Map", width=6, command=lambda idx=i: self.on_map_click(idx))
            map_btn.pack(side="left", padx=2)

            params_frame = ttk.Frame(frame)
            params_frame.pack(expand=True, fill="both", padx=5, pady=5)
            labels = self.create_params_widgets(params_frame)
            
            udp_enabled_var = tk.BooleanVar(value=True)
            ttk.Checkbutton(frame, text="UDP Enabled", variable=udp_enabled_var).pack(anchor="w", padx=10, pady=(0, 5))
            
            tg_arrow = windrose_canvas.create_line(100, 100, 100, 100, arrow=tk.LAST, width=4, fill="red", state="hidden")
            mag_arrow = windrose_canvas.create_line(100, 100, 100, 100, arrow=tk.LAST, width=3, fill="green", state="hidden")
            wind_arrow = windrose_canvas.create_line(100, 100, 100, 100, arrow=tk.LAST, width=2, fill="blue", state="hidden")
            gps_arrow = windrose_canvas.create_line(100, 100, 100, 100, arrow=tk.LAST, width=3, fill="black", state="hidden")

            self.buoy_frames.append({
                'frame': frame, 'windrose_canvas': windrose_canvas, 'bb_bar': bb_bar, 'syn_indicator': sync_indicator, # placeholder to keep alignment
                'bb_bar': bb_bar, 'sb_bar': sb_bar,
                'bb_val_label': bb_val_label, 'sb_val_label': sb_val_label, 
                'volt_bar': volt_bar, 'volt_val_label': volt_val_label,
                'curr_bar': curr_bar, 'curr_val_label': curr_val_label,
                'is_label': is_label, 'ir_label': ir_label,
                'status_label': status_label, 'ip_header_label': ip_header_label,
                'udp_indicator': udp_indicator, 'lora_indicator': lora_indicator, 'sync_indicator': sync_indicator,
                'last_udp_time': 0, 'last_lora_time': 0, 'last_udp_content': "", 'last_lora_content': "",
                'lock_btn': lock_btn, 'dock_btn': dock_btn, 'dirdist_send_btn': dirdist_send_btn, 'map_btn': map_btn,
                'udp_enabled_var': udp_enabled_var, 'dist_text': dist_text,
                'wind_text': wind_text,
                'tg_dir_text': tg_dir_text, 'mag_dir_text': mag_dir_text, 'wind_arrow': wind_arrow,
                'tg_arrow': tg_arrow, 'mag_arrow': mag_arrow, 'gps_arrow': gps_arrow,
                'params_frame': params_frame, 'labels': labels, 'id': None, 'data': {}
            })

        self.global_btn_frame = ttk.Frame(self.master)
        self.global_btn_frame.pack(fill="x", padx=10, pady=5)
        self.align_startline_btn = ttk.Button(self.global_btn_frame, text="Align Startline", command=self.on_align_startline)
        self.align_startline_btn.pack(side="left", expand=True, fill="x", padx=5)
        self.align_track_btn = ttk.Button(self.global_btn_frame, text="Align Track", command=self.on_align_track)
        self.align_track_btn.pack(side="left", expand=True, fill="x", padx=5)
        self.dock_all_btn = ttk.Button(self.global_btn_frame, text="Dock All", command=self.on_dock_all)
        self.dock_all_btn.pack(side="left", expand=True, fill="x", padx=5)

        self.logs_container = ttk.Frame(self.master)
        self.logs_container.pack(expand=True, fill="both", padx=15, pady=5)
        self.log_windows = {}
        log_configs = [("UDP", 0, 0, "lime"), ("LORA", 0, 1, "yellow")]
        for name, row, col, color in log_configs:
            f = ttk.LabelFrame(self.logs_container, text=name)
            f.grid(row=row, column=col, sticky="nsew", padx=2, pady=2)
            self.logs_container.grid_columnconfigure(col, weight=1)
            self.logs_container.grid_rowconfigure(row, weight=1)
            txt = tk.Text(f, height=8, bg="black", fg=color, font=("Consolas", 8))
            sb = ttk.Scrollbar(f, command=txt.yview)
            txt.configure(yscrollcommand=sb.set)
            txt.pack(side="left", expand=True, fill="both")
            sb.pack(side="right", fill="y")
            self.log_windows[name] = txt

    def log_message(self, message, source="UDP"):
        def _log():
            ts = datetime.now().strftime("%H:%M:%S")
            # Prefix with TX/RX if applicable
            prefix = ""
            if "IN" in source: prefix = "RX "
            elif "OUT" in source: prefix = "TX "
            
            entry = f"[{ts}] {prefix}{message}\n"
            
            # Map source to consolidated window
            target_key = "LORA" if "LORA" in source.upper() else "UDP"
            target = self.log_windows.get(target_key)
            if target:
                target.insert("end", entry)
                target.see("end")
                if float(target.index("end-1c")) > 200: target.delete("1.0", "2.0")
        self.master.after(0, _log)

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_var.get(): self.port_var.set(ports[0])

    def toggle_serial(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.running_serial = False
            self.serial_conn.close()
            self.serial_status_label.config(text="Disconnected", fg="red")
            self.connect_btn.config(text="Connect LoRa")
        else:
            port = self.port_var.get()
            if not port: return
            try:
                self.serial_conn = serial.Serial(port, 115200, timeout=1)
                self.serial_conn.dtr = False
                self.serial_conn.rts = False
                self.running_serial = True
                self.serial_thread = threading.Thread(target=self.serial_reader)
                self.serial_thread.daemon = True
                self.serial_thread.start()
                self.serial_status_label.config(text=f"Connected: {port}", fg="green")
                self.connect_btn.config(text="Disconnect")
            except Exception as e:
                self.log_message(f"SERIAL ERROR: {e}")

    def serial_reader(self):
        while self.running and self.running_serial:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.log_message(line, "LORA IN")
                        self.master.after(0, lambda msg=line: self.parse_message(msg, "LoRa"))
                except: break
            time.sleep(0.01)

    def udp_listener(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                msg = data.decode('utf-8').strip()
                self.log_message(msg, "UDP IN")
                self.parse_message(msg, addr[0])
            except socket.timeout: pass
            except Exception as e: print(f"UDP Error: {e}")

    def parse_message(self, message, sender_ip):
        ts = datetime.now().strftime("%H:%M:%S")
        log_source = "LORA IN" if sender_ip == "LoRa" else "UDP IN"
        if not message.startswith('$') or '*' not in message: return
        try:
            content, crc_str = message[1:].split('*', 1)
            if int(crc_str, 16) != self.calculate_crc(content):
                self.log_message(f"CRC ERROR in message: {message}", log_source)
                return
            fields = [f if f != "" else "0" for f in content.split(',')]
            if len(fields) < 5: return
            
            # Protocol: $Target,Sender,ACK,CMD,Status,...
            target_id = fields[0].lower()
            sender_id = fields[1].lower()
            
            # 1. Echo prevention: if the sender is "99", it's from us. Ignore.
            if sender_id == "99": return
            
            # 2. Identify the buoy frame
            target_buoy = None
            with self.data_lock:
                # First check for an exact match of either sender or target in our known IDs
                for b in self.buoy_frames:
                    if b['id'] is not None and (b['id'] == sender_id or b['id'] == target_id):
                        target_buoy = b
                        break
                
                # If still no match, and sender_id looks like a hex ID, assign it to an empty frame
                if not target_buoy and all(c in "0123456789abcdef" for c in sender_id) and len(sender_id) >= 4:
                    for b in self.buoy_frames:
                        if b['id'] is None:
                            b['id'] = sender_id
                            b['frame'].config(text=f"Buoy: {sender_id}")
                            target_buoy = b
                            break
            
            if not target_buoy: return
            buoy_id = target_buoy['id']
            
            if sender_ip != "LoRa" and not target_buoy['udp_enabled_var'].get(): return
            if sender_ip == "LoRa":
                target_buoy['last_lora_time'], target_buoy['last_lora_content'] = time.time(), content
            else:
                target_buoy['last_udp_time'], target_buoy['last_udp_content'] = time.time(), content

            cmd = int(fields[3])
            ack = fields[2]
            data = {"Timestamp": ts, "IP": sender_ip, "ACK": ack, "Status": fields[4]}

            # 3. Mode/Data Consistency: Only accept primary data if ACK is LORAINF (6)
            # This prevents echoed requests (ACK=1) from being treated as responses.
            is_info_packet = (ack == "6")

            if cmd == MsgType.TOPDATA and is_info_packet and len(fields) >= 21:
                data.update({
                    "Magnetic Dir": fields[5], "GPS Dir": fields[6], "Target Dir": fields[7], "Target Dist": fields[8],
                    "Wind Dir": fields[9], "Wind StdDev": fields[10], "Bow Thruster (BB)": fields[11], "Stern Thruster (SB)": fields[12],
                    "PID I-term": fields[13], "PID R-term": fields[14], "Sub Battery V": fields[15], "Sub Battery %": fields[16],
                    "Latitude (Lat)": fields[17], "Longitude (Lon)": fields[18], "GPS Fix": fields[19], "GPS Satellites": fields[20], 
                    "Current": fields[21] if len(fields)>21 else "0"
                })
            elif cmd == MsgType.DIRDIST and len(fields) >= 7:
                data.update({"Target Dir": fields[5], "Target Dist": fields[6]})
            elif cmd == MsgType.BUOYPOS and len(fields) >= 14:
                data.update({"Latitude (Lat)": fields[5], "Longitude (Lon)": fields[6], "Magnetic Dir": fields[7], "Wind Dir": fields[8], "Bow Thruster (BB)": fields[10], "Stern Thruster (SB)": fields[11]})
            elif cmd in [MsgType.PIDRUDDER, MsgType.PIDRUDDERSET] and is_info_packet and len(fields) >= 8:
                data.update({"Kpr": fields[5], "Kir": fields[6], "Kdr": fields[7]})
            elif cmd in [MsgType.PIDSPEED, MsgType.PIDSPEEDSET] and is_info_packet and len(fields) >= 8:
                data.update({"Kps": fields[5], "Kis": fields[6], "Kds": fields[7]})
            elif cmd in [MsgType.MAXMINPWR, MsgType.MAXMINPWRSET] and is_info_packet and len(fields) >= 7:
                data.update({"maxSpeed": fields[5], "minSpeed": fields[6], "pivotSpeed": fields[7] if len(fields)>7 else "0.2"})
            elif cmd == MsgType.SETUPDATA and is_info_packet and len(fields) >= 14:
                data.update({
                    "Kpr": fields[5], "Kir": fields[6], "Kdr": fields[7], "Kps": fields[8], "Kis": fields[9], "Kds": fields[10],
                    "maxSpeed": fields[11], "minSpeed": fields[12], "pivotSpeed": fields[13], "compassOffset": fields[14] if len(fields)>14 else "0",
                    "minOfsetDist": fields[15] if len(fields)>15 else "2", "revBB": fields[16] if len(fields)>16 else "0",
                    "revSB": fields[17] if len(fields)>17 else "0", "swap_BB_SB": fields[18] if len(fields)>18 else "0"
                })
            self.update_buoy_data(buoy_id, data)
        except Exception as e: print(f"Parse error: {e}")

    def update_buoy_data(self, buoy_id, data):
        with self.data_lock:
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

    def update_gui(self):
        ct = time.time()
        with self.data_lock:
            for b in self.buoy_frames:
                d = b['data']
                try: curr_stat = int(d.get("Status", "0"))
                except: curr_stat = 0
                
                b['udp_indicator'].config(text="UDP: OK" if b['last_udp_time']>0 and (ct-b['last_udp_time']<5) else "UDP: --", fg="green" if b['last_udp_time']>0 and (ct-b['last_udp_time']<5) else "gray")
                b['lora_indicator'].config(text="LoRa: OK" if b['last_lora_time']>0 and (ct-b['last_lora_time']<5) else "LoRa: --", fg="green" if b['last_lora_time']>0 and (ct-b['last_lora_time']<5) else "gray")
                if b['last_udp_content'] and b['last_lora_content']:
                    b['sync_indicator'].config(text="Data: SYNC" if b['last_udp_content']==b['last_lora_content'] else "Data: DIFF", fg="green" if b['last_udp_content']==b['last_lora_content'] else "red")
                else: b['sync_indicator'].config(text="Data: WAIT", fg="gray")

                m_dir = d.get("Magnetic Dir", "N/A")
                b['windrose_canvas'].itemconfig(b['mag_dir_text'], text=f"Mag:{float(m_dir):.0f}°" if m_dir not in ["N/A", "nan", ""] else "Mag:-")

                if curr_stat == MsgType.IDLE:
                    b['windrose_canvas'].itemconfig(b['dist_text'], text="-")
                    b['windrose_canvas'].itemconfig(b['tg_dir_text'], text="-")
                    b['windrose_canvas'].itemconfig(b['wind_text'], text="-")
                    b['is_label'].config(text="Is: -")
                    b['ir_label'].config(text="Ir: -")
                else:
                    if curr_stat in [MsgType.LOCKING, MsgType.LOCKED, MsgType.DOCKING, MsgType.DOCKED]:
                        dist = d.get("Target Dist", "0")
                        b['windrose_canvas'].itemconfig(b['dist_text'], text=f"{float(dist):.2f}m" if dist not in ["N/A", "nan", ""] else "0.00m")
                    else:
                        b['windrose_canvas'].itemconfig(b['dist_text'], text="-")
                    
                    if curr_stat in [MsgType.LOCKING, MsgType.LOCKED]:
                        w_dir = d.get("Wind Dir", "0")
                        w_std = d.get("Wind StdDev", "0")
                        try:
                            b['windrose_canvas'].itemconfig(b['wind_text'], text=f"Wind:{float(w_dir):.0f}°\nstd:{float(w_std):.0f}")
                        except:
                            b['windrose_canvas'].itemconfig(b['wind_text'], text="-")
                    else:
                        b['windrose_canvas'].itemconfig(b['wind_text'], text="-")

                    pi, pr = d.get("PID I-term", "0"), d.get("PID R-term", "0")
                    try:
                        b['is_label'].config(text=f"Is:{float(pi):.1f}")
                        b['ir_label'].config(text=f"Ir:{float(pr):.1f}")
                    except: pass
                    if curr_stat in [MsgType.LOCKING, MsgType.LOCKED, MsgType.DOCKING, MsgType.DOCKED]:
                        tdir = d.get("Target Dir", "N/A")
                        b['windrose_canvas'].itemconfig(b['tg_dir_text'], text=f"Tg:{float(tdir):.0f}°" if tdir not in ["N/A", "nan", ""] else "-")
                    else:
                        b['windrose_canvas'].itemconfig(b['tg_dir_text'], text="-")

                if b['id'] is not None:
                    st_txt = "IDLE" if curr_stat==MsgType.IDLE else "LOCKED" if curr_stat in [MsgType.LOCKING, MsgType.LOCKED] else "DOCKING" if curr_stat in [MsgType.DOCKING, MsgType.DOCKED] else f"STATUS {curr_stat}"
                    b['status_label'].config(text=st_txt)
                    b['ip_header_label'].config(text=d.get("IP", ""))
                    b['lock_btn'].config(text="IDLE" if curr_stat in [MsgType.LOCKING, MsgType.LOCKED] else "LOCK")
                    b['dock_btn'].config(text="IDLE" if curr_stat in [MsgType.DOCKING, MsgType.DOCKED] else "DOCK")
                    
                    try: f_dist = float(d.get("Target Dist", "0"))
                    except: f_dist = 0
                    self.update_windrose(b, d.get("Wind Dir", "N/A") if curr_stat in [MsgType.LOCKING, MsgType.LOCKED] and f_dist<=10.0 else "N/A", d.get("Target Dir", "N/A") if curr_stat!=MsgType.IDLE else "N/A", d.get("Magnetic Dir", "N/A"), d.get("GPS Dir", "N/A") if f_dist>10.0 else "N/A")

                    for thrust in ["Bow Thruster (BB)", "Stern Thruster (SB)"]:
                        try: val = float(d.get(thrust, "0")) if curr_stat!=MsgType.IDLE else 0
                        except: val = 0
                        canv, lbl = (b['bb_bar'], b['bb_val_label']) if thrust=="Bow Thruster (BB)" else (b['sb_bar'], b['sb_val_label'])
                        canv.delete("bar")
                        if val!=0: canv.create_rectangle(0, 90, 20, 90-val*0.9, fill="red" if val<0 else "green", tags="bar")
                        lbl.config(text=f"{int(val)}%")
                    
                    try: v_val = float(d.get("Sub Battery V", "0.0"))
                    except: v_val = 0.0
                    b['volt_bar']['value'] = max(0, min(8.2, v_val - 17.0))
                    b['volt_val_label'].config(text=f"{v_val:.1f}V")
                    
                    try: c_val = float(d.get("Current", "0.0"))
                    except: c_val = 0.0
                    b['curr_bar']['value'] = max(0, min(25.0, c_val + 5.0))
                    b['curr_val_label'].config(text=f"{c_val:.1f}A")
                    
                    for name, widget in b['labels'].items():
                        if name == "Battery": widget.config(text=f"{v_val:.1f}V")
                        elif name == "Current": 
                            try: widget.config(text=f"{float(d.get('Current','0')):.1f}A")
                            except: widget.config(text="0.0A")
                        else: widget.config(text=d.get(name, "N/A"))

            active_count = sum(1 for b in self.buoy_frames if b['id'] is not None)
            if active_count == 0:
                self.align_startline_btn.config(state="disabled")
                self.align_track_btn.config(state="disabled")
                self.dock_all_btn.config(state="disabled")
            elif active_count == 1:
                self.align_startline_btn.config(state="disabled")
                self.align_track_btn.config(state="disabled")
                self.dock_all_btn.config(state="normal")
            elif active_count == 2:
                self.align_startline_btn.config(state="normal")
                self.align_track_btn.config(state="disabled")
                self.dock_all_btn.config(state="normal")
            else: # 3 or more
                self.align_startline_btn.config(state="normal")
                self.align_track_btn.config(state="normal")
                self.dock_all_btn.config(state="normal")

        self.master.after(500, self.update_gui)

    def update_windrose(self, b, w, t, m, g):
        def draw(val, aid, length):
            if val not in ["N/A", "nan", None]:
                try:
                    rad = math.radians(float(val)-90)
                    b['windrose_canvas'].coords(aid, 100, 100, 100+length*math.cos(rad), 100+length*math.sin(rad))
                    b['windrose_canvas'].itemconfig(aid, state="normal")
                except: b['windrose_canvas'].itemconfig(aid, state="hidden")
            else: b['windrose_canvas'].itemconfig(aid, state="hidden")
        draw(t, b['tg_arrow'], 80); draw(m, b['mag_arrow'], 70); draw(w, b['wind_arrow'], 60); draw(g, b['gps_arrow'], 50)

    def draw_windrose_background(self, canvas):
        canvas.create_oval(20, 20, 180, 180, outline="black")
        for a, l in [(0,"N"),(90,"E"),(180,"S"),(270,"W")]:
            rad = math.radians(a-90)
            canvas.create_text(100+80*math.cos(rad), 100+80*math.sin(rad), text=l, font=("Arial",10,"bold"))

    def create_params_widgets(self, frame):
        labels = {}
        for i, n in enumerate(["Timestamp", "Wind Dir", "Wind StdDev", "PID I-term", "PID R-term", "Battery", "Current", "GPS Fix", "GPS Satellites"]):
            ttk.Label(frame, text=f"{n}:").grid(row=i, column=0, sticky="w", padx=2, pady=1)
            labels[n] = ttk.Label(frame, text="N/A", foreground="blue")
            labels[n].grid(row=i, column=1, sticky="w", padx=2, pady=1)
        return labels

    def on_lock_click(self, idx):
        b = self.buoy_frames[idx]
        if not b['id']: return
        cmd = MsgType.IDELING if int(b['data'].get("Status", "0")) in [MsgType.LOCKING, MsgType.LOCKED, MsgType.LOCK_POS] else MsgType.LOCKING
        self.send_udp_command(b['id'], cmd)

    def on_dock_click(self, idx):
        b = self.buoy_frames[idx]
        if not b['id']: return
        cmd = MsgType.IDELING if int(b['data'].get("Status", "0")) in [MsgType.DOCKING, MsgType.DOCKED, MsgType.DOC] else MsgType.DOCKING
        self.send_udp_command(b['id'], cmd)

    def on_dirdist_click(self, idx, d_val, dist_val):
        b = self.buoy_frames[idx]
        if not b['id']: return
        try:
            msg = f"{b['id']},99,6,{MsgType.DIRDIST},{MsgType.IDLE},{float(d_val):.2f},{float(dist_val):.2f}"
            self.send_custom_udp_command(b['id'], msg)
        except: pass

    def on_map_click(self, idx):
        b = self.buoy_frames[idx]
        lat, lon = b['data'].get("Latitude (Lat)"), b['data'].get("Longitude (Lon)")
        if lat and lon and lat not in ["N/A", "nan"] and lon not in ["N/A", "nan"]:
            webbrowser.open(f"https://www.openstreetmap.org/?mlat={lat}&mlon={lon}#map=18/{lat}/{lon}")

    def on_setup_click(self, idx):
        b = self.buoy_frames[idx]
        if not b['id']: return
        with self.data_lock:
            for key in ["Kpr", "Kir", "Kdr", "Kps", "Kis", "Kds", "maxSpeed", "minSpeed", "pivotSpeed", "compassOffset", "revBB", "revSB", "swap_BB_SB"]:
                b['data'].pop(key, None)
        loading_win = tk.Toplevel(self.master)
        loading_win.title("Loading...")
        loading_win.geometry("250x100")
        ttk.Label(loading_win, text=f"Fetching Setup for Buoy {b['id']}...", font=("Arial", 10)).pack(pady=20)
        def check_data(retries=0):
            if not loading_win.winfo_exists(): return
            if all(k in b['data'] for k in ["Kpr", "maxSpeed", "pivotSpeed"]):
                loading_win.destroy(); self.open_setup_window(b)
            elif retries > 50:
                loading_win.destroy(); messagebox.showerror("Timeout", f"Could not retrieve setup data for Buoy {b['id']}")
            else:
                if retries % 10 == 0: self.send_custom_udp_command(b['id'], f"{b['id']},99,1,{MsgType.SETUPDATA},,,,,,,")
                self.master.after(200, lambda: check_data(retries + 1))
        check_data()

    def open_setup_window(self, b):
        setup_win = tk.Toplevel(self.master)
        setup_win.title(f"Setup Buoy {b['id']}")
        setup_win.geometry("350x650")
        main_frame = ttk.Frame(setup_win, padding="20"); main_frame.pack(fill="both", expand=True)
        def create_entry(parent, label, key, row):
            ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=5)
            ent = ttk.Entry(parent, width=15); ent.grid(row=row, column=1, sticky="e", pady=5)
            ent.insert(0, b['data'].get(key, "")); return ent
        ttk.Label(main_frame, text="Rudder PID", font=("Arial", 10, "bold")).grid(row=0, column=0, columnspan=2, pady=(0, 10))
        kpr, kir, kdr = create_entry(main_frame, "P:", "Kpr", 1), create_entry(main_frame, "I:", "Kir", 2), create_entry(main_frame, "D:", "Kdr", 3)
        ttk.Label(main_frame, text="Speed PID", font=("Arial", 10, "bold")).grid(row=4, column=0, columnspan=2, pady=(15, 10))
        kps, kis, kds = create_entry(main_frame, "P:", "Kps", 5), create_entry(main_frame, "I:", "Kis", 6), create_entry(main_frame, "D:", "Kds", 7)
        ttk.Label(main_frame, text="Limits & Motors", font=("Arial", 10, "bold")).grid(row=8, column=0, columnspan=2, pady=(15, 10))
        max_s, min_s, piv_s, c_off = create_entry(main_frame, "Max Speed:", "maxSpeed", 9), create_entry(main_frame, "Min Speed:", "minSpeed", 10), create_entry(main_frame, "Pivot Speed:", "pivotSpeed", 11), create_entry(main_frame, "Compass Offset:", "compassOffset", 12)
        rev_bb_var, rev_sb_var = tk.BooleanVar(value=b['data'].get("revBB")=="1"), tk.BooleanVar(value=b['data'].get("revSB")=="1")
        ttk.Checkbutton(main_frame, text="Reverse BB", variable=rev_bb_var).grid(row=13, column=0, sticky="w")
        ttk.Checkbutton(main_frame, text="Reverse SB", variable=rev_sb_var).grid(row=13, column=1, sticky="e")
        def save_all():
            try:
                # Use ACK=3 for "Command" and Status=7 (or current status) to ensure the buoy accepts it as an instruction
                curr_status = b['data'].get("Status", "7")
                vals = [kpr.get(), kir.get(), kdr.get(), kps.get(), kis.get(), kds.get(), max_s.get(), min_s.get(), piv_s.get(), c_off.get(), "2", "1" if rev_bb_var.get() else "0", "1" if rev_sb_var.get() else "0", "0"]
                
                # 1. Send the bulk setup data update
                self.send_custom_udp_command(b['id'], f"{b['id']},99,3,{MsgType.SETUPDATA},{curr_status},{','.join(vals)}")
                
                # 2. Also send specific set commands for Rudder, Speed, and Power to be safe
                self.send_custom_udp_command(b['id'], f"{b['id']},99,3,{MsgType.PIDRUDDERSET},{curr_status},{kpr.get()},{kir.get()},{kdr.get()}")
                self.send_custom_udp_command(b['id'], f"{b['id']},99,3,{MsgType.PIDSPEEDSET},{curr_status},{kps.get()},{kis.get()},{kds.get()}")
                self.send_custom_udp_command(b['id'], f"{b['id']},99,3,{MsgType.MAXMINPWRSET},{curr_status},{max_s.get()},{min_s.get()},{piv_s.get()}")
                
                setup_win.destroy()
            except Exception as e: messagebox.showerror("Error", f"Failed to save: {e}")
        ttk.Button(main_frame, text="Save & Close", command=save_all).grid(row=15, column=0, columnspan=2, pady=20)

    def send_custom_udp_command(self, tid, base, use_udp=True, use_lora=None):
        target_buoy = next((b for b in self.buoy_frames if b['id'] == tid), None)
        udp_allowed = target_buoy['udp_enabled_var'].get() if target_buoy else True
        if use_lora is None: use_lora = not (udp_allowed and target_buoy and time.time()-target_buoy['last_udp_time']<5)
        full = f"${base}*{self.calculate_crc(base):02X}"
        if use_udp and udp_allowed:
            tip = target_buoy['data'].get("IP", "255.255.255.255") if target_buoy else "255.255.255.255"
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1); s.sendto(full.encode(), (tip, 1001))
                self.log_message(full, "UDP OUT")
            except Exception as e: self.log_message(f"UDP ERROR: {e}", "UDP OUT")
        if use_lora and self.serial_conn and self.serial_conn.is_open:
            try: self.serial_conn.write((full + "\n").encode()); self.log_message(full, "LORA OUT")
            except Exception as e: self.log_message(f"LORA ERROR: {e}", "LORA OUT")

    def send_udp_command(self, tid, cid): self.send_custom_udp_command(tid, f"{tid},99,3,{cid},7")

    def on_align_startline(self):
        self.log_message("Global: Align Startline Clicked", "UDP OUT")
        # Placeholder for startline alignment logic

    def on_align_track(self):
        self.log_message("Global: Align Track Clicked", "UDP OUT")
        # Placeholder for track alignment logic

    def on_dock_all(self):
        self.log_message("Global: Dock All Clicked", "UDP OUT")
        for i in range(len(self.buoy_frames)):
            if self.buoy_frames[i]['id']:
                self.on_dock_click(i)

    def on_closing(self):
        self.running = False; self.running_serial = False
        if self.serial_conn: self.serial_conn.close()
        self.sock.close(); self.master.destroy(); sys.exit(0)

if __name__ == "__main__":
    root = tk.Tk(); app = RoboMonitor(root); root.protocol("WM_DELETE_WINDOW", app.on_closing); root.mainloop()
