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
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'RoboPython')))
from message_utils import generate_nmea_message, send_message

class RoboMonitor:
    def __init__(self, master):
        self.master = master
        master.title("Robobuoy Monitor - 3 Buoys")
        master.geometry("1000x950")
        master.resizable(False, False)

        self.udp_ip = "0.0.0.0"  # Listen on all available interfaces
        self.udp_port = 1001     # Port RobobuoyTop sends UDP messages to
        
        # Serial connection state
        self.serial_conn = None
        self.serial_thread = None
        self.running = True

        self.buoy_frames = []
        self.create_widgets()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((self.udp_ip, self.udp_port))
        except Exception as e:
            print(f"Error binding UDP: {e}")

        self.udp_thread = threading.Thread(target=self.udp_listener)
        self.udp_thread.daemon = True
        self.udp_thread.start()

        self.update_gui()

    def create_widgets(self):
        # Serial Control Bar at the very top
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
            
            # IP Label at top right of the frame
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
            
            status_label = tk.Label(center_frame, text="UNKNOWN", font=("Arial", 12, "bold"), fg="darkblue")
            status_label.pack(pady=(0, 5))
            
            windrose_canvas = tk.Canvas(center_frame, width=200, height=200, bg="white")
            windrose_canvas.pack()
            self.draw_windrose_background(windrose_canvas)
            
            # Info overlay on canvas (on the border of the windrose field)
            dist_text = windrose_canvas.create_text(2, 2, anchor="nw", text="-", font=("Arial", 12, "bold"), fill="red")
            pid_i_text = windrose_canvas.create_text(198, 2, anchor="ne", text="*", font=("Arial", 12, "bold"), fill="purple")
            
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
            tk.Label(legend_frame, text="■ Target", fg="red", font=("Arial", 8)).pack(side="left", expand=True)
            tk.Label(legend_frame, text="■ Mag", fg="green", font=("Arial", 8)).pack(side="left", expand=True)
            tk.Label(legend_frame, text="■ GPS", fg="black", font=("Arial", 8)).pack(side="left", expand=True)
            tk.Label(legend_frame, text="■ Wind", fg="blue", font=("Arial", 8)).pack(side="left", expand=True)
            
            # Control Buttons
            btn_frame = ttk.Frame(frame)
            btn_frame.pack(fill="x", padx=5, pady=5)
            
            lock_btn = ttk.Button(btn_frame, text="LOCK", command=lambda idx=i: self.on_lock_click(idx))
            lock_btn.pack(side="left", expand=True, fill="x", padx=1)
            
            dock_btn = ttk.Button(btn_frame, text="DOCK", command=lambda idx=i: self.on_dock_click(idx))
            dock_btn.pack(side="left", expand=True, fill="x", padx=1)
            
            setup_btn = ttk.Button(btn_frame, text="SETUP", command=lambda idx=i: self.on_setup_click(idx))
            setup_btn.pack(side="left", expand=True, fill="x", padx=1)

            # DIRDIST Inputs
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

            # Calibration Buttons
            calib_frame = ttk.Frame(frame)
            calib_frame.pack(fill="x", padx=5, pady=2)
            
            calib_comp_btn = ttk.Button(calib_frame, text="In-Field Calib", command=lambda idx=i: self.on_infield_calib_click(idx))
            calib_comp_btn.pack(side="left", expand=True, fill="x", padx=1)
            
            calib_off_btn = ttk.Button(calib_frame, text="In-Field Offset", command=lambda idx=i: self.on_infield_offset_click(idx))
            calib_off_btn.pack(side="left", expand=True, fill="x", padx=1)

            # Parameters below
            params_frame = ttk.Frame(frame)
            params_frame.pack(expand=True, fill="both", padx=5, pady=5)
            
            labels = self.create_params_widgets(params_frame)
            
            udp_enabled_var = tk.BooleanVar(value=True)
            udp_toggle = ttk.Checkbutton(frame, text="UDP Enabled", variable=udp_enabled_var)
            udp_toggle.pack(anchor="w", padx=10, pady=(0, 5))
            
            self.buoy_frames.append({
                'frame': frame,
                'windrose_canvas': windrose_canvas,
                'bb_bar': bb_bar,
                'sb_bar': sb_bar,
                'bb_val_label': bb_val_label,
                'sb_val_label': sb_val_label,
                'volt_bar': volt_bar,
                'status_label': status_label,
                'ip_header_label': ip_header_label,
                'udp_indicator': udp_indicator,
                'lora_indicator': lora_indicator,
                'sync_indicator': sync_indicator,
                'last_udp_time': 0,
                'last_lora_time': 0,
                'last_udp_content': "",
                'last_lora_content': "",
                'lock_btn': lock_btn,
                'dock_btn': dock_btn,
                'dirdist_send_btn': dirdist_send_btn,
                'map_btn': map_btn,
                'udp_enabled_var': udp_enabled_var,
                'dist_text': dist_text,
                'pid_i_text': pid_i_text,
                'wind_arrow': None,
                'tg_arrow': None,
                'mag_arrow': None,
                'gps_arrow': None,
                'gps_dir_first_seen': None,
                'params_frame': params_frame,
                'labels': labels,
                'id': None,
                'setup_entries': None,
                'data': {}.fromkeys(["Latitude (Lat)", "Longitude (Lon)", "Bow Thruster (BB)", "Stern Thruster (SB)"] + list(labels.keys()), "N/A")
            })

        # Incoming Data Log at the bottom
        self.log_frame = ttk.LabelFrame(self.master, text="Incoming Raw Data")
        self.log_frame.pack(expand=False, fill="both", padx=15, pady=10)
        
        self.log_text = tk.Text(self.log_frame, height=8, bg="black", fg="lime", font=("Consolas", 9))
        self.scrollbar = ttk.Scrollbar(self.log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=self.scrollbar.set)
        
        self.log_text.pack(side="left", expand=True, fill="both")
        self.scrollbar.pack(side="right", fill="y")

    def on_map_click(self, idx):
        b = self.buoy_frames[idx]
        if not b['id']: return
        lat = b['data'].get("Latitude (Lat)")
        lon = b['data'].get("Longitude (Lon)")
        
        if lat and lon and lat not in ["N/A", "nan", ""] and lon not in ["N/A", "nan", ""]:
            try:
                # Basic validation
                float(lat)
                float(lon)
                url = f"https://www.openstreetmap.org/?mlat={lat}&mlon={lon}#map=18/{lat}/{lon}"
                webbrowser.open(url)
            except ValueError:
                pass

    def on_lock_click(self, idx):
        b = self.buoy_frames[idx]
        if not b['id']: return
        
        current_status = str(b['data'].get("Status", "0")).strip()
        # Action-oriented: if currently locked (12, 13, 14), send IDELING (8), otherwise send LOCKING (12)
        cmd = 8 if current_status in ["12", "13", "14"] else 12
        self.send_udp_command(b['id'], cmd)

    def on_dock_click(self, idx):
        b = self.buoy_frames[idx]
        if not b['id']: return
        
        current_status = str(b['data'].get("Status", "0")).strip()
        # Action-oriented: if currently docking (15, 16, 17), send IDELING (8), otherwise send DOCKING (15)
        cmd = 8 if current_status in ["15", "16", "17"] else 15
        self.send_udp_command(b['id'], cmd)

    def on_dirdist_click(self, idx, dir_val, dist_val):
        b = self.buoy_frames[idx]
        if not b['id']: return
        try:
            d = float(dir_val)
            dist = float(dist_val)
            # CMD=47 (DIRDIST), STATUS=7 (IDLE)
            val_str = f"{format(d, '.2f')},{format(dist, '.2f')}"
            base_msg = f"{b['id']},99,0,47,7,{val_str}"
            self.send_custom_udp_command(b['id'], base_msg)
        except ValueError:
            pass

    def on_setup_click(self, idx):
        b = self.buoy_frames[idx]
        if not b['id']: return
        
        # Clear existing PID data to ensure we fetch fresh values
        for key in ["Kpr", "Kir", "Kdr", "Kps", "Kis", "Kds", "maxSpeed", "minSpeed"]:
            b['data'].pop(key, None)
            
        loading_win = tk.Toplevel(self.master)
        loading_win.title("Loading...")
        loading_win.geometry("250x100")
        loading_win.resizable(False, False)
        
        ttk.Label(loading_win, text=f"Getting PID info for Buoy {b['id']}", font=("Arial", 10, "bold")).pack(pady=(20, 5))
        ttk.Label(loading_win, text="Please wait...").pack()
        
        current_t = time.time()
        udp_active = b['last_udp_time'] > 0 and (current_t - b['last_udp_time'] < 5)
        
        def check_data_and_open(retries=0):
            if not loading_win.winfo_exists():
                return # User closed the loading window
            
            has_rudder = all(k in b['data'] for k in ["Kpr", "Kir", "Kdr"])
            has_speed = all(k in b['data'] for k in ["Kps", "Kis", "Kds"])
            has_maxmin = all(k in b['data'] for k in ["maxSpeed", "minSpeed", "pivotSpeed"])
            
            if has_rudder and has_speed and has_maxmin:
                loading_win.destroy()
                self.open_setup_window(b)
            else:
                if retries >= 12: # 6 seconds total (1s UDP + 5s LoRa, or just 6s LoRa)
                    self.log_message(f"Timeout: Could not retrieve PID data for Buoy {b['id']}")
                    loading_win.destroy()
                    return

                if udp_active:
                    if retries == 0:
                        # Initial attempt: UDP only
                        self.send_custom_udp_command(b['id'], f"{b['id']},99,1,55,0,0,0,0,0,0,0", use_udp=True, use_lora=False)
                        self.send_custom_udp_command(b['id'], f"{b['id']},99,1,57,0,0,0,0,0,0,0", use_udp=True, use_lora=False)
                        self.send_custom_udp_command(b['id'], f"{b['id']},99,1,68,0,0,0,0,0,0,0", use_udp=True, use_lora=False)
                    elif retries >= 2 and (retries - 2) % 4 == 0:
                        # After 1 second (retries >= 2) and every 2 seconds thereafter: LoRa with LORAGETACK
                        self.send_custom_udp_command(b['id'], f"{b['id']},99,3,55,0,0,0,0,0,0,0", use_udp=False, use_lora=True)
                        self.send_custom_udp_command(b['id'], f"{b['id']},99,3,57,0,0,0,0,0,0,0", use_udp=False, use_lora=True)
                        self.send_custom_udp_command(b['id'], f"{b['id']},99,3,68,0,0,0,0,0,0,0", use_udp=False, use_lora=True)
                else:
                    if retries % 4 == 0:
                        # Immediately use LoRa and repeat every 2 seconds
                        self.send_custom_udp_command(b['id'], f"{b['id']},99,3,55,0,0,0,0,0,0,0", use_udp=False, use_lora=True)
                        self.send_custom_udp_command(b['id'], f"{b['id']},99,3,57,0,0,0,0,0,0,0", use_udp=False, use_lora=True)
                        self.send_custom_udp_command(b['id'], f"{b['id']},99,3,68,0,0,0,0,0,0,0", use_udp=False, use_lora=True)
                
                self.master.after(500, check_data_and_open, retries + 1)

        check_data_and_open()

    def open_setup_window(self, b):
        setup_win = tk.Toplevel(self.master)
        setup_win.title(f"Setup Buoy {b['id']}")
        setup_win.geometry("350x600")
        setup_win.resizable(False, False)
        
        def on_setup_close():
            b['setup_entries'] = None
            setup_win.destroy()
        setup_win.protocol("WM_DELETE_WINDOW", on_setup_close)
        
        # Main container with padding
        main_setup_frame = ttk.Frame(setup_win, padding="20")
        main_setup_frame.pack(expand=True, fill="both")

        ttk.Label(main_setup_frame, text="Rudder PID", font=("Arial", 11, "bold")).grid(row=0, column=0, columnspan=2, pady=(0, 10))
        
        ttk.Label(main_setup_frame, text="P:").grid(row=1, column=0, sticky="e", padx=10, pady=5)
        kpr_entry = ttk.Entry(main_setup_frame, width=15)
        kpr_entry.grid(row=1, column=1, sticky="w", pady=5)
        
        ttk.Label(main_setup_frame, text="I:").grid(row=2, column=0, sticky="e", padx=10, pady=5)
        kir_entry = ttk.Entry(main_setup_frame, width=15)
        kir_entry.grid(row=2, column=1, sticky="w", pady=5)
        
        ttk.Label(main_setup_frame, text="D:").grid(row=3, column=0, sticky="e", padx=10, pady=5)
        kdr_entry = ttk.Entry(main_setup_frame, width=15)
        kdr_entry.grid(row=3, column=1, sticky="w", pady=5)
        
        def send_rudder_pid():
            try:
                kpr = float(kpr_entry.get() or 0)
                kir = float(kir_entry.get() or 0)
                kdr = float(kdr_entry.get() or 0)
                val_str = f"{format(kpr, '.10g')},{format(kir, '.10g')},{format(kdr, '.10g')}"
                base_msg = f"{b['id']},99,6,56,0,{val_str},0,0,0"
                self.send_custom_udp_command(b['id'], base_msg)
            except ValueError:
                pass

        ttk.Button(main_setup_frame, text="Send Rudder PID", command=send_rudder_pid).grid(row=4, column=0, columnspan=2, pady=(10, 20))
        
        ttk.Label(main_setup_frame, text="Speed PID", font=("Arial", 11, "bold")).grid(row=5, column=0, columnspan=2, pady=(0, 10))
        
        ttk.Label(main_setup_frame, text="P:").grid(row=6, column=0, sticky="e", padx=10, pady=5)
        kps_entry = ttk.Entry(main_setup_frame, width=15)
        kps_entry.grid(row=6, column=1, sticky="w", pady=5)
        
        ttk.Label(main_setup_frame, text="I:").grid(row=7, column=0, sticky="e", padx=10, pady=5)
        kis_entry = ttk.Entry(main_setup_frame, width=15)
        kis_entry.grid(row=7, column=1, sticky="w", pady=5)
        
        ttk.Label(main_setup_frame, text="D:").grid(row=8, column=0, sticky="e", padx=10, pady=5)
        kds_entry = ttk.Entry(main_setup_frame, width=15)
        kds_entry.grid(row=8, column=1, sticky="w", pady=5)
        
        def send_speed_pid():
            try:
                kps = float(kps_entry.get() or 0)
                kis = float(kis_entry.get() or 0)
                kds = float(kds_entry.get() or 0)
                val_str = f"{format(kps, '.10g')},{format(kis, '.10g')},{format(kds, '.10g')}"
                base_msg = f"{b['id']},99,6,58,0,{val_str},0,0,0"
                self.send_custom_udp_command(b['id'], base_msg)
            except ValueError:
                pass

        ttk.Button(main_setup_frame, text="Send Speed PID", command=send_speed_pid).grid(row=9, column=0, columnspan=2, pady=(10, 5))
        
        ttk.Label(main_setup_frame, text="Speed Limits", font=("Arial", 11, "bold")).grid(row=10, column=0, columnspan=2, pady=(0, 10))
        
        ttk.Label(main_setup_frame, text="Max Speed:").grid(row=11, column=0, sticky="e", padx=10, pady=5)
        max_speed_entry = ttk.Entry(main_setup_frame, width=15)
        max_speed_entry.grid(row=11, column=1, sticky="w", pady=5)
        
        ttk.Label(main_setup_frame, text="Min Speed:").grid(row=12, column=0, sticky="e", padx=10, pady=5)
        min_speed_entry = ttk.Entry(main_setup_frame, width=15)
        min_speed_entry.grid(row=12, column=1, sticky="w", pady=5)

        ttk.Label(main_setup_frame, text="Pivot Speed:").grid(row=13, column=0, sticky="e", padx=10, pady=5)
        pivot_speed_entry = ttk.Entry(main_setup_frame, width=15)
        pivot_speed_entry.grid(row=13, column=1, sticky="w", pady=5)
        
        def send_speed_limits():
            try:
                max_s = int(max_speed_entry.get() or 0)
                min_s = int(min_speed_entry.get() or 0)
                piv_s = float(pivot_speed_entry.get() or 0.2)
                val_str = f"{max_s},{min_s},{format(piv_s, '.2f')}"
                base_msg = f"{b['id']},99,6,69,0,{val_str},0,0,0,0,0"
                self.send_custom_udp_command(b['id'], base_msg)
            except ValueError:
                pass

        ttk.Button(main_setup_frame, text="Send Speed Limits", command=send_speed_limits).grid(row=14, column=0, columnspan=2, pady=(10, 5))
        
        b['setup_entries'] = {
            "Kpr": kpr_entry, "Kir": kir_entry, "Kdr": kdr_entry,
            "Kps": kps_entry, "Kis": kis_entry, "Kds": kds_entry,
            "maxSpeed": max_speed_entry, "minSpeed": min_speed_entry,
            "pivotSpeed": pivot_speed_entry
        }
        # Pre-fill if we have data already
        for key, entry in b['setup_entries'].items():
            val = b['data'].get(key, "")
            if val:
                entry.delete(0, tk.END)
                entry.insert(0, val)

    def on_infield_calib_click(self, idx):
        b = self.buoy_frames[idx]
        if not b['id']: return
        
        response = messagebox.askyesno(
            "Confirm In-Field Compass Calibration",
            f"Deploying In-Field Compass Calibration will cause Buoy {b['id']} to move autonomously in circles for ~3 minutes.\n\nEnsure the area is clear. Continue?"
        )
        if response:
            # CMD=70, STATUS=0
            base_msg = f"{b['id']},99,0,70,0,0,0,0,0,0,0"
            self.send_custom_udp_command(b['id'], base_msg)
            self.log_message(f"Triggered In-Field Compass Calibration for Buoy {b['id']}")

    def on_infield_offset_click(self, idx):
        b = self.buoy_frames[idx]
        if not b['id']: return
        
        response = messagebox.askyesno(
            "Confirm In-Field Offset Calibration",
            f"Deploying In-Field Offset Calibration will cause Buoy {b['id']} to sail South for ~2 minutes, then return North.\n\nEnsure you have enough open water to the South. Continue?"
        )
        if response:
            # CMD=71, STATUS=0
            base_msg = f"{b['id']},99,0,71,0,0,0,0,0,0,0"
            self.send_custom_udp_command(b['id'], base_msg)
            self.log_message(f"Triggered In-Field Offset Calibration for Buoy {b['id']}")

    def send_custom_udp_command(self, target_id, base_msg, use_udp=True, use_lora=None):
        # Determine if UDP is allowed for this target
        udp_allowed = True
        target_buoy = None
        for b in self.buoy_frames:
            if b['id'] == target_id:
                target_buoy = b
                udp_allowed = b['udp_enabled_var'].get()
                break

        # If use_lora is None, treat LoRa as a backup: only use if UDP is inactive or disabled
        if use_lora is None:
            use_lora = True # Default to True
            if target_buoy:
                current_t = time.time()
                # If UDP is healthy AND enabled, we don't need LoRa backup
                if udp_allowed and target_buoy['last_udp_time'] > 0 and (current_t - target_buoy['last_udp_time'] < 5):
                    use_lora = False

        crc = 0
        for char in base_msg:
            crc ^= ord(char)
        full_msg = f"${base_msg}*{crc:02X}"
        
        if use_udp and udp_allowed:
            target_ip = "255.255.255.255"
            if target_buoy and target_buoy['data'].get("IP"):
                target_ip = target_buoy['data']["IP"]
            
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                    s.sendto(full_msg.encode(), (target_ip, 1001))
                self.log_message(f"SENT UDP TO {target_id} ({target_ip}): {full_msg}")
            except Exception as e:
                self.log_message(f"UDP SEND ERROR: {e}")
        elif use_udp and not udp_allowed:
            self.log_message(f"UDP SEND SKIPPED (DISABLED) - {target_id}")
            
        if use_lora:
            # Send via Serial (LoRa)
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write((full_msg + "\n").encode())
                    self.log_message(f"SENT LORA TO {target_id}: {full_msg}")
                except Exception as e:
                    self.log_message(f"LORA SEND ERROR: {e}")
            else:
                self.log_message(f"LORA BACKUP SKIPPED (NOT CONNECTED) - {target_id}")

    def send_udp_command(self, target_id, cmd_id, use_udp=True, use_lora=None):
        if target_id is None:
            target_id = "1"

        # Determine if UDP is allowed for this target
        udp_allowed = True
        target_buoy = None
        for b in self.buoy_frames:
            if b['id'] == target_id:
                target_buoy = b
                udp_allowed = b['udp_enabled_var'].get()
                break

        # If use_lora is None, treat LoRa as a backup: only use if UDP is inactive or disabled
        if use_lora is None:
            use_lora = True 
            if target_buoy:
                current_t = time.time()
                if udp_allowed and target_buoy['last_udp_time'] > 0 and (current_t - target_buoy['last_udp_time'] < 5):
                    use_lora = False
        
        base_msg = f"{target_id},99,3,{cmd_id},7"
        crc = 0
        for char in base_msg:
            crc ^= ord(char)
        full_msg = f"${base_msg}*{crc:02X}"
        
        if use_udp and udp_allowed:
            target_ip = "255.255.255.255"
            if target_buoy and target_buoy['data'].get("IP"):
                target_ip = target_buoy['data']["IP"]
            
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                    s.sendto(full_msg.encode(), (target_ip, 1001))
                self.log_message(f"SENT UDP TO {target_id} ({target_ip}): {full_msg}")
            except Exception as e:
                self.log_message(f"UDP SEND ERROR: {e}")
        elif use_udp and not udp_allowed:
            self.log_message(f"UDP SEND SKIPPED (DISABLED) - {target_id}")
            
        if use_lora:
            # Send via Serial (LoRa)
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write((full_msg + "\n").encode())
                    self.log_message(f"SENT LORA TO {target_id}: {full_msg}")
                except Exception as e:
                    self.log_message(f"LORA SEND ERROR: {e}")
            else:
                self.log_message(f"LORA BACKUP SKIPPED (NOT CONNECTED) - {target_id}")

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

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
                self.serial_conn = serial.Serial()
                self.serial_conn.port = port
                self.serial_conn.baudrate = 115200
                self.serial_conn.timeout = 1
                # Prevent ESP32 auto-reset by disabling DTR and RTS
                self.serial_conn.dtr = False
                self.serial_conn.rts = False
                self.serial_conn.open()
                
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
                        self.log_message(f"LORA IN: {line}")
                        # We treat LoRa incoming the same as UDP incoming
                        # Fix late-binding in lambda by explicitly passing line as a default argument
                        self.master.after(0, lambda msg=line: self.parse_message(msg, "LoRa"))
                except Exception as e:
                    print(f"Serial Read Error: {e}")
                    break
            time.sleep(0.01)

    def log_message(self, message):
        """Thread-safe logging of raw messages to the text widget"""
        def _log():
            timestamp = datetime.now().strftime("%H:%M:%S")
            log_entry = f"[{timestamp}] {message}\n"
            
            self.log_text.insert("end", log_entry)
            self.log_text.see("end")
            
            if float(self.log_text.index("end-1c")) > 50:
                self.log_text.delete("1.0", "2.0")
        
        self.master.after(0, _log)

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
            "Timestamp",
            "Target Dir (tgDir)", "Magnetic Dir (mDir)",
            "Wind Dir (wDir)", "Wind StdDev (wStd)",
            "PID I-term (IP)", "PID R-term (IR)",
            "Sub Battery V",
            "GPS Fix", "GPS Satellites (GpsSat)"
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
                self.parse_message(message, addr[0])
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

    def parse_message(self, message, sender_ip):
        current_timestamp = datetime.now().strftime("%H:%M:%S")
        
        if not message.startswith('$') or '*' not in message:
            return
            
        try:
            content, crc = message[1:].split('*', 1)
            fields = content.split(',')
            
            if len(fields) < 5:
                return
                
            cmd = fields[3]
            buoy_id = fields[1]
            data = {"Timestamp": current_timestamp, "IP": sender_ip}
            
            # --- Network Monitor and Filtering Logic ---
            for b in self.buoy_frames:
                if b['id'] == buoy_id or b['id'] is None:
                    # If this is UDP data but UDP is disabled for this buoy, ignore it
                    if sender_ip != "LoRa" and not b['udp_enabled_var'].get():
                        return
                        
                    if sender_ip == "LoRa":
                        b['last_lora_time'] = time.time()
                        b['last_lora_content'] = content
                    else:
                        b['last_udp_time'] = time.time()
                        b['last_udp_content'] = content
                    break
            # --------------------------------------------
            
            if cmd == "51" and len(fields) >= 21: # TOPDATA
                data.update({
                    "IDr": fields[0], "IDs": fields[1], "ACK": fields[2], "CMD": fields[3], "Status": fields[4],
                    "Magnetic Dir (mDir)": fields[5], "GPS Dir (gpsDir)": fields[6],
                    "Target Dir (tgDir)": fields[7], "Target Dist (tgDist)": fields[8],
                    "Wind Dir (wDir)": fields[9], "Wind StdDev (wStd)": fields[10],
                    "Bow Thruster (BB)": fields[11], "Stern Thruster (SB)": fields[12],
                    "PID I-term (IP)": fields[13], "PID R-term (IR)": fields[14],
                    "Sub Battery V": fields[15], "Sub Battery %": fields[16],
                    "Latitude (Lat)": fields[17], "Longitude (Lon)": fields[18],
                    "GPS Fix": fields[19], "GPS Satellites (GpsSat)": fields[20]
                })
                self.update_buoy_data(buoy_id, data)
                
            elif cmd == "47" and len(fields) >= 7: # DIRDIST
                data.update({
                    "IDr": fields[0], "IDs": fields[1], "ACK": fields[2], "CMD": fields[3], "Status": fields[4],
                    "Target Dir (tgDir)": fields[5], "Target Dist (tgDist)": fields[6]
                })
                self.update_buoy_data(buoy_id, data)
                
            elif cmd == "7" and len(fields) >= 5: # IDLE
                data.update({
                    "IDr": fields[0], "IDs": fields[1], "ACK": fields[2], "CMD": fields[3], "Status": fields[4],
                    "Bow Thruster (BB)": "0", "Stern Thruster (SB)": "0"
                })
                self.update_buoy_data(buoy_id, data)
                
            elif cmd == "19" and len(fields) >= 14: # BUOYPOS
                data.update({
                    "IDr": fields[0], "IDs": fields[1], "ACK": fields[2], "CMD": fields[3], "Status": fields[4],
                    "Latitude (Lat)": fields[5], "Longitude (Lon)": fields[6],
                    "Magnetic Dir (mDir)": fields[7], "Wind Dir (wDir)": fields[8], "Wind StdDev (wStd)": fields[9],
                    "Bow Thruster (BB)": fields[10], "Stern Thruster (SB)": fields[11],
                    "GPS Fix": fields[12], "GPS Satellites (GpsSat)": fields[13]
                })
                self.update_buoy_data(buoy_id, data)
                
            elif cmd == "55" and len(fields) >= 8: # PIDRUDDER
                data.update({
                    "Kpr": fields[5], "Kir": fields[6], "Kdr": fields[7]
                })
                self.update_buoy_data(buoy_id, data)
                
            elif cmd == "57" and len(fields) >= 8: # PIDSPEED
                data.update({
                    "Kps": fields[5], "Kis": fields[6], "Kds": fields[7]
                })
                self.update_buoy_data(buoy_id, data)
                
            elif cmd == "68" and len(fields) >= 7: # MAXMINPWR
                piv_s = "0.2"
                if len(fields) >= 8:
                    piv_s = fields[7]
                data.update({
                    "maxSpeed": fields[5], "minSpeed": fields[6], "pivotSpeed": piv_s
                })
                self.update_buoy_data(buoy_id, data)
                
        except Exception as e:
            print(f"Parse error: {e}")

    def update_gui(self):
        current_t = time.time()
        for b in self.buoy_frames:
            data = b['data']
            current_status = str(data.get("Status", "0")).strip()
            
            # --- Network Monitor Update ---
            if b['last_udp_time'] > 0 and (current_t - b['last_udp_time'] < 5):
                b['udp_indicator'].config(text="UDP: OK", fg="green")
            else:
                b['udp_indicator'].config(text="UDP: --", fg="gray")
                
            if b['last_lora_time'] > 0 and (current_t - b['last_lora_time'] < 5):
                b['lora_indicator'].config(text="LoRa: OK", fg="green")
            else:
                b['lora_indicator'].config(text="LoRa: --", fg="gray")
                
            if b['last_udp_content'] and b['last_lora_content']:
                if b['last_udp_content'] == b['last_lora_content']:
                    b['sync_indicator'].config(text="Data: SYNC", fg="green")
                else:
                    b['sync_indicator'].config(text="Data: DIFF", fg="red")
            else:
                b['sync_indicator'].config(text="Data: WAIT", fg="gray")
            # ------------------------------
            
            # Update Canvas Texts (Always show, even if waiting for ID)
            if current_status == "7": # IDLE
                b['windrose_canvas'].itemconfig(b['dist_text'], text="-")
                b['windrose_canvas'].itemconfig(b['pid_i_text'], text="*")
            else:
                dist_val = data.get("Target Dist (tgDist)", "N/A")
                if dist_val != "N/A" and dist_val != "nan" and dist_val != "":
                    try:
                        dist_val = f"{float(dist_val):.2f}m"
                    except ValueError: 
                        dist_val = "0.00m"
                else:
                    dist_val = "0.00m"
                b['windrose_canvas'].itemconfig(b['dist_text'], text=dist_val)
                
                pid_i_val = data.get("PID I-term (IP)", "N/A")
                if pid_i_val != "N/A" and pid_i_val != "nan" and pid_i_val != "":
                    try:
                        pid_i_val = f"{float(pid_i_val):.2f}"
                    except ValueError:
                        pid_i_val = "0.00"
                else:
                    pid_i_val = "0.00"
                b['windrose_canvas'].itemconfig(b['pid_i_text'], text=f"I:{pid_i_val}")
            
            # Ensure text stays on top of arrows
            b['windrose_canvas'].tag_raise(b['dist_text'])
            b['windrose_canvas'].tag_raise(b['pid_i_text'])

            if b['id'] is not None:
                labels = b['labels']
                # Update Lock Button Text (Action-oriented)
                
                status_text = "UNKNOWN"
                if current_status == "7":
                    status_text = "IDLE"
                elif current_status in ["12", "13", "14"]:
                    status_text = "LOCKED"
                elif current_status in ["15", "16", "17"]:
                    status_text = "DOCKING"
                elif current_status == "49":
                    status_text = "SETUP"
                else:
                    status_text = f"STATUS {current_status}"
                b['status_label'].config(text=status_text)
                
                # Update IP Header Label
                b['ip_header_label'].config(text=data.get("IP", ""))
                
                if current_status in ["12", "13", "14"]: # LOCKED
                    b['lock_btn'].config(text="IDLE")
                elif current_status == "7": # IDLE
                    b['lock_btn'].config(text="LOCK")
                
                # Update Dock Button Text
                if current_status in ["15", "16", "17"]: # DOCKING
                    b['dock_btn'].config(text="IDLE")
                else:
                    b['dock_btn'].config(text="DOCK")
                
                # Update DIRDIST Send Button state based on GPS Fix
                gps_fix = str(data.get("GPS Fix", "0")).strip()
                if gps_fix == "1" or gps_fix.lower() == "true":
                    b['dirdist_send_btn'].config(state="normal")
                else:
                    b['dirdist_send_btn'].config(state="disabled")
                
                tg_dir = data.get("Target Dir (tgDir)", "N/A")
                w_dir = data.get("Wind Dir (wDir)", "N/A")
                if current_status == "7": # Hide in IDLE
                    tg_dir = "N/A"
                    w_dir = "N/A"
                
                gps_dir = data.get("GPS Dir (gpsDir)", "N/A")
                if gps_dir not in ["N/A", "nan", ""]:
                    if b.get("gps_dir_first_seen") is None:
                        b["gps_dir_first_seen"] = time.time()
                    
                    if time.time() - b["gps_dir_first_seen"] >= 5.0:
                        gps_dir_to_show = gps_dir
                    else:
                        gps_dir_to_show = "N/A"
                else:
                    b["gps_dir_first_seen"] = None
                    gps_dir_to_show = "N/A"
                
                self.update_windrose(b, w_dir, tg_dir, data.get("Magnetic Dir (mDir)", "N/A"), gps_dir_to_show)
                
                # Update Setup Window entries if open
                if b.get('setup_entries'):
                    for key, entry in b['setup_entries'].items():
                        val = data.get(key, "")
                        # Only update if the entry is empty (first time)
                        if val and not entry.get():
                            entry.insert(0, val)

                # --- Update visual indicators independently of the labels list ---
                for thrust_name in ["Bow Thruster (BB)", "Stern Thruster (SB)"]:
                    val = data.get(thrust_name, "N/A")
                    if current_status == "7":
                        val = "0"
                        
                    canvas = b['bb_bar'] if thrust_name == "Bow Thruster (BB)" else b['sb_bar']
                    bar_label = b['bb_val_label'] if thrust_name == "Bow Thruster (BB)" else b['sb_val_label']
                    
                    canvas.delete("bar")
                    if val != "N/A":
                        try:
                            v = float(val)
                            bar_height = abs(v) * 0.9
                            if v >= 0:
                                canvas.create_rectangle(0, 90 - bar_height, 20, 90, fill="green", outline="green", tags="bar")
                            else:
                                canvas.create_rectangle(0, 90, 20, 90 + bar_height, fill="red", outline="red", tags="bar")
                            bar_label.config(text=f"{val}%")
                        except ValueError: pass
                    else:
                        bar_label.config(text="N/A")

                volt_val = data.get("Sub Battery V", "N/A")
                if volt_val != "N/A":
                    try:
                        v = float(volt_val)
                        b['volt_bar']['value'] = max(0, min(8.2, v - 17.0))
                    except ValueError: pass
                # ---------------------------------------------------------------

                for name, label_widget in labels.items():
                    if name == "Sub Battery V":
                        label_widget.config(text=volt_val)
                    elif name in ["Target Dir (tgDir)", "Magnetic Dir (mDir)", "GPS Dir (gpsDir)", "Wind Dir (wDir)"]:
                        val = data.get(name, "N/A")
                        if val != "N/A":
                            try:
                                val = f"{float(val):.0f}"
                            except ValueError: pass
                        label_widget.config(text=val)
                    else:
                        label_widget.config(text=data.get(name, "N/A"))

        self.master.after(500, self.update_gui)

    def update_windrose(self, b, w_dir_str, tg_dir_str, m_dir_str, gps_dir_str="N/A"):
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
        draw_arrow(tg_dir_str, 'tg_arrow', 'red', 80, 4)
        draw_arrow(m_dir_str, 'mag_arrow', 'green', 70, 3)
        draw_arrow(w_dir_str, 'wind_arrow', 'blue', 60, 2)
        draw_arrow(gps_dir_str, 'gps_arrow', 'black', 50, 3)

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
