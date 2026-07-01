#!/usr/bin/env python3
"""
Over-The-Air (OTA) Device Discovery Radar
==========================================
A highly visual, real-time desktop dashboard to discover OTA-capable microcontrollers
(ESP32, ESP8266, Arduino) on the local network using mDNS (Multicast DNS).

Features:
  - Interactive radar-like sweep visualization on a Tkinter Canvas.
  - Displays discovered devices as glowing blips that ripple upon radar sweeps.
  - Multi-threaded non-blocking service discovery (Zeroconf with pure-socket UDP fallback).
  - Copy PlatformIO upload command directly to clipboard with a single click.
  - In-app device connectivity verification (Ping / TCP socket check).
  - Modern, dark-themed developer-centric user interface.
"""

import os
import sys
import math
import time
import socket
import struct
import threading
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox

# --- ENVIRONMENT & DEPENDENCY CHECK ---
ZEROCONF_AVAILABLE = False
try:
    import zeroconf
    ZEROCONF_AVAILABLE = True
except ImportError:
    print("[Discovery] 'zeroconf' library not found. Attempting automatic installation...")
    try:
        # Run pip install silently
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", "--quiet", "zeroconf"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        import zeroconf
        ZEROCONF_AVAILABLE = True
        print("[Discovery] Successfully installed and imported 'zeroconf'!")
    except Exception as e:
        print(f"[Discovery] Could not auto-install 'zeroconf': {e}")
        print("[Discovery] Falling back to pure Python UDP mDNS discovery mode.")


class OTADeviceListener:
    """Listener class for Zeroconf service discovery."""
    def __init__(self, on_device_found, on_device_removed):
        self.on_device_found = on_device_found
        self.on_device_removed = on_device_removed

    def add_service(self, zc_obj, service_type, name):
        # Resolve service details in a separate thread to prevent blocking
        threading.Thread(
            target=self._resolve_and_report,
            args=(zc_obj, service_type, name),
            daemon=True
        ).start()

    def remove_service(self, zc_obj, service_type, name):
        self.on_device_removed(name)

    def update_service(self, zc_obj, service_type, name):
        threading.Thread(
            target=self._resolve_and_report,
            args=(zc_obj, service_type, name),
            daemon=True
        ).start()

    def _resolve_and_report(self, zc_obj, service_type, name):
        try:
            info = zc_obj.get_service_info(service_type, name, timeout=3000)
            if info:
                # Standardize IP addresses
                ips = [socket.inet_ntoa(addr) for addr in info.addresses]
                if ips:
                    ip = ips[0]
                    port = info.port
                    server = info.server.rstrip(".") if info.server else f"{name}.local"
                    
                    # Parse TXT record properties
                    props = {}
                    if info.properties:
                        for k, v in info.properties.items():
                            key = k.decode('utf-8', errors='ignore') if isinstance(k, bytes) else str(k)
                            val = v.decode('utf-8', errors='ignore') if isinstance(v, bytes) else str(v)
                            props[key] = val
                    
                    self.on_device_found(name, ip, port, server, props)
        except Exception as e:
            print(f"[Discovery] Error resolving service info for {name}: {e}")


class OTARadarApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Over-The-Air (OTA) Device Discovery Radar")
        self.root.geometry("1050x680")
        self.root.configure(bg="#0c0f12")
        self.root.resizable(True, True)

        # Threading/Control States
        self.running = True
        self.devices = {}
        self.ripples = []
        self.sweep_angle = 0.0
        self.selected_device_key = None
        self.fallback_stop_event = threading.Event()
        
        # UI Styling Setup
        self.setup_styles()
        
        # Build Interface
        self.create_widgets()
        
        # Bind events
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<Motion>", self.on_canvas_hover)

        # Start Services Discovery
        self.start_discovery()
        
        # Start Radar Animation loop
        self.tick()

    def setup_styles(self):
        """Sets up custom styles for a modern, dark developer interface."""
        self.style = ttk.Style()
        # Use 'clam' as a base theme to allow extensive color overrides
        self.style.theme_use('clam')
        
        # Main Frame styling
        self.style.configure(".", background="#0c0f12", foreground="#c9d1d9")
        
        # LabelFrame styling
        self.style.configure("TLabelframe", background="#161b22", borderwidth=1, relief="solid")
        self.style.configure("TLabelframe.Label", background="#161b22", foreground="#58a6ff", font=("Arial", 10, "bold"))
        
        # Custom Button styling (Modern dark mode)
        self.style.configure("TButton", background="#21262d", foreground="#ffffff", borderwidth=1, relief="flat", font=("Arial", 9, "bold"))
        self.style.map("TButton",
            background=[("active", "#30363d"), ("pressed", "#58a6ff")],
            foreground=[("active", "#58a6ff"), ("pressed", "#ffffff")]
        )
        
        # Treeview Scroll styling & look
        self.style.configure("Treeview", 
            background="#161b22", 
            fieldbackground="#161b22", 
            foreground="#c9d1d9", 
            borderwidth=0, 
            gridcolor="#21262d",
            rowheight=25,
            font=("Arial", 9)
        )
        self.style.configure("Treeview.Heading", 
            background="#21262d", 
            foreground="#58a6ff", 
            borderwidth=1, 
            relief="solid", 
            font=("Arial", 9, "bold")
        )
        self.style.map("Treeview", 
            background=[("selected", "#0f355c")], 
            foreground=[("selected", "#58a6ff")]
        )

    def create_widgets(self):
        """Builds the visual interface split into Header, Radar, and Details panel."""
        # 1. Header Frame
        header = tk.Frame(self.root, bg="#0c0f12", padx=15, pady=10)
        header.pack(fill="x", side="top")
        
        title_label = tk.Label(
            header, 
            text="🌐 OVER-THE-AIR (OTA) DEVICE RADAR", 
            fg="#00ffcc", 
            bg="#0c0f12", 
            font=("Consolas", 18, "bold")
        )
        title_label.pack(anchor="w")
        
        subtitle_label = tk.Label(
            header, 
            text="Real-time network scanner discovering ESP32, ESP8266, and Arduino OTA nodes in your local subnet.", 
            fg="#8b949e", 
            bg="#0c0f12", 
            font=("Arial", 9, "italic")
        )
        subtitle_label.pack(anchor="w")

        # Main horizontal container
        main_container = tk.Frame(self.root, bg="#0c0f12")
        main_container.pack(fill="both", expand=True, padx=15, pady=5)

        # LEFT COLUMN - Radar Canvas
        left_column = tk.Frame(main_container, bg="#0c0f12")
        left_column.pack(side="left", fill="both", expand=False, padx=(0, 10))

        # Canvas Border Frame (for a neat neon edge)
        canvas_border = tk.Frame(left_column, bg="#1b4d3e", bd=1)
        canvas_border.pack(side="top")

        # Radar Canvas
        self.canvas = tk.Canvas(
            canvas_border, 
            width=400, 
            height=400, 
            bg="#0b1011", 
            highlightthickness=0, 
            cursor="crosshair"
        )
        self.canvas.pack()

        # Draw static radar items (background rings and cardinal directions)
        self.draw_radar_background()

        # Canvas Info Banner
        canvas_info = tk.Frame(left_column, bg="#161b22", bd=1, relief="solid", padx=10, pady=8)
        canvas_info.pack(fill="x", pady=(10, 0))
        
        self.lbl_coordinates = tk.Label(
            canvas_info, 
            text="Radar Coordinates: - , -", 
            fg="#8b949e", 
            bg="#161b22", 
            font=("Consolas", 9)
        )
        self.lbl_coordinates.pack(anchor="w")
        
        self.lbl_total_devices = tk.Label(
            canvas_info, 
            text="Detected Nodes: 0", 
            fg="#00ffcc", 
            bg="#161b22", 
            font=("Consolas", 10, "bold")
        )
        self.lbl_total_devices.pack(anchor="w", pady=(3, 0))

        # RIGHT COLUMN - Discovery List & Inspector
        right_column = tk.Frame(main_container, bg="#0c0f12")
        right_column.pack(side="right", fill="both", expand=True)

        # Top Right - Device List
        list_frame = ttk.LabelFrame(right_column, text=" ACTIVE OTA DEVICES ")
        list_frame.pack(fill="both", expand=True, pady=(0, 10))

        # Treeview for Listing Nodes
        self.tree = ttk.Treeview(
            list_frame, 
            columns=("name", "ip", "port", "board"), 
            show="headings", 
            selectmode="browse"
        )
        self.tree.heading("name", text="Device Name")
        self.tree.heading("ip", text="IP Address")
        self.tree.heading("port", text="OTA Port")
        self.tree.heading("board", text="Board/Hardware")

        self.tree.column("name", width=180, anchor="w")
        self.tree.column("ip", width=110, anchor="center")
        self.tree.column("port", width=70, anchor="center")
        self.tree.column("board", width=110, anchor="w")

        self.tree.pack(side="left", fill="both", expand=True, padx=(5, 0), pady=5)
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

        # Treeview Scrollbar
        scroll = ttk.Scrollbar(list_frame, orient="vertical", command=self.tree.yview)
        scroll.pack(side="right", fill="y", padx=(0, 5), pady=5)
        self.tree.configure(yscrollcommand=scroll.set)

        # Bottom Right - Device Inspector Panel
        self.inspector_frame = ttk.LabelFrame(right_column, text=" NODE INSPECTOR ")
        self.inspector_frame.pack(fill="x", expand=False)

        inspect_container = tk.Frame(self.inspector_frame, bg="#161b22", padx=10, pady=10)
        inspect_container.pack(fill="x")

        # Metadata labels (grid layout)
        tk.Label(inspect_container, text="Device Name:", fg="#58a6ff", bg="#161b22", font=("Arial", 9, "bold")).grid(row=0, column=0, sticky="w", pady=2)
        self.lbl_inspect_name = tk.Label(inspect_container, text="-", fg="#c9d1d9", bg="#161b22", font=("Consolas", 9))
        self.lbl_inspect_name.grid(row=0, column=1, sticky="w", padx=10, pady=2)

        tk.Label(inspect_container, text="IP Address:", fg="#58a6ff", bg="#161b22", font=("Arial", 9, "bold")).grid(row=1, column=0, sticky="w", pady=2)
        self.lbl_inspect_ip = tk.Label(inspect_container, text="-", fg="#c9d1d9", bg="#161b22", font=("Consolas", 9, "bold"))
        self.lbl_inspect_ip.grid(row=1, column=1, sticky="w", padx=10, pady=2)

        tk.Label(inspect_container, text="OTA Port:", fg="#58a6ff", bg="#161b22", font=("Arial", 9, "bold")).grid(row=2, column=0, sticky="w", pady=2)
        self.lbl_inspect_port = tk.Label(inspect_container, text="-", fg="#c9d1d9", bg="#161b22", font=("Consolas", 9))
        self.lbl_inspect_port.grid(row=2, column=1, sticky="w", padx=10, pady=2)

        tk.Label(inspect_container, text="MDNS Hostname:", fg="#58a6ff", bg="#161b22", font=("Arial", 9, "bold")).grid(row=3, column=0, sticky="w", pady=2)
        self.lbl_inspect_server = tk.Label(inspect_container, text="-", fg="#c9d1d9", bg="#161b22", font=("Consolas", 9))
        self.lbl_inspect_server.grid(row=3, column=1, sticky="w", padx=10, pady=2)

        tk.Label(inspect_container, text="Properties / TXT:", fg="#58a6ff", bg="#161b22", font=("Arial", 9, "bold")).grid(row=4, column=0, sticky="nw", pady=2)
        self.lbl_inspect_props = tk.Label(inspect_container, text="-", fg="#8b949e", bg="#161b22", font=("Consolas", 8), justify="left", wraplength=400)
        self.lbl_inspect_props.grid(row=4, column=1, sticky="w", padx=10, pady=2)

        # Action Buttons for Node
        btn_frame = tk.Frame(inspect_container, bg="#161b22", pady=8)
        btn_frame.grid(row=5, column=0, columnspan=2, sticky="w")

        self.btn_copy_ip = ttk.Button(btn_frame, text="📋 Copy IP", width=12, command=self.action_copy_ip, state="disabled")
        self.btn_copy_ip.pack(side="left", padx=5)

        self.btn_copy_pio = ttk.Button(btn_frame, text="🚀 Copy PlatformIO Cmd", width=25, command=self.action_copy_pio, state="disabled")
        self.btn_copy_pio.pack(side="left", padx=5)

        self.btn_ping = ttk.Button(btn_frame, text="⚡ Ping Node", width=13, command=self.action_ping_device, state="disabled")
        self.btn_ping.pack(side="left", padx=5)

        self.lbl_ping_status = tk.Label(btn_frame, text="", fg="#8b949e", bg="#161b22", font=("Arial", 9, "bold"))
        self.lbl_ping_status.pack(side="left", padx=10)

        # BOTTOM BAR - Connection & Status indicator
        bottom_bar = tk.Frame(self.root, bg="#161b22", bd=1, relief="ridge", height=24)
        bottom_bar.pack(fill="x", side="bottom")

        # Discovery Mode Status Display
        if ZEROCONF_AVAILABLE:
            mode_text = "MODE: Active Multicast-DNS Service Browser (Zeroconf)"
            mode_color = "#58a6ff"
        else:
            mode_text = "MODE: Fallback UDP Multicast listener (Zeroconf unavailable)"
            mode_color = "#ff9800"

        self.lbl_mode = tk.Label(bottom_bar, text=mode_text, fg=mode_color, bg="#161b22", font=("Consolas", 9, "bold"))
        self.lbl_mode.pack(side="left", padx=15)

        self.lbl_network_state = tk.Label(bottom_bar, text="🟢 Scanner Listening", fg="#00ffcc", bg="#161b22", font=("Consolas", 9))
        self.lbl_network_state.pack(side="right", padx=15)

    # --- RADAR RENDERING ENGINE ---
    def draw_radar_background(self):
        """Draws the static radar scale, rings, and directions (N, S, E, W)."""
        xc, yc = 200, 200
        
        # Outer compass boundary
        self.canvas.create_oval(
            xc - 180, yc - 180, xc + 180, yc + 180, 
            outline="#1b4d3e", width=2, tags="static"
        )
        
        # Inner range marker rings (50px, 100px, 150px)
        for r in [50, 100, 150]:
            self.canvas.create_oval(
                xc - r, yc - r, xc + r, yc + r, 
                outline="#103d30", width=1, dash=(3, 5), tags="static"
            )
            # Circle labels
            self.canvas.create_text(
                xc + 3, yc - r - 6, 
                text=f"{r}m", fill="#103d30", 
                font=("Consolas", 7), anchor="w", tags="static"
            )
            
        # Crosshair dividing grids
        self.canvas.create_line(xc - 180, yc, xc + 180, yc, fill="#103d30", dash=(2, 4), tags="static")
        self.canvas.create_line(xc, yc - 180, xc, yc + 180, fill="#103d30", dash=(2, 4), tags="static")
        
        # Cardinal direction text
        self.canvas.create_text(xc, yc - 192, text="N", fill="#00ffcc", font=("Courier", 11, "bold"), tags="static")
        self.canvas.create_text(xc + 192, yc, text="E", fill="#00ffcc", font=("Courier", 11, "bold"), tags="static")
        self.canvas.create_text(xc, yc + 192, text="S", fill="#00ffcc", font=("Courier", 11, "bold"), tags="static")
        self.canvas.create_text(xc - 192, yc, text="W", fill="#00ffcc", font=("Courier", 11, "bold"), tags="static")
        
        # Compass angles ticks and numbers (every 30 degrees)
        for deg in range(0, 360, 30):
            # Convert degrees to radians and offset by -90 to start at North
            rad = math.radians(deg - 90)
            x1 = xc + 180 * math.cos(rad)
            y1 = yc + 180 * math.sin(rad)
            x2 = xc + 174 * math.cos(rad)
            y2 = yc + 174 * math.sin(rad)
            self.canvas.create_line(x1, y1, x2, y2, fill="#1b4d3e", width=1, tags="static")
            
            # Compass numbers
            tx = xc + 162 * math.cos(rad)
            ty = yc + 162 * math.sin(rad)
            if deg in [30, 60, 120, 150, 210, 240, 300, 330]:
                self.canvas.create_text(
                    tx, ty, text=str(deg), 
                    fill="#155c47", font=("Consolas", 8), tags="static"
                )

    def draw_radar_dynamic(self):
        """Handles drawing of all non-static animated items (sweep lines, blips, ripples)."""
        self.canvas.delete("dynamic")
        xc, yc = 200, 200
        now = time.time()
        
        # 1. Draw Phosphor Sweep Trail (multi-line gradient)
        trail_segments = 24
        for i in range(trail_segments):
            angle = (self.sweep_angle - i * 0.8) % 360
            rad = math.radians(angle - 90)
            
            # Calculate gradient fade factor
            ratio = i / trail_segments
            r_val = 0
            g_val = int(255 * (1 - ratio) + 12 * ratio)
            b_val = int(204 * (1 - ratio) + 10 * ratio)
            color_hex = f"#{r_val:02x}{g_val:02x}{b_val:02x}"
            
            x2 = xc + 180 * math.cos(rad)
            y2 = yc + 180 * math.sin(rad)
            
            # Main sweeping beam is thicker
            width = 2 if i == 0 else 1
            self.canvas.create_line(xc, yc, x2, y2, fill=color_hex, width=width, tags="dynamic")

        # 2. Draw Sonar Expanding Ripples
        remaining_ripples = []
        for rpl in self.ripples:
            elapsed = now - rpl['start_time']
            if elapsed < rpl['duration']:
                ratio = elapsed / rpl['duration']
                # Circular diameter expansion
                current_r = rpl['start_r'] + ratio * (rpl['max_r'] - rpl['start_r'])
                
                # Fade color out (grows darker green/cyan)
                gr = int(255 * (1 - ratio) + 10 * ratio)
                gb = int(255 * (1 - ratio) + 16 * ratio)
                ripple_color = f"#00{gr:02x}{gb:02x}"
                
                x = rpl['x']
                y = rpl['y']
                self.canvas.create_oval(
                    x - current_r, y - current_r, x + current_r, y + current_r, 
                    outline=ripple_color, width=1, tags="dynamic"
                )
                remaining_ripples.append(rpl)
        self.ripples = remaining_ripples

        # 3. Draw Discovered Devices (Blips)
        for key, dev in self.devices.items():
            angle_deg = dev['angle']
            r = dev['radius']
            
            # Convert polar coordinates (angle, radius) to Cartesian canvas (x, y)
            rad = math.radians(angle_deg - 90)
            bx = xc + r * math.cos(rad)
            by = yc + r * math.sin(rad)
            
            # Check for radar sweep line intersection
            diff = (self.sweep_angle - angle_deg) % 360
            # If the sweep beam passes the device, trigger glowing blip and ripple
            if diff < 2.0 and (now - dev['last_sweep_hit'] > 2.0):
                dev['last_sweep_hit'] = now
                self.ripples.append({
                    'x': bx, 'y': by,
                    'start_r': 4.0, 'max_r': 36.0,
                    'start_time': now, 'duration': 0.8
                })
            
            # Adjust blip visual intensity based on time elapsed since hit
            time_since_hit = now - dev['last_sweep_hit']
            if time_since_hit < 1.5:
                # Intensely glowing white/cyan blip
                ratio = time_since_hit / 1.5
                gr = int(255 * (1 - ratio) + 10 * ratio)
                gb = int(255 * (1 - ratio) + 255 * ratio)
                blip_color = f"#{gr:02x}ff{gb:02x}"
                size = 6
            elif time_since_hit < 4.0:
                # Decaying phosphor amber-green blip
                ratio = (time_since_hit - 1.5) / 2.5
                g = int(255 * (1 - ratio) + 110 * ratio)
                blip_color = f"#00{g:02x}88"
                size = 5
            else:
                # Dim, resting standby green blip
                blip_color = "#005a3c"
                size = 4
                
            # Draw node circle
            # Draw highlighted circle if it is currently selected
            is_selected = (self.selected_device_key == key)
            if is_selected:
                self.canvas.create_oval(
                    bx - (size + 4), by - (size + 4), 
                    bx + (size + 4), by + (size + 4), 
                    outline="#00ffcc", width=1.5, tags="dynamic"
                )
                
            self.canvas.create_oval(
                bx - size, by - size, bx + size, by + size, 
                fill=blip_color, outline="#00ffcc", width=1, tags="dynamic"
            )
            
            # Draw text label next to blip
            lbl_color = "#88ccaa" if time_since_hit < 3.0 else "#447e5e"
            if is_selected:
                lbl_color = "#00ffcc"
            
            display_name = dev['name'].split('.')[0]
            if len(display_name) > 14:
                display_name = display_name[:12] + ".."
                
            self.canvas.create_text(
                bx + 10, by, 
                text=display_name, anchor="w", 
                fill=lbl_color, font=("Consolas", 8, "bold" if is_selected else "normal"), 
                tags="dynamic"
            )

    def tick(self):
        """Periodic animation tick representing ~60 FPS update frequency."""
        if not self.running:
            return
            
        # Rotate radar sweep beam (degree increments)
        self.sweep_angle = (self.sweep_angle + 1.8) % 360
        
        # Redraw changing parts
        self.draw_radar_dynamic()
        
        # Keep animation rolling
        self.root.after(16, self.tick)

    # --- COORDINATE MAPPING & STABILITY ---
    def calculate_position(self, name, ip):
        """
        Calculates a deterministic polar location for a node on the radar using hashing,
        guaranteeing stable non-overlapping placements for identical devices.
        """
        # Form stable integer hash
        hash_val = 0
        for char in (name + ip):
            hash_val = (hash_val * 31 + ord(char)) & 0xffffffff
            
        # Angle in degrees (0 to 359)
        angle = hash_val % 360
        
        # Radius mapped inside 50px to 170px to avoid center & boundary overlap
        radius = 50 + ((hash_val >> 8) % 115)
        
        return angle, radius

    # --- INTERACTION & EVENT HANDLERS ---
    def on_canvas_click(self, event):
        """Permits clicking directly on a radar blip to inspect the node."""
        xc, yc = 200, 200
        click_x, click_y = event.x, event.y
        
        closest_key = None
        min_dist = 16.0  # Click bounding box threshold
        
        for key, dev in self.devices.items():
            angle_deg = dev['angle']
            r = dev['radius']
            rad = math.radians(angle_deg - 90)
            bx = xc + r * math.cos(rad)
            by = yc + r * math.sin(rad)
            
            dist = math.sqrt((click_x - bx)**2 + (click_y - by)**2)
            if dist < min_dist:
                min_dist = dist
                closest_key = key
                
        if closest_key:
            self.select_device_by_key(closest_key)

    def on_canvas_hover(self, event):
        """Dynamically displays cursor coordinates on the radar widget."""
        xc, yc = 200, 200
        dx = event.x - xc
        dy = event.y - yc
        
        # Calculate distance and compass angle from center
        dist = int(math.sqrt(dx**2 + dy**2))
        angle_rad = math.atan2(dy, dx)
        angle_deg = int((math.degrees(angle_rad) + 90) % 360)
        
        if dist <= 180:
            self.lbl_coordinates.config(text=f"Radar Coordinates: Range={dist:3d}m, Angle={angle_deg:3d}°")
        else:
            self.lbl_coordinates.config(text="Radar Coordinates: Out-of-Range")

    def on_tree_select(self, event):
        """Synchronizes selected Treeview rows with the inspector and radar target selection."""
        selection = self.tree.selection()
        if not selection:
            return
            
        item_id = selection[0]
        vals = self.tree.item(item_id, "values")
        if not vals:
            return
            
        # Match by name and IP to select
        name, ip = vals[0], vals[1]
        for key, dev in self.devices.items():
            if dev['name'] == name and dev['ip'] == ip:
                self.selected_device_key = key
                self.update_inspector_details(dev)
                break

    def select_device_by_key(self, key):
        """Highlights a device in both the spreadsheet list and inspector."""
        self.selected_device_key = key
        dev = self.devices[key]
        
        # Find item in Treeview and highlight
        for item_id in self.tree.get_children():
            vals = self.tree.item(item_id, "values")
            if vals and (vals[0] == dev['name'] and vals[1] == dev['ip']):
                self.tree.selection_set(item_id)
                self.tree.see(item_id)
                break
                
        self.update_inspector_details(dev)

    def update_inspector_details(self, dev):
        """Renders information into the Node Inspector GUI frame."""
        self.lbl_inspect_name.config(text=dev['name'])
        self.lbl_inspect_ip.config(text=dev['ip'])
        self.lbl_inspect_port.config(text=str(dev['port']))
        self.lbl_inspect_server.config(text=dev['server'])
        
        # Format TXT Properties nicely
        props_str = ""
        if dev['properties']:
            props_str = ", ".join([f"{k}={v}" for k, v in dev['properties'].items()])
        else:
            props_str = "None"
        self.lbl_inspect_props.config(text=props_str)
        
        # Enable relevant inspect controls
        self.btn_copy_ip.config(state="normal")
        self.btn_copy_pio.config(state="normal")
        self.btn_ping.config(state="normal")
        
        # Reset Ping Status label
        if dev['online_status'] == 'Online':
            self.lbl_ping_status.config(text="● Online", fg="#00ffcc")
        elif dev['online_status'] == 'Offline':
            self.lbl_ping_status.config(text="● No Response", fg="#ff4444")
        else:
            self.lbl_ping_status.config(text="Standby", fg="#8b949e")

    # --- ACTION CONTROLLERS ---
    def action_copy_ip(self):
        """Copies selected node IP address directly to OS clipboard."""
        if not self.selected_device_key:
            return
        ip = self.devices[self.selected_device_key]['ip']
        self.root.clipboard_clear()
        self.root.clipboard_append(ip)
        self.root.update()
        messagebox.showinfo("Clipboard", f"Copied IP Address '{ip}' to clipboard!")

    def action_copy_pio(self):
        """Generates and copies the PlatformIO OTA command using the discovered IP address."""
        if not self.selected_device_key:
            return
        ip = self.devices[self.selected_device_key]['ip']
        # Build platformio upload command
        cmd = f"pio run -t upload --upload-port {ip}"
        
        self.root.clipboard_clear()
        self.root.clipboard_append(cmd)
        self.root.update()
        messagebox.showinfo("PlatformIO Cmd", f"Copied PlatformIO upload command to clipboard:\n\n{cmd}")

    def action_ping_device(self):
        """Initiates async ping verification to keep UI responsive during networking operations."""
        if not self.selected_device_key:
            return
            
        dev = self.devices[self.selected_device_key]
        ip = dev['ip']
        port = dev['port']
        key = self.selected_device_key
        
        self.lbl_ping_status.config(text="● Pinging...", fg="#e2b834")
        self.btn_ping.config(state="disabled")
        
        # Execute ping in a safe non-blocking worker thread
        threading.Thread(
            target=self._ping_worker,
            args=(ip, port, key),
            daemon=True
        ).start()

    def _ping_worker(self, ip, port, dev_key):
        is_online = False
        
        # 1. First approach: Try subprocess command-line ICMP ping
        try:
            # -n 1 sends single packet. -w 1000 sets 1000ms timeout on Windows.
            cmd = ["ping", "-n", "1", "-w", "1000", ip]
            startup_info = None
            # Hide spawned shell command window on Windows platforms
            if os.name == 'nt':
                startup_info = subprocess.STARTUPINFO()
                startup_info.dwFlags |= subprocess.STARTF_USESHOWWINDOW
                
            p_res = subprocess.run(
                cmd, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE, 
                startupinfo=startup_info, 
                timeout=1.5
            )
            if p_res.returncode == 0:
                is_online = True
        except Exception:
            pass
            
        # 2. Alternative approach: Fallback to active TCP Socket Connect on target OTA port
        if not is_online:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(1.0)
                s.connect((ip, port))
                s.close()
                is_online = True
            except Exception:
                pass
                
        # Return result back to UI main thread safely
        self.root.after(0, lambda: self._ping_finished_callback(dev_key, is_online))

    def _ping_finished_callback(self, dev_key, is_online):
        if not self.running:
            return
            
        self.btn_ping.config(state="normal")
        
        # Update node status in memory database
        if dev_key in self.devices:
            self.devices[dev_key]['online_status'] = 'Online' if is_online else 'Offline'
            
            # Update UI text if this remains the selected device
            if self.selected_device_key == dev_key:
                if is_online:
                    self.lbl_ping_status.config(text="● Online", fg="#00ffcc")
                else:
                    self.lbl_ping_status.config(text="● No Response", fg="#ff4444")

    # --- DISCOVERY SCHEDULER & THREADING ---
    def start_discovery(self):
        """Starts either Zeroconf mDNS or Raw UDP socket fallback scanner depending on system capabilities."""
        if ZEROCONF_AVAILABLE:
            try:
                self.zeroconf_instance = zeroconf.Zeroconf()
                self.listener = OTADeviceListener(self.on_device_discovered, self.on_device_removed)
                
                # Active listeners for typical OTA protocols
                self.browsers = [
                    zeroconf.ServiceBrowser(self.zeroconf_instance, "_arduino._tcp.local.", self.listener),
                    zeroconf.ServiceBrowser(self.zeroconf_instance, "_esp_ota._udp.local.", self.listener)
                ]
                print("[Discovery] Zeroconf engine successfully started.")
            except Exception as e:
                print(f"[Discovery] Error booting Zeroconf: {e}. Reverting to raw socket.")
                self.boot_raw_udp_fallback()
        else:
            self.boot_raw_udp_fallback()

    def boot_raw_udp_fallback(self):
        """Configures and runs pure Python UDP socket scanner thread."""
        self.fallback_thread = threading.Thread(
            target=self._raw_udp_scanner_loop,
            daemon=True
        )
        self.fallback_thread.start()
        print("[Discovery] Pure UDP Socket fallback engine running.")

    def _raw_udp_scanner_loop(self):
        """
        Pure-python multicast listener background worker.
        Sends active mDNS discovery queries for '_arduino._tcp.local' and parses incoming UDP frames.
        """
        MCAST_GRP = '224.0.0.251'
        MCAST_PORT = 5353
        
        # Create socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            sock.bind(('', MCAST_PORT))
        except Exception:
            try:
                sock.bind(('0.0.0.0', MCAST_PORT))
            except Exception:
                pass
                
        # Join standard mDNS multicast group IP
        try:
            mreq = struct.pack('4sl', socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        except Exception as e:
            print(f"[Fallback Scan] Error joining multicast group: {e}")
            
        sock.settimeout(1.0)
        
        # DNS Packet Construction (Query for _arduino._tcp.local)
        # DNS Header: Transaction ID (0), Flags (0), Questions (1), Answers (0), Authorities (0), Additionals (0)
        header = struct.pack('!HHHHHH', 0, 0, 1, 0, 0, 0)
        # Question record encoded as DNS labels
        question = b'\x08_arduino\x04_tcp\x05local\x00'
        # Record Type PTR (12), Class IN (1)
        qtype_qclass = struct.pack('!HH', 12, 1)
        query_packet = header + question + qtype_qclass
        
        last_query_time = 0.0
        
        while not self.fallback_stop_event.is_set():
            now = time.time()
            # Broadcast query packet every 4 seconds to force replies
            if now - last_query_time > 4.0:
                try:
                    sock.sendto(query_packet, (MCAST_GRP, MCAST_PORT))
                    last_query_time = now
                except Exception as e:
                    print(f"[Fallback Scan] Transmit error: {e}")
            
            try:
                data, addr = sock.recvfrom(2048)
                ip = addr[0]
                
                # Check if payload appears to be an OTA response
                if b'_arduino' in data or b'arduino' in data:
                    name = "Arduino-OTA-Node"
                    
                    # Parse DNS bytes to extract any local hostname ending with '.local'
                    try:
                        local_idx = data.find(b'.local')
                        if local_idx != -1:
                            start_idx = local_idx
                            # Traverse backward looking for printable hostname characters
                            while start_idx > 0 and 32 <= data[start_idx-1] <= 126:
                                start_idx -= 1
                            name = data[start_idx:local_idx+6].decode('utf-8', errors='ignore')
                            name = name.lstrip('.')
                    except Exception:
                        pass
                        
                    # Standardize Arduino port mapping (ESP32 uses 3232, ESP8266 uses 8266)
                    port = 3232
                    if b'\x0c\xa0' in data:  # 3232 in big endian
                        port = 3232
                    elif b'\x20\x4a' in data:  # 8266 in big-endian
                        port = 8266
                        
                    props = {
                        "mode": "Raw mDNS",
                        "note": "Discovered via raw socket scan",
                        "board": "ESP32/ESP8266" if port == 3232 else "Arduino"
                    }
                    
                    # Dispatch discovered node back to UI thread
                    self.root.after(0, lambda: self.on_device_discovered(name, ip, port, f"{name}.local", props))
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[Fallback Scan] Socket read error: {e}")
                time.sleep(1.0)
                
        sock.close()

    # --- UI DATABASE CALLBACKS ---
    def on_device_discovered(self, name, ip, port, server, props):
        """Called when a new device is found or updated."""
        if not self.running:
            return
            
        key = f"{name}@{ip}"
        
        # Check if IP already exists to prevent duplication on multi-protocol advertisements
        existing_key = None
        for k, dev in self.devices.items():
            if dev['ip'] == ip:
                existing_key = k
                break
                
        now = time.time()
        
        if existing_key:
            # Update values of existing device
            dev = self.devices[existing_key]
            dev['name'] = name
            dev['port'] = port
            dev['server'] = server
            dev['properties'].update(props)
            dev['last_seen'] = now
            key_to_use = existing_key
        else:
            # Create a brand new device
            angle, radius = self.calculate_position(name, ip)
            self.devices[key] = {
                'name': name,
                'ip': ip,
                'port': port,
                'server': server,
                'properties': props,
                'last_seen': now,
                'angle': angle,
                'radius': radius,
                'last_sweep_hit': 0.0,
                'online_status': 'Unknown'
            }
            key_to_use = key
            
        # Rebuild Treeview UI to reflect database state
        self.refresh_treeview()
        
        # If the currently inspected device got updated, update inspector panel
        if self.selected_device_key == key_to_use:
            self.update_inspector_details(self.devices[key_to_use])

    def on_device_removed(self, name):
        """Called by Zeroconf browser when a device service is removed."""
        if not self.running:
            return
            
        # Match device by name and delete
        keys_to_remove = []
        for key, dev in self.devices.items():
            if dev['name'] == name:
                keys_to_remove.append(key)
                
        for key in keys_to_remove:
            if self.selected_device_key == key:
                self.selected_device_key = None
                self.reset_inspector_details()
            del self.devices[key]
            
        if keys_to_remove:
            self.refresh_treeview()

    def refresh_treeview(self):
        """Regenerates spreadsheet view items from current memory store."""
        # Save selection to restore it afterwards
        selected_row = self.tree.selection()
        selected_vals = self.tree.item(selected_row[0], "values") if selected_row else None
        
        # Clear existing items
        for item in self.tree.get_children():
            self.tree.delete(item)
            
        # Sort and re-add devices
        for dev in sorted(self.devices.values(), key=lambda d: d['name']):
            # Pick a pretty board name representation
            board = dev['properties'].get('board', '')
            if not board:
                # Deduce based on port if properties not available
                if dev['port'] == 3232:
                    board = "ESP32 (Default)"
                elif dev['port'] == 8266:
                    board = "ESP8266 (Default)"
                else:
                    board = "Arduino / Custom"
                    
            item_id = self.tree.insert(
                "", "end", 
                values=(dev['name'], dev['ip'], dev['port'], board)
            )
            
            # Restore selection if applicable
            if selected_vals and selected_vals[0] == dev['name'] and selected_vals[1] == dev['ip']:
                self.tree.selection_set(item_id)
                
        # Update device counts
        count = len(self.devices)
        self.lbl_total_devices.config(text=f"Detected Nodes: {count}")

    def reset_inspector_details(self):
        """Resets Inspector labels back to initial values."""
        self.lbl_inspect_name.config(text="-")
        self.lbl_inspect_ip.config(text="-")
        self.lbl_inspect_port.config(text="-")
        self.lbl_inspect_server.config(text="-")
        self.lbl_inspect_props.config(text="-")
        self.btn_copy_ip.config(state="disabled")
        self.btn_copy_pio.config(state="disabled")
        self.btn_ping.config(state="disabled")
        self.lbl_ping_status.config(text="")

    def on_closing(self):
        """Guarantees a clean shutdown sequence on window close."""
        self.running = False
        self.fallback_stop_event.set()
        
        # Shutdown Zeroconf browsers
        if ZEROCONF_AVAILABLE and hasattr(self, 'zeroconf_instance'):
            try:
                for b in self.browsers:
                    b.cancel()
                self.zeroconf_instance.close()
            except Exception as e:
                print(f"[Discovery] Error closing zeroconf safely: {e}")
                
        self.root.destroy()


# --- BOOTSTRAP EXECUTION ---
if __name__ == "__main__":
    root = tk.Tk()
    app = OTARadarApp(root)
    root.mainloop()
