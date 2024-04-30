from tkinter import *
from bluetooth import *
import math
import webbrowser
import serial
import threading
import re
import compass



font_settings = ("Arial", 8)  # Change the font name and size as needed

windowh = 350
windoww = 500
latitude = None
longitude = None
gps_fix = False
gps_sat = None
gps_speed = None
gps_hdg = None
buoy_hdg = None
tg_hdg = None
tg_dst = None
bb_sp = None
sb_sp = None
gps_collor  = "red"
buoy_collor = "green"
tg_collor = "blue"
blk_collor = "black"

# Open COM3 port
#ser = serial.Serial('COM7', 115200)  # Adjust baud rate as per your requirement


def read_bleutooth():
    server_sock = BluetoothSocket(RFCOMM)
    server_sock.bind(("", PORT_ANY))
    server_sock.listen(1)
    port = server_sock.getsockname()[1]
    uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
    advertise_service(server_sock, "TestServer", service_id=uuid,
                    service_classes=[uuid, SERIAL_PORT_CLASS],
                    profiles=[SERIAL_PORT_PROFILE])

    while True:
        print(f"Waiting for connection on RFCOMM channel {port}")
        client_sock, client_info = server_sock.accept()
        print("Accepted connection from", client_info)
    
        try:
            while True:
                # Receive data from the client
                data = client_sock.recv(1024)
                if not data:
                    break
                
                # Print the received data
                newdata = data.decode("utf-8")
                print("Received:", newdata)
                decode_message(newdata)
                in_box1.delete(1.0, END)   
                in_box1.insert(END, f"{newdata}")

        except KeyboardInterrupt:
            pass
        finally:
            # Close the client socket
            client_sock.close()
            # Close the server socket
            server_sock.close()








def Adjust_position():
    pos_dir_content = adj_pos_dir.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    pos_dist_content = adj_pos_dst.get("1.0", "end-1c")  # Get content of adj_pos_dir2
    out = "*^1^31^1^" + pos_dir_content + "," + pos_dist_content + "^1"
    out_box1.delete(1.0, END)   
    out_box1.insert(END, out)
    #ser.write(out.encode())

def Adjust_speed_pid():
    p = adj_speed_p.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    i = adj_speed_i.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    d = adj_speed_d.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    out = "*^1^29^1^" + p + "," + i +","+ d + "^1"
    out_box1.delete(1.0, END)   
    out_box1.insert(END, out)
    #ser.write(out.encode())

def Adjust_rudder_pid():
    p = adj_rudder_p.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    i = adj_rudder_i.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    d = adj_rudder_d.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    out = "*^1^30^1^" + p + "," + i +","+ d + "^1"
    out_box1.delete(1.0, END)   
    out_box1.insert(END, out)
    #ser.write(out.encode())
    
def idle():
    out = "*^1^21^1^0^1"
    out_box1.delete(1.0, END)   
    out_box1.insert(END, out)
    #ser.write(out.encode())

def lock_position():
    out = "*^1^4^1^0^1"
    out_box1.delete(1.0, END)   
    out_box1.insert(END, out)
    #ser.write(out.encode())
    
def doc_position():
    out = "*^1^10^1^0^1"
    out_box1.delete(1.0, END)   
    out_box1.insert(END, out)
    #ser.write(out.encode())
   
#<97.00,60.02,76,-76,76,188>
def decode_18(data):#DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_M_HEADING
    global tg_hdg, tg_dst, bb_sp,sb_sp,buoy_hdg , speed, dist_txt
    values = data.group(5).split(',')
    if len(values) == 6:
        tg_hdg = int(float(values[0]))
        #tg_hdg = tg_hdg + 180
        #if tg_hdg > 360:
        #    tg_hdg = tg_hdg - 360
        tg_dst = float(values[1])
        bb_sp = int(float(values[3]))
        sb_sp = int(float(values[4]))
        buoy_hdg = int(float(values[5]))
        compass.draw_barr(bb_sp,sb_sp,comp)
        compass.draw_pointer(tg_hdg,comp,tg_collor)
        compass.draw_pointer(buoy_hdg,comp,buoy_collor)
        output_str = f"{bb_sp}% , {sb_sp}%"
        speed.config(text=output_str)
        output_str = f"Target distance:{tg_dst} M"
        dist_txt.config(text=output_str)
        
def decode_19(data):#DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_M_HEADING
    global tg_hdg, tg_dst, bb_sp,sb_sp,buoy_hdg , speed
    values = data.group(5).split(',')
    if len(values) == 4:
        bb_sp = int(values[2])
        sb_sp = int(values[3])
        output_str = f"{bb_sp}% , {sb_sp}%"
        speed.config(text=output_str)
        compass.draw_barr(bb_sp,sb_sp,comp)

def decode_20(data):#DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_M_HEADING
    global bb_sp,sb_sp
    values = data.group(5).split(',')
    if len(values) == 4:
        bb_sp = int(values[0])
        sb_sp = int(values[1])
        output_str = f"{bb_sp}% , {sb_sp}%"
        speed.config(text=output_str)
        compass.draw_barr(bb_sp,sb_sp)
    
     #<52.32038000,4.96563000,10,1,0,0,3.00>
def decode_23(data): #GPS_LAT_LON_NRSAT_FIX_HEADING_SPEED_MHEADING,  // lat,lon,fix,heading,speed,m_heading
    global latitude, longitude, gps_fix, comp
    values = data.group(5).split(',')
    if len(values) == 7:
        gpspos.config(text=data.group(5))
        latitude = values[0]
        longitude = values[1]
        gps_sat =  int(values[2])
        if values[3] == "1":
            gps_fix = True
        else:
            gps_fix = False
        gps_hdg = int(values[4])
        gps_speed = int(values[5])
        buoy_hdg = int(float(values[6]))
        if gps_speed > 5:
            compass.draw_pointer(gps_hdg,comp,gps_collor)
        else:
            compass.draw_pointer(0,comp,blk_collor)
        compass.draw_pointer(buoy_hdg,comp,buoy_collor)
    
def decode_24(data): #BATTERY_VOLTAGE_PERCENTAGE,                    // 0.0V, %
    values = data.group(5).split(',')
    if len(values) == 2:
        voltage_str, percentage_str = data.group(5).split(',')
        try:
            voltage = float(voltage_str)
            percentage = float(percentage_str)
            output_str = f"Voltage: {voltage} Percentage: {percentage}%"
            batt.config(text=output_str)
        except ValueError:
            print("One or both of the variables is not a float")        

def decode_29(data):
    values = data.group(5).split(',')
    if len(values) == 4:
        adj_speed_kI.config(text="P:" + values[0] + ", I:" + values[1]+ ",D:" + values[2] + ", Ki" + values[3])  # Pre-fill entry1    

def decode_30(data):
    values = data.group(5).split(',')
    if len(values) == 4:
        adj_rudder_kI.config(text="P:" + values[0] + ", I:" + values[1]+ ",D:" + values[2] + ", Ki" + values[3])  # Pre-fill entry1    

def decode_32(data):
    values = data.group(5).split(',')
    if len(values) == 2:
        wind_dir.config(text=values[0])
        wind_dev.config(text=values[1])

def decode_34(data):
    global buoy_hdg
    values = data.group(5).split(',')
    if len(values) == 1:
        buoy_hdg = int(float(values[0]))
        compass.draw_pointer(buoy_hdg,comp,buoy_collor)

def decode_status(data):
    if data == "5":
        status_txt.config(text=f"IDLE")
    if data == "7":
        status_txt.config(text=f"LOCKED")
    if data == "13":
        status_txt.config(text=f"DOCKED")
    
def open_google_maps():
    global latitude, longitude, gps_fix
    # Construct the Google Maps URL with the coordinates
    #if latitude is not None and longitude is not None:
    if gps_fix == True:
        url = f"https://www.google.com/maps?q={latitude},{longitude},20z?entry=ttu"
        # Open the URL in a web browser
        webbrowser.open(url)     

def decode_xx(data):
    values = data.group(5).split(',')
    print(len(values));
    #if len(values) == 2:
     #   printf(len)

    in_box1.delete(1.0, END)   
    in_box1.insert(END, f"{data}")

def test():
    msg = "*^1^23^1^maffe data^1"
    #ser.write(msg.encode())
    
    

def remove_spaces(string):
    return "".join(string.split())

def decode_message(message):
    # Regular expression pattern to match the message format: <sender><status><msgId><ack><msg>
    pattern = r'<(\d+)><(\d+)><(\d+)><(\d+)><(.+)>'
    # Match the pattern in the message
    match = re.match(pattern, message)
    if match:
        msg_id = match.group(3)
        decode_status(match.group(2))
        if(msg_id == "18"):
            decode_18(match)
        if(msg_id == "19"):
            decode_19(match)
        if(msg_id == "20"):
            decode_20(match)
        if(msg_id == "23"):
            decode_23(match)
        if(msg_id == "24"):
            decode_24(match)
        if(msg_id == "29"):
            decode_29(match)
        if(msg_id == "30"):
            decode_30(match)
        if(msg_id == "32"):
            decode_32(match)
        if(msg_id == "34"):
            decode_34(match)

def read_serial():
    while True:
        #if #ser.in_waiting > 0:
        #    try:
        #        received_data = #ser.readline().decode().strip()
        #    except UnicodeDecodeError as e:
        #        received_data = "troep"
        #    if received_data != "troep":
        received_data = remove_spaces(received_data)
        in_box1.delete(1.0, END)   
        in_box1.insert(END, f"{received_data}")
        decode_message(received_data)
        print(received_data)

def start_serial_thread():
    serial_thread = threading.Thread(target=read_serial)
    serial_thread.daemon = True
    serial_thread.start()
   
def start_compass_thread():
    serial_thread = threading.Thread(target=write_compass)
    serial_thread.daemon = True
    serial_thread.start()

def start_ble_thread():
    serial_thread = threading.Thread(target=hallo)
    serial_thread.daemon = True
    serial_thread.start()

def hallo():
    read_bleutooth()

def write_compass():
    compass.root.mainloop()
# Thread to run the compass
def start_compass_thread():
    compass_thread = threading.Thread(target=write_compass)
    compass_thread.daemon = True
    compass_thread.start()


root = Tk()
root.title("Nice buoy control!")
frame = Frame(root, width=windoww, height=windowh)
frame.pack()
statuslb = Label(text="STATUS:")
statuslb.place(x=windoww/2-20, y=10)
status_txt = Label(text="?")
status_txt.place(x=windoww/2+30, y=10)
dist_txt = Label(text="")
dist_txt.place(x=windoww/2-20, y=30)


adj_pos = Button(frame, text="Adjust position", command=Adjust_position)
adj_pos.place(x=10, y=10, height=30, width=100)
adj_pos_dir = Text(frame)
adj_pos_dir.configure(font=font_settings)
adj_pos_dir.insert(END, "-90")  # Pre-fill entry1
adj_pos_dir.place(x=120, y=15, height=20, width=30)
adj_pos_dst = Text(frame)
adj_pos_dst.configure(font=font_settings)
adj_pos_dst.insert(END, "30")  # Pre-fill entry2
adj_pos_dst.place(x=120 + 30, y=15, height=20, width=30)

adj_speed_pid = Button(frame, text="Adjust speed pid",command=Adjust_speed_pid)
adj_speed_pid.place(x=10, y=50, height=30, width=100)
adj_speed_p = Text(frame)
adj_speed_p.configure(font=font_settings)
adj_speed_p.insert(END, "20")  # Pre-fill entry1
adj_speed_p.place(x=120, y=55, height=20, width=30)
adj_speed_i = Text(frame)
adj_speed_i.configure(font=font_settings)
adj_speed_i.insert(END, "0.01")  # Pre-fill entry1
adj_speed_i.place(x=120 + 30, y=55, height=20, width=30)
adj_speed_d = Text(frame)
adj_speed_d.configure(font=font_settings)
adj_speed_d.insert(END, "0.1")  # Pre-fill entry1
adj_speed_d.place(x=120 + 60, y=55, height=20, width=30)
adj_speed_kI = Label(text="")
adj_speed_kI.place(x=210, y=55)

adj_rudder_pid = Button(frame, text="Adjust rudder pid",command=Adjust_rudder_pid)
adj_rudder_pid.place(x=10, y=90, height=30, width=100)
adj_rudder_p = Text(frame)
adj_rudder_p.configure(font=font_settings)
adj_rudder_p.insert(END, "1")  # Pre-fill entry1
adj_rudder_p.place(x=120, y=95, height=20, width=30)
adj_rudder_i = Text(frame)
adj_rudder_i.configure(font=font_settings)
adj_rudder_i.insert(END, "0.1")  # Pre-fill entry1
adj_rudder_i.place(x=120 + 30, y=95, height=20, width=30)
adj_rudder_d = Text(frame)
adj_rudder_d.configure(font=font_settings)
adj_rudder_d.insert(END, "0")  # Pre-fill entry1
adj_rudder_d.place(x=120 + 60, y=95, height=20, width=30)
adj_rudder_kI = Label(text="0.0")
adj_rudder_kI.place(x=210, y=95)

lock = Button(frame, text="IDLE", command=idle)
lock.place( x=windoww - 100 -10,y=10, height=30, width=100)

lock = Button(frame, text="Lock position", command=lock_position)
lock.place( x=windoww - 100 -10,y=50, height=30, width=100)

dock = Button(frame, text="Sail to doc", command=doc_position)
dock.place( x=windoww - 100 -10,y=90, height=30, width=100)

# Create a button to open Google Maps
open_maps_button = Button(root, text="Open Google Maps", command=open_google_maps)
open_maps_button.pack(pady=10)
#dock.place( x=windoww - 100 -10,y=90, height=30, width=100)

#placeholders incomming data
wind_label_dir = Label(text="Wind direction:")
wind_label_dir.place(x=10, y=130)
wind_dir = Label(text="0")
wind_dir.place(x=90, y=130)
wind_label_dev = Label(text="Deviation:")
wind_label_dev.place(x=120, y=130)
wind_dev = Label(text="0")
wind_dev.place(x=175, y=130)

speed = Label(text="Pwr bb,sb:")
speed.place(x=10, y=150)
speed = Label(text="0%,0%")
speed.place(x=75, y=150)

batt = Label(text="Battery:")
batt.place(x=10, y=170)
batt = Label(text="0.0,0%")
batt.place(x=75, y=170)

gpspos = Label(text="GPS pos:")
gpspos.place(x=10, y=190)
gpspos = Label(text="?")
gpspos.place(x=75, y=190)



test = Button(frame, text="TESTING", command=test)
test.place( x=windoww/2-50,y=(windowh - 100), height=30, width=100)


out_text = Label(text="Out:")
out_text.place(x=10, y=(windowh - 60))
out_box1 = Text(frame)
out_box1.place(x=50, y=(windowh - 60), height=20, width=(windoww - 50 - 10))

out_text = Label(text="In:")
out_text.place(x=10, y=(windowh - 30))
in_box1 = Text(frame)
in_box1.place(x=50, y=(windowh - 30), height=20, width=(windoww - 50 - 10))
comp = compass.create_compass(root)
#start_serial_thread()  # Start the serial reading thread
start_ble_thread()
root.mainloop()
