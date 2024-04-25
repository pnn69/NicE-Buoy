from tkinter import *
import serial
import threading
import re

windowh = 300
windoww = 500

def Adjust_position():
    pos_dir_content = adj_pos_dir.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    pos_dist_content = adj_pos_dst.get("1.0", "end-1c")  # Get content of adj_pos_dir2
    out_box1.delete(1.0, END)   
    out_box1.insert(END, f"1'CHANGE_POS_DIR_DIST'{pos_dir_content}'{pos_dist_content}")

def Adjust_speed_pid():
    p = adj_speed_p.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    i = adj_speed_i.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    d = adj_speed_d.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    out_box1.delete(1.0, END)   
    out_box1.insert(END, f"1'PID_SPEED_PARAMETERS'{p}'{i}'{d}")

def Adjust_rudder_pid():
    p = adj_rudder_p.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    i = adj_rudder_i.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    d = adj_rudder_d.get("1.0", "end-1c")  # Get content of adj_pos_dir1
    out_box1.delete(1.0, END)   
    out_box1.insert(END, f"1'PID_RUDDER_PARAMETERS'{p}'{i}'{d}")
    
def idle():
    out_box1.delete(1.0, END)   
    out_box1.insert(END, f"1'IDLE")

def lock_position():
    out_box1.delete(1.0, END)   
    out_box1.insert(END, f"1'TARGET_POSITION")
    
def doc_position():
    out_box1.delete(1.0, END)   
    out_box1.insert(END, f"1'DOC_POSITION")
   

def decode_23(data): #GPS_LAT_LON_NRSAT_FIX_HEADING_SPEED_MHEADING,  // lat,lon,fix,heading,speed,m_heading
    gpspos.config(text=data.group(5))
def decode_24(data): #BATTERY_VOLTAGE_PERCENTAGE,                    // 0.0V, %
    gpspos.config(text=data.group(5))
def decode_status(data):
    if data == "5":
        status_txt.config(text=f"IDLE")
    if data == "7":
        status_txt.config(text=f"LOCKED")
    if data == "13":
        status_txt.config(text=f"DOCED")
    
     

def decode_xx(data):
    in_box1.delete(1.0, END)   
    in_box1.insert(END, f"{data}")

def test():
    decode_message("<1><7><23><1><maffe data>")
    



def decode_message(message):
    # Regular expression pattern to match the message format: <sender><status><msgId><ack><msg>
    pattern = r'<(\d+)><(\d+)><(\d+)><(\d+)><(.+)>'
    # Match the pattern in the message
    match = re.match(pattern, message)
    if match:
        sender = match.group(1)
        status = match.group(2)
        msg_id = match.group(3)
        ack = match.group(4)
        msg = match.group(5)
        print("Sender:", sender)
        print("Status:", status)
        print("Message ID:", msg_id)
        print("Acknowledge:", ack)
        print("Message:", msg)
        decode_status("7")
        if(msg_id == "23"):
            decode_23(match)
        if(msg_id == ""):
            decode_xx(msg)
            
    else:
        print("Invalid message format:", message)





root = Tk()
root.title("Nice buoy control!")
frame = Frame(root, width=windoww, height=windowh)
frame.pack()
statuslb = Label(text="STATUS:")
statuslb.place(x=windoww/2-20, y=10)
status_txt = Label(text="?")
status_txt.place(x=windoww/2+30, y=10)

adj_pos = Button(frame, text="Adjust position", command=Adjust_position)
adj_pos.place(x=10, y=10, height=30, width=100)
adj_pos_dir = Text(frame)
adj_pos_dir.insert(END, "-90")  # Pre-fill entry1
adj_pos_dir.place(x=120, y=15, height=20, width=40)
adj_pos_dst = Text(frame)
adj_pos_dst.insert(END, "30")  # Pre-fill entry2
adj_pos_dst.place(x=120 + 40, y=15, height=20, width=40)

adj_speed_pid = Button(frame, text="Adjust speed pid",command=Adjust_speed_pid)
adj_speed_pid.place(x=10, y=50, height=30, width=100)
adj_speed_p = Text(frame)
adj_speed_p.insert(END, "20")  # Pre-fill entry1
adj_speed_p.place(x=120, y=55, height=20, width=40)
adj_speed_i = Text(frame)
adj_speed_i.insert(END, "0.01")  # Pre-fill entry1
adj_speed_i.place(x=120 + 40, y=55, height=20, width=40)
adj_speed_d = Text(frame)
adj_speed_d.insert(END, "0.1")  # Pre-fill entry1
adj_speed_d.place(x=120 + 80, y=55, height=20, width=40)
adj_speed_kItxt = Label(text="kI:")
adj_speed_kItxt.place(x=250, y=55)
adj_speed_kI = Label(text="0.0")
adj_speed_kI.place(x=270, y=55)

adj_rudder_pid = Button(frame, text="Adjust rudder pid",command=Adjust_rudder_pid)
adj_rudder_pid.place(x=10, y=90, height=30, width=100)
adj_rudder_p = Text(frame)
adj_rudder_p.insert(END, "1")  # Pre-fill entry1
adj_rudder_p.place(x=120, y=95, height=20, width=40)
adj_rudder_i = Text(frame)
adj_rudder_i.insert(END, "0.1")  # Pre-fill entry1
adj_rudder_i.place(x=120 + 40, y=95, height=20, width=40)
adj_rudder_d = Text(frame)
adj_rudder_d.insert(END, "0")  # Pre-fill entry1
adj_rudder_d.place(x=120 + 80, y=95, height=20, width=40)
adj_rudder_kItxt = Label(text="kI:")
adj_rudder_kItxt.place(x=250, y=95)
adj_rudder_kI = Label(text="0.0")
adj_rudder_kI.place(x=270, y=95)

lock = Button(frame, text="IDLE", command=idle)
lock.place( x=windoww - 100 -10,y=10, height=30, width=100)

lock = Button(frame, text="Lock position", command=lock_position)
lock.place( x=windoww - 100 -10,y=50, height=30, width=100)

dock = Button(frame, text="Sail to doc", command=doc_position)
dock.place( x=windoww - 100 -10,y=90, height=30, width=100)


#placeholders incomming data
wind_label_dir = Label(text="Wind direction:")
wind_label_dir.place(x=10, y=130)
wind_dir = Label(text="350")
wind_dir.place(x=90, y=130)
wind_label_dev = Label(text="Deviation:")
wind_label_dev.place(x=120, y=130)
wind_dev = Label(text="35")
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

root.mainloop()
