from enum import Enum

class MsgType(Enum):
    LOTAGET = 1              # info request
    LORASET = 2              # info to store
    LORAGETACK = 3           # ack required
    LORAACK = 4              # ack on message
    LORANAC = 5              # nak
    LORAINF = 6              # update message
    IDLE = 7
    IDELING = 8
    PING = 9
    PONG = 10
    LOCKING = 11
    LOCKED = 12
    LOCK_POS = 13
    DOCKING = 14
    DOCKED = 15
    DOC = 16
    UNLOCK = 17
    REMOTE = 18
    REMOTEING = 19
    CALIBRATE_MAGNETIC_COMPASS = 20
    LINEAR_CALLIBRATING = 21
    CALIBRATE_OFFSET_MAGNETIC_COMPASS = 22
    STORE_CALIBRATE_OFFSET_MAGNETIC_COMPASS = 23
    DOCK_STORING = 24
    MUTE_ESC = 25
    BLINK_SLOW = 26
    BLINK_FAST = 27
    BLINK_OFF = 28
    SUBDATA = 29            # all data send known by sub
    SUBACCU = 30            # V,P accu voltage, accu percentage
    SUBDIR = 31             # magnetic direction
    SUBSPEED = 32           # speed(given), speed BB, speed SB
    SUBDIRSPEED = 33        # mDir,speedbb,speedsb,speed
    TOPIDLE = 34
    TOPID = 35              # mac[unsigned long]
    TOPDATA = 36            # ?
    TOPDIRSPEED = 37        # Dir,Speed
    LORADIRSPEED = 38       # Dir,Speed
    TOPDIRDIST = 39         # Direction and distance
    TOPSPBBSPSB = 40        # SpeedBb,SpeedSb
    TOPROUTTOPOINT = 41     # route to point data
    TOPCALCRUDDER = 42      # tgDir,tgDist,Speed
    PIDRUDDER = 43          # Prudder,Irudder,Drudder,kp,ki,kd PID parameters rudder + act data (p i d t) t = total
    PIDRUDDERSET = 44       # Prudder,Irudder,Drudder,kp,ki,kd PID parameters rudder
    PIDSPEED = 45           # Pspeed,Ispeed,Dspeed,kp,ki,kd PID parameters speed + act data (p i d t) t= total
    PIDSPEEDSET = 46        # Pspeed,Ispeed,Dspeed,kp,ki,kd PID parameters speed
    SUBID = 47              # mac sub
    UDPERROR = 48           # no udp communication
    STOREASDOC = 49         # Store location as doc location
    LORABUOYPOS = 50        # STATUS,LAT,LON,mDir,wDir,wStd,BattPercTop,BattPercBott,speedbb,speedsb
    LORALOCKPOS = 51        # LAT,LON,wDir,wStd
    LORADOCKPOS = 52        # LAT,LON.wDir,wStd
    LORADIRDIST = 53        # tgDir,tgDist
    LORASENDTRACK = 54      # new track positions of buoys
    LORASENDTXT = 55        # Text message
    LORAIDELING = 56
    COMPUTESTART = 57
    COMPUTETRACK = 58
    UDPTGDIRSPEED = 59      # compute speed dir
    UDPDIRSPEED = 60        # speed dir
    NEWBUOYPOS = 61         # new computed buoy pos
    ROBODEFAULTS = 62

if __name__ == "__main__":    
    print(MsgType.LOTAGET)
    print(MsgType.LOTAGET.value)  # Output: 1