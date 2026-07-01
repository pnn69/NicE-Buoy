import re

def clean_file(file_path, patterns):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    new_lines = []
    for line in lines:
        match = False
        # Check for printf and the specific patterns
        if 'printf' in line and not line.strip().startswith('//'):
            for p in patterns:
                if p in line:
                    # Don't comment out version prints
                    if "Robobuoy Sub Version" in line or "Robobuoy ID" in line:
                        continue
                    new_lines.append('// ' + line.lstrip())
                    match = True
                    break
        if not match:
            new_lines.append(line)
            
    with open(file_path, 'w') as f:
        f.writelines(new_lines)

sercom_patterns = [
    "ACK_STORE:", "ACK_REMOVE:", "ACK_RETRY:",
    "SER_PC_IN", "SER_TOP_ACK", "SER_TOP_IN", "SER_TOP_ACK_SEND",
    "SER_TOP_OUT>", "SER_TOP_RETRY>"
]

main_patterns = [
    "Entering IDLE state (ramping down)",
    "IDLE command recieved (ramping down)",
    "New rudder PID settings",
    "Rudder PID stored",
    "New speed PID settings",
    "Speed PID stored",
    "Sent SETUPDATA back to",
    "New setup received",
    "Resetting PID"
]

clean_file(r'C:\tmp\NicE-Buoy\Firmware\RoboSub\src\sercom.cpp', sercom_patterns)
clean_file(r'C:\tmp\NicE-Buoy\Firmware\RoboSub\src\main.cpp', main_patterns)
print("Files cleaned successfully.")
