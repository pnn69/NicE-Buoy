import re
with open('C:/tmp/NicE-Buoy/Firmware/RoboTop/src/sercom.cpp', 'r') as file:
    content = file.read()
content = content.replace('xQueueSend(serIn, (void *)&serDataIn, 10);', 'printf("SER_IN CMD=%d DIRMAG=%.2f\\n", serDataIn.cmd, serDataIn.dirMag); xQueueSend(serIn, (void *)&serDataIn, 10);')
with open('C:/tmp/NicE-Buoy/Firmware/RoboTop/src/sercom.cpp', 'w') as file:
    file.write(content)
