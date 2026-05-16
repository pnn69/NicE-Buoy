import re
with open('C:/tmp/NicE-Buoy/Firmware/RoboSub/src/compass.cpp', 'r') as file:
    content = file.read()
content = content.replace('"off":%.1f', '"off":%.1f,"mac":%lu')
content = content.replace(', mainData.compassOffset,', ', mainData.compassOffset, espMac(),')
with open('C:/tmp/NicE-Buoy/Firmware/RoboSub/src/compass.cpp', 'w') as file:
    file.write(content)
