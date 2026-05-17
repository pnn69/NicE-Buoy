import re

with open("../RoboDependency/RoboCompute/src/RoboCompute.cpp", "r") as f:
    content = f.read()

content = content.replace("numbers[14].length() > 0) dataStore->revBB = (bool)numbers[14].toInt();", "numbers[13].length() > 0) dataStore->revBB = (bool)numbers[13].toInt();")
content = content.replace("numbers[15].length() > 0) dataStore->revSB = (bool)numbers[15].toInt();", "numbers[14].length() > 0) dataStore->revSB = (bool)numbers[14].toInt();")

with open("../RoboDependency/RoboCompute/src/RoboCompute.cpp", "w") as f:
    f.write(content)
