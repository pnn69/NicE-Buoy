import re

with open("src/main.cpp", "r") as f:
    content = f.read()

content = content.replace("CompassOffsetCorrection(&dataIn, false);\n", "")
content = content.replace("CompassOffsetCorrection(ser, GET);\n", "")
content = content.replace("CompassOffsetCorrection(&dataIn, SET);\n", "")

with open("src/main.cpp", "w") as f:
    f.write(content)
