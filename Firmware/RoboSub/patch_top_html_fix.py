import re

with open("Firmware/RoboTop/data/index.html", "r") as f:
    content = f.read()

content = content.replace("setVal(, b.icmCompassOffset);", "")

with open("Firmware/RoboTop/data/index.html", "w") as f:
    f.write(content)
