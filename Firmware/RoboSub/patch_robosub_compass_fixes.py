import re

with open("src/compass.cpp", "r") as f:
    content = f.read()

content = content.replace("compassOffsetLoad", "CompasOffset")
content = content.replace("HardIron", "hardIron")
content = content.replace("SoftIron", "softIron")

with open("src/compass.cpp", "w") as f:
    f.write(content)
