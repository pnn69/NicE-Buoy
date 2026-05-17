import re

with open("src/datastorage.cpp", "r") as f:
    content = f.read()

content = content.replace("icmSoftIron(buoy, get);", "")
content = content.replace("icmHardIron(buoy, get);", "")
content = content.replace("icmCompassOffsetLoad(buoy, get);", "")
content = content.replace("CompassOffsetCorrection(buoy, get);", "") # Wait, if it replaced icmCompassOffsetLoad with CompassOffsetCorrection it might be there

with open("src/datastorage.cpp", "w") as f:
    f.write(content)
