import re

# compass.cpp updates
with open("src/compass.cpp", "r") as f:
    content = f.read()

content = content.replace("icmCompassOffset", "compassOffset")
content = content.replace("icmMagHard", "magHard")
content = content.replace("icmMagSoft", "magSoft")
content = content.replace("icmHardIron", "HardIron")
content = content.replace("icmSoftIron", "SoftIron")

with open("src/compass.cpp", "w") as f:
    f.write(content)

# main.cpp updates
with open("src/main.cpp", "r") as f:
    content = f.read()

# remove icmCompassOffset uses
content = content.replace("dataIn.icmCompassOffset", "dataIn.compassOffset")
content = content.replace("ser->icmCompassOffset = dataIn.compassOffset;", "")
content = content.replace("icmCompassOffsetLoad", "CompassOffsetCorrection") # Map load func
content = content.replace('printf("New compass offset: %f | %f", dataIn.compassOffset, dataIn.compassOffset);', 'printf("New compass offset: %f", dataIn.compassOffset);')

with open("src/main.cpp", "w") as f:
    f.write(content)

# datastorage.h updates
with open("src/datastorage.h", "r") as f:
    content = f.read()

content = content.replace("void icmHardIron(RoboStruct *buoy, bool get);\n", "")
content = content.replace("void icmSoftIron(RoboStruct *buoy, bool get);\n", "")
content = content.replace("void icmCompassOffsetLoad(RoboStruct *buoy, bool get);\n", "")

with open("src/datastorage.h", "w") as f:
    f.write(content)

# datastorage.cpp updates
with open("src/datastorage.cpp", "r") as f:
    content = f.read()

# Remove icmHardIron, icmSoftIron, icmCompassOffsetLoad entirely
content = re.sub(r'void icmHardIron\(RoboStruct \*buoy, bool get\)\s*\{.*?\n\}', '', content, flags=re.DOTALL)
content = re.sub(r'void icmSoftIron\(RoboStruct \*buoy, bool get\)\s*\{.*?\n\}', '', content, flags=re.DOTALL)
content = re.sub(r'void icmCompassOffsetLoad\(RoboStruct \*buoy, bool get\)\s*\{.*?\n\}', '', content, flags=re.DOTALL)

with open("src/datastorage.cpp", "w") as f:
    f.write(content)
