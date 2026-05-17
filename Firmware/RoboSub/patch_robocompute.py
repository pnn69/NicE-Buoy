import re

with open("../RoboDependency/RoboCompute/src/RoboCompute.h", "r") as f:
    content = f.read()

content = re.sub(r'\s*double icmMagHard\[3\].*;\n', '\n', content)
content = re.sub(r'\s*double icmMagSoft\[3\]\[3\].*;\n', '\n', content)
content = re.sub(r'\s*double icmCompassOffset.*;\n', '\n', content)

with open("../RoboDependency/RoboCompute/src/RoboCompute.h", "w") as f:
    f.write(content)

with open("../RoboDependency/RoboCompute/src/RoboCompute.cpp", "r") as f:
    content = f.read()

# Remove icmCompassOffset from STORE_COMPASS_OFFSET
content = re.sub(r'\s*dataStore->icmCompassOffset = numbers\[3\]\.toDouble\(\);\n', '\n', content)
content = re.sub(r'\s*out \+= "," \+ String\(dataOut->icmCompassOffset,\s*2\);\n', '\n', content)

# Remove icmCompassOffset from SETUPDATA
content = re.sub(r'\s*if \(numbers\[13\]\.length\(\) > 0\) dataStore->icmCompassOffset = numbers\[13\]\.toDouble\(\);\n', '\n', content)

with open("../RoboDependency/RoboCompute/src/RoboCompute.cpp", "w") as f:
    f.write(content)
