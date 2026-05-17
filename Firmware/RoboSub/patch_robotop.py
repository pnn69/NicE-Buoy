import re
import os

top_dir = "../../Firmware/RoboTop/src"

with open(os.path.join(top_dir, "main.cpp"), "r") as f:
    content = f.read()

content = content.replace("RfOut->icmCompassOffset = RfIn.icmCompassOffset; // Update Top Buoy's local data", "")

with open(os.path.join(top_dir, "main.cpp"), "w") as f:
    f.write(content)

with open(os.path.join(top_dir, "topwifi.cpp"), "r") as f:
    content = f.read()

content = re.sub(r'\s*json \+= "\\"icmCompassOffset\\":\\"" \+ String\(mainData.icmCompassOffset, 2\) \+ "\\",";\n', '\n', content)
content = re.sub(r'\s*json \+= "\\"icmCompassOffset\\":\\"" \+ String\(buoyPara\[i\]\.icmCompassOffset, 2\) \+ "\\",";\n', '\n', content)
content = re.sub(r'\s*mainData\.icmCompassOffset = server\.arg\("icmoffset"\)\.toFloat\(\);\n', '\n', content)
content = re.sub(r'\s*msg\.icmCompassOffset = server\.arg\("icmoffset"\)\.toFloat\(\);\n', '\n', content)

with open(os.path.join(top_dir, "topwifi.cpp"), "w") as f:
    f.write(content)
