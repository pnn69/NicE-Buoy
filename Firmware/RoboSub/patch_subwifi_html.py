import re

with open("src/subwifi.cpp", "r") as f:
    content = f.read()

# Replace lsm with icm in the JS part so it doesn't crash
content = content.replace("data.lsm.toFixed(1)", "data.icm.toFixed(1)")
content = content.replace("updateBar('lsm', 'x', data.lsm_x, data.lsm_min.x, data.lsm_max.x);", "// lsm removed")
content = content.replace("updateBar('lsm', 'y', data.lsm_y, data.lsm_min.y, data.lsm_max.y);", "// lsm removed")
content = content.replace("updateBar('lsm', 'z', data.lsm_z, data.lsm_min.z, data.lsm_max.z);", "// lsm removed")
content = content.replace("drawRose(data.lsm, data.icm);", "drawRose(0, data.icm);")

with open("src/subwifi.cpp", "w") as f:
    f.write(content)
