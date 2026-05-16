import os

file_path = 'C:/tmp/NicE-Buoy/Firmware/RoboSub/src/compass.cpp'
with open(file_path, 'r') as file:
    content = file.read()

# Change forward vector in GetHeading()
get_heading_old = """    if (lsm_ready) {
        mHeding = heading((vector_t<int>){0, 1, 0});
        mHeding = mHeding + mainData.compassOffset;
    } else {
        mHeding = heading_icm((vector_t<int>){0, 1, 0});
        mHeding = mHeding + mainData.icmCompassOffset;
    }"""
get_heading_new = """    if (lsm_ready) {
        mHeding = heading((vector_t<int>){1, 0, 0});
        mHeding = mHeding + mainData.compassOffset;
    } else {
        mHeding = heading_icm((vector_t<int>){1, 0, 0});
        mHeding = mHeding + mainData.icmCompassOffset;
    }"""
content = content.replace(get_heading_old, get_heading_new)

# Change forward vector in global hdg assignments
global_old = """        if (lsm_ready) {
            global_lsmHdg = heading((vector_t<int>){0, 1, 0});
            global_lsmHdg += mainData.compassOffset;
            if (!isnan(global_lsmHdg)) {
                global_lsmHdg = fmod(global_lsmHdg, 360.0);
                if (global_lsmHdg < 0) global_lsmHdg += 360.0;
            }
        } else {
            global_lsmHdg = 0.0;
        }

        if (icm_ready) {
            global_icmHdg = heading_icm((vector_t<int>){0, 1, 0});
            global_icmHdg += mainData.icmCompassOffset;"""

global_new = """        if (lsm_ready) {
            global_lsmHdg = heading((vector_t<int>){1, 0, 0});
            global_lsmHdg += mainData.compassOffset;
            if (!isnan(global_lsmHdg)) {
                global_lsmHdg = fmod(global_lsmHdg, 360.0);
                if (global_lsmHdg < 0) global_lsmHdg += 360.0;
            }
        } else {
            global_lsmHdg = 0.0;
        }

        if (icm_ready) {
            global_icmHdg = heading_icm((vector_t<int>){1, 0, 0});
            global_icmHdg += mainData.icmCompassOffset;"""
content = content.replace(global_old, global_new)

with open(file_path, 'w') as file:
    file.write(content)
