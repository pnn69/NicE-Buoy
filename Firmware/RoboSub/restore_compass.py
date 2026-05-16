import os
import re

file_path = 'C:/tmp/NicE-Buoy/Firmware/RoboSub/src/compass.cpp'
with open(file_path, 'r') as file:
    content = file.read()

# 1. Add magSoft identity matrix sanity checks and fix temp_m / a alignment
heading_lsm_old = """    if (!mag.getEvent(&event)) return 0.0f;
    m_lsm_last = event;
    vector_t<float> temp_m = {event.magnetic.x, event.magnetic.y, event.magnetic.z};

    if (!accel.getEvent(&event)) return 0.0f;
    m_lsm_a_last = event;
    vector_t<float> a = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

    // Apply Soft and Hard Iron calibration"""

heading_lsm_new = """    if (!mag.getEvent(&event)) return 0.0f;
    m_lsm_last = event;
    vector_t<float> temp_m = {-event.magnetic.y, event.magnetic.x, -event.magnetic.z};

    if (!accel.getEvent(&event)) return 0.0f;
    m_lsm_a_last = event;
    vector_t<float> a = {-event.acceleration.y, event.acceleration.x, -event.acceleration.z};

    // Sanity check: If the soft iron matrix is all zeros, reset to Identity
    if (mainData.magSoft[0][0] == 0.0f && mainData.magSoft[1][1] == 0.0f) {
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {
                mainData.magSoft[i][j] = (i==j) ? 1.0f : 0.0f;
            }
        }
    }

    // Apply Soft and Hard Iron calibration"""

content = content.replace(heading_lsm_old, heading_lsm_new)

# 2. Add phase shift to GetHeading
get_heading_old = """    if (lsm_ready) {
        mHeding = heading((vector_t<int>){0, 1, 0});
        mHeding = mHeding + mainData.compassOffset;
    } else {"""
get_heading_new = """    if (lsm_ready) {
        mHeding = heading((vector_t<int>){0, 1, 0});
        mHeding = mHeding + mainData.compassOffset - 90.0f;
    } else {"""
content = content.replace(get_heading_old, get_heading_new)

# 3. Add phase shift to global_lsmHdg
global_lsm_old = """        if (lsm_ready) {
            global_lsmHdg = heading((vector_t<int>){0, 1, 0});
            global_lsmHdg += mainData.compassOffset;"""
global_lsm_new = """        if (lsm_ready) {
            global_lsmHdg = heading((vector_t<int>){0, 1, 0});
            global_lsmHdg += mainData.compassOffset - 90.0f;"""
content = content.replace(global_lsm_old, global_lsm_new)

# 4. Force CalibrateCompass to use Min/Max
calib_old = """        if (lokcnt > 500)
        {
            if (min_mag[0] < -20 && max_mag[0] > 20) // arbitrary check for spread"""
calib_new = """        if (lokcnt > 500)
        {
            // FORCE MIN/MAX FALLBACK OVER ELLIPSOID
            cal_lsm.useMinMax(true);
            cal_icm.useMinMax(true);
            
            if (min_mag[0] < -20 && max_mag[0] > 20) // arbitrary check for spread"""
content = content.replace(calib_old, calib_new)

with open(file_path, 'w') as file:
    file.write(content)
