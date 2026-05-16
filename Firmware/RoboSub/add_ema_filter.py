import os
import re

file_path = 'C:/tmp/NicE-Buoy/Firmware/RoboSub/src/compass.cpp'
with open(file_path, 'r') as file:
    content = file.read()

# 1. EMA filter for heading()
heading_lsm_old = """    vector_t<float> a = {-event.acceleration.y, event.acceleration.x, -event.acceleration.z};"""
heading_lsm_new = """    // EMA filter to remove thruster-induced vibration from the gravity vector
    static vector_t<float> a_avg = {0,0,0};
    if(a_avg.x == 0.0f && a_avg.y == 0.0f) {
        a_avg = {-event.acceleration.y, event.acceleration.x, -event.acceleration.z};
    } else {
        a_avg.x = a_avg.x * 0.95f + (-event.acceleration.y) * 0.05f;
        a_avg.y = a_avg.y * 0.95f + (event.acceleration.x) * 0.05f;
        a_avg.z = a_avg.z * 0.95f + (-event.acceleration.z) * 0.05f;
    }
    vector_t<float> a = a_avg;"""
content = content.replace(heading_lsm_old, heading_lsm_new)

# 2. EMA filter for heading_icm()
heading_icm_old = """    vector_t<float> a = {accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z};"""
heading_icm_new = """    // EMA filter to remove thruster-induced vibration from the gravity vector
    static vector_t<float> a_avg_icm = {0,0,0};
    if(a_avg_icm.x == 0.0f && a_avg_icm.y == 0.0f) {
        a_avg_icm = {accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z};
    } else {
        a_avg_icm.x = a_avg_icm.x * 0.95f + accel_event.acceleration.x * 0.05f;
        a_avg_icm.y = a_avg_icm.y * 0.95f + accel_event.acceleration.y * 0.05f;
        a_avg_icm.z = a_avg_icm.z * 0.95f + accel_event.acceleration.z * 0.05f;
    }
    vector_t<float> a = a_avg_icm;"""
content = content.replace(heading_icm_old, heading_icm_new)

with open(file_path, 'w') as file:
    file.write(content)
