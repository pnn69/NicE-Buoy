import re

with open('C:/tmp/NicE-Buoy/Firmware/RoboSub/src/compass.cpp', 'r') as file:
    content = file.read()

# --- FIX LSM ---

# Remove old mapping and add EMA filter for LSM
heading_lsm_old = """    vector_t<float> temp_m = {-event.magnetic.y, event.magnetic.x, -event.magnetic.z};

    if (!accel.getEvent(&event)) return 0.0f;
    m_lsm_a_last = event;
    vector_t<float> a = {-event.acceleration.y, event.acceleration.x, -event.acceleration.z};"""

heading_lsm_new = """    vector_t<float> temp_m = {event.magnetic.x, event.magnetic.y, event.magnetic.z};

    if (!accel.getEvent(&event)) return 0.0f;
    m_lsm_a_last = event;
    
    // EMA filter to remove thruster-induced vibration from the gravity vector
    static vector_t<float> a_avg = {0,0,0};
    if(a_avg.y == 0.0f) {
        a_avg = {event.acceleration.x, event.acceleration.y, event.acceleration.z};
    } else {
        a_avg.x = a_avg.x * 0.95f + event.acceleration.x * 0.05f;
        a_avg.y = a_avg.y * 0.95f + event.acceleration.y * 0.05f;
        a_avg.z = a_avg.z * 0.95f + event.acceleration.z * 0.05f;
    }
    vector_t<float> a = a_avg;"""
content = content.replace(heading_lsm_old, heading_lsm_new)


# --- FIX ICM ---

# Remove manual swapping for ICM in calibration
calib_icm_old = """        // Map ICM magnetometer to LSM frame: X = Y, Y = -X, Z = -Z
        float temp_mag_x = event_icm.magnetic.x;
        event_icm.magnetic.x = event_icm.magnetic.y;
        event_icm.magnetic.y = -temp_mag_x;
        event_icm.magnetic.z = -event_icm.magnetic.z;"""
content = content.replace(calib_icm_old, "")

# Remove manual swapping and add EMA filter for ICM in heading
heading_icm_old = """    // Map ICM magnetometer to LSM frame: X = Y, Y = -X, Z = -Z
    float temp_mag_x = mag_event.magnetic.x;
    mag_event.magnetic.x = mag_event.magnetic.y;
    mag_event.magnetic.y = -temp_mag_x;
    mag_event.magnetic.z = -mag_event.magnetic.z;
    m_icm_last = mag_event;

    // Map ICM accelerometer to LSM frame: X = Y, Y = -X, Z = Z
    float temp_accel_x = accel_event.acceleration.x;
    accel_event.acceleration.x = accel_event.acceleration.y;
    accel_event.acceleration.y = -temp_accel_x;
    // Z stays the same
    m_icm_a_last = accel_event;

    vector_t<float> temp_m = {mag_event.magnetic.x, mag_event.magnetic.y, mag_event.magnetic.z};
    vector_t<float> a = {accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z};"""

heading_icm_new = """    m_icm_last = mag_event;
    m_icm_a_last = accel_event;

    vector_t<float> temp_m = {mag_event.magnetic.x, mag_event.magnetic.y, mag_event.magnetic.z};
    
    // EMA filter to remove thruster-induced vibration from the gravity vector
    static vector_t<float> a_avg_icm = {0,0,0};
    if(a_avg_icm.y == 0.0f) {
        a_avg_icm = {accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z};
    } else {
        a_avg_icm.x = a_avg_icm.x * 0.95f + accel_event.acceleration.x * 0.05f;
        a_avg_icm.y = a_avg_icm.y * 0.95f + accel_event.acceleration.y * 0.05f;
        a_avg_icm.z = a_avg_icm.z * 0.95f + accel_event.acceleration.z * 0.05f;
    }
    vector_t<float> a = a_avg_icm;"""
content = content.replace(heading_icm_old, heading_icm_new)


# --- CHANGE FORWARD VECTOR TO X-AXIS ---

get_heading_old = """    if (lsm_ready) {
        mHeding = heading((vector_t<int>){0, 1, 0});
        mHeding = mHeding + mainData.compassOffset - 90.0f;
    } else {
        mHeding = heading_icm((vector_t<int>){0, 1, 0});
        mHeding = mHeding + mainData.icmCompassOffset - 90.0f;
    }"""
get_heading_new = """    if (lsm_ready) {
        mHeding = heading((vector_t<int>){1, 0, 0});
        mHeding = mHeding + mainData.compassOffset;
    } else {
        mHeding = heading_icm((vector_t<int>){1, 0, 0});
        mHeding = mHeding + mainData.icmCompassOffset;
    }"""
content = content.replace(get_heading_old, get_heading_new)

global_lsm_old = """        if (lsm_ready) {
            global_lsmHdg = heading((vector_t<int>){0, 1, 0});
            global_lsmHdg += mainData.compassOffset - 90.0f;
            if (!isnan(global_lsmHdg)) {
                global_lsmHdg = fmod(global_lsmHdg, 360.0);
                if (global_lsmHdg < 0) global_lsmHdg += 360.0;
            }
        } else {
            global_lsmHdg = 0.0;
        }

        if (icm_ready) {
            global_icmHdg = heading_icm((vector_t<int>){0, 1, 0});
            global_icmHdg += mainData.icmCompassOffset - 90.0f;"""
global_lsm_new = """        if (lsm_ready) {
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
content = content.replace(global_lsm_old, global_lsm_new)


with open('C:/tmp/NicE-Buoy/Firmware/RoboSub/src/compass.cpp', 'w') as file:
    file.write(content)
