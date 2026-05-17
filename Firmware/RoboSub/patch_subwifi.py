import re

with open("src/subwifi.cpp", "r") as f:
    content = f.read()

# Remove externs
content = re.sub(r'extern float global_lsmHdg;\n', '', content)
content = re.sub(r'extern sensors_event_t m_lsm_last, m_icm_last;\n', '', content)
content = re.sub(r'extern sensors_event_t m_lsm_a_last, m_icm_a_last;\n', '', content)

# Clean up JSON route
old_json_route = """    subServer.on("/data", []() {
        String json = "{";
        json += "\\"lsm\\":" + String(global_lsmHdg, 2) + ", ";
        json += "\\"icm\\":" + String(global_icmHdg, 2) + ", ";
        json += "\\"lsm_x\\":" + String(m_lsm_last.magnetic.x, 2) + ", ";
        json += "\\"lsm_y\\":" + String(m_lsm_last.magnetic.y, 2) + ", ";
        json += "\\"lsm_z\\":" + String(m_lsm_last.magnetic.z, 2) + ", ";
        json += "\\"icm_x\\":" + String(m_icm_last.magnetic.x, 2) + ", ";
        json += "\\"icm_y\\":" + String(m_icm_last.magnetic.y, 2) + ", ";
        json += "\\"icm_z\\":" + String(m_icm_last.magnetic.z, 2) + ", ";
        json += "\\"lsm_min\\":{\\"x\\":" + String(m_min.x, 1) + ",\\"y\\":" + String(m_min.y, 1) + ",\\"z\\":" + String(m_min.z, 1) + "}, ";
        json += "\\"lsm_max\\":{\\"x\\":" + String(m_max.x, 1) + ",\\"y\\":" + String(m_max.y, 1) + ",\\"z\\":" + String(m_max.z, 1) + "}, ";
        json += "\\"icm_min\\":{\\"x\\":" + String(icm_min.x, 1) + ",\\"y\\":" + String(icm_min.y, 1) + ",\\"z\\":" + String(icm_min.z, 1) + "}, ";
        json += "\\"icm_max\\":{\\"x\\":" + String(icm_max.x, 1) + ",\\"y\\":" + String(icm_max.y, 1) + ",\\"z\\":" + String(icm_max.z, 1) + "}, ";
        json += "\\"speed_bb\\":" + String(global_speed_bb, 0) + ", ";
        json += "\\"speed_sb\\":" + String(global_speed_sb, 0) + ", ";
        json += "\\"a_x\\":" + String(m_icm_a_last.acceleration.x, 2) + ", ";
        json += "\\"a_y\\":" + String(m_icm_a_last.acceleration.y, 2) + ", ";
        json += "\\"a_z\\":" + String(m_icm_a_last.acceleration.z, 2);
        json += "}";
        subServer.send(200, "application/json", json);
    });"""

new_json_route = """    subServer.on("/data", []() {
        extern float last_raw_x, last_raw_y, last_raw_z;
        extern vector_t<float> icm_min, icm_max;
        String json = "{";
        json += "\\"icm\\":" + String(global_icmHdg, 2) + ", ";
        json += "\\"icm_x\\":" + String(last_raw_x, 2) + ", ";
        json += "\\"icm_y\\":" + String(last_raw_y, 2) + ", ";
        json += "\\"icm_z\\":" + String(last_raw_z, 2) + ", ";
        json += "\\"icm_min\\":{\\"x\\":" + String(icm_min.x, 1) + ",\\"y\\":" + String(icm_min.y, 1) + ",\\"z\\":" + String(icm_min.z, 1) + "}, ";
        json += "\\"icm_max\\":{\\"x\\":" + String(icm_max.x, 1) + ",\\"y\\":" + String(icm_max.y, 1) + ",\\"z\\":" + String(icm_max.z, 1) + "}, ";
        json += "\\"speed_bb\\":" + String(global_speed_bb, 0) + ", ";
        json += "\\"speed_sb\\":" + String(global_speed_sb, 0) + ", ";
        json += "\\"a_x\\":0, \\"a_y\\":0, \\"a_z\\":0";
        json += "}";
        subServer.send(200, "application/json", json);
    });"""
content = content.replace(old_json_route, new_json_route)

with open("src/subwifi.cpp", "w") as f:
    f.write(content)
