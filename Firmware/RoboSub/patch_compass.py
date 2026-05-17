import re

with open("src/compass.cpp", "r") as f:
    content = f.read()

# Remove m_max, m_min
content = re.sub(r'vector_t<float> m_max;\nvector_t<float> m_min;\n', '', content)

# In InitCompass, remove m_min/m_max logic
content = re.sub(r'    if \(abs\(max_mag\[0\] - min_mag\[0\]\) < 0\.01\) \{\n        m_min = \(vector_t<float>\)\{-50, -50, -50\};\n        m_max = \(vector_t<float>\)\{50, 50, 50\};\n    \} else \{\n        m_min = \(vector_t<float>\)\{min_mag\[0\], min_mag\[1\], min_mag\[2\]\};\n        m_max = \(vector_t<float>\)\{max_mag\[0\], max_mag\[1\], max_mag\[2\]\};\n    \}\n', '', content)

# In CompassTask, replace calculation with atan2f(mz, my) and circular average
old_calc = """            float cal_x = mx - mainData.icmMagHard[0];
            float cal_y = my - mainData.icmMagHard[1];
            float cal_z = mz - mainData.icmMagHard[2];

            float final_x = mainData.icmMagSoft[0][0] * cal_x + mainData.icmMagSoft[0][1] * cal_y + mainData.icmMagSoft[0][2] * cal_z;
            float final_y = mainData.icmMagSoft[1][0] * cal_x + mainData.icmMagSoft[1][1] * cal_y + mainData.icmMagSoft[1][2] * cal_z;
            float final_z = mainData.icmMagSoft[2][0] * cal_x + mainData.icmMagSoft[2][1] * cal_y + mainData.icmMagSoft[2][2] * cal_z;

            // Proven axis mapping: atan2f(final_z, final_y)
            float current_h_rad = atan2f(final_z, final_y);"""

new_calc = """            float current_h_rad = atan2f(mz, my);"""
content = content.replace(old_calc, new_calc)

with open("src/compass.cpp", "w") as f:
    f.write(content)
