import re

with open("src/compass.h", "r") as f:
    content = f.read()

# Remove m_min, m_max, icm_min
content = re.sub(r'extern vector_t<float> m_min;\n', '', content)
content = re.sub(r'extern vector_t<float> m_max;\n', '', content)
content = re.sub(r'extern vector_t<float> icm_min;\n', '', content)
content = re.sub(r'extern vector_t<float> icm_max;\n', '', content)

with open("src/compass.h", "w") as f:
    f.write(content)
