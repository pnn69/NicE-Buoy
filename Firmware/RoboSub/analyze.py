import json
import math

with open('compass_data.txt', 'r') as f:
    lines = f.readlines()

data = []
for line in lines:
    try:
        data.append(json.loads(line))
    except:
        pass

if not data:
    print("No data parsed.")
    exit()

# Extract arrays of axes
l_m_x = [d['l_m'][0] for d in data]
l_m_y = [d['l_m'][1] for d in data]
l_m_z = [d['l_m'][2] for d in data]

i_m_x = [d['i_m'][0] for d in data]
i_m_y = [d['i_m'][1] for d in data]
i_m_z = [d['i_m'][2] for d in data]

# Find min/max for each to see which are changing
print("LSM ranges:")
print(f" X: {min(l_m_x):.1f} to {max(l_m_x):.1f} (span {max(l_m_x)-min(l_m_x):.1f})")
print(f" Y: {min(l_m_y):.1f} to {max(l_m_y):.1f} (span {max(l_m_y)-min(l_m_y):.1f})")
print(f" Z: {min(l_m_z):.1f} to {max(l_m_z):.1f} (span {max(l_m_z)-min(l_m_z):.1f})")

print("\nICM ranges:")
print(f" X: {min(i_m_x):.1f} to {max(i_m_x):.1f} (span {max(i_m_x)-min(i_m_x):.1f})")
print(f" Y: {min(i_m_y):.1f} to {max(i_m_y):.1f} (span {max(i_m_y)-min(i_m_y):.1f})")
print(f" Z: {min(i_m_z):.1f} to {max(i_m_z):.1f} (span {max(i_m_z)-min(i_m_z):.1f})")

# Print first few samples side by side to see correlation
print("\nFirst 10 samples (LSM X, Y, Z | ICM X, Y, Z):")
for i in range(min(10, len(data))):
    print(f"{l_m_x[i]:6.1f} {l_m_y[i]:6.1f} {l_m_z[i]:6.1f}  |  {i_m_x[i]:6.1f} {i_m_y[i]:6.1f} {i_m_z[i]:6.1f}")
