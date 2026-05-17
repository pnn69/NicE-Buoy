import re

with open("../../Firmware/RoboPythonDisplay/RoboControl.py", "r") as f:
    content = f.read()

# Fix parsing of incoming SETUPDATA
content = content.replace(
    '"compassOffset": comp_off, "minOfsetDist": min_off, "icmCompassOffset": icm_comp_off,',
    '"compassOffset": comp_off, "minOfsetDist": min_off,'
)
content = content.replace(
    'icm_comp_off = fields[16]',
    ''
)
# We also need to fix the indices for rev_bb and rev_sb
content = content.replace('rev_bb = fields[17]', 'rev_bb = fields[16]')
content = content.replace('rev_sb = fields[18]', 'rev_sb = fields[17]')
content = content.replace('if len(fields) >= 17:\n', '')
content = content.replace('if len(fields) >= 18:', 'if len(fields) >= 17:')
content = content.replace('if len(fields) >= 19:', 'if len(fields) >= 18:')

# Fix the dictionary where variables are stored
content = content.replace('"icmCompassOffset": icm_offset_entry', '')
content = re.sub(r',\s*"icmCompassOffset": icm_offset_entry', '', content)

# UI Elements
content = content.replace('ttk.Label(main_setup_frame, text="LSM Offset:").grid(row=18, column=0, sticky="e", padx=10, pady=2)', 'ttk.Label(main_setup_frame, text="Compass Offset:").grid(row=18, column=0, sticky="e", padx=10, pady=2)')
content = re.sub(r'\s*ttk\.Label\(main_setup_frame, text="ICM Offset:"\)\.grid\(row=19, column=0, sticky="e", padx=10, pady=2\)\n', '\n', content)
content = re.sub(r'\s*icm_offset_entry = ttk\.Entry\(main_setup_frame, width=15\)\n', '\n', content)
content = re.sub(r'\s*icm_offset_entry\.grid\(row=19, column=1, sticky="w", pady=2\)\n', '\n', content)

# send_speed_limits_and_motors
content = re.sub(r'\s*icmoff = float\(icm_offset_entry\.get\(\) or 0\)\n', '\n', content)
content = content.replace('val_str = f"{kpr},{kir},{kdr},{kps},{kis},{kds},{max_s},{min_s},{piv_s},{coff},{min_off},{icmoff},{rev_bb_int},{rev_sb_int}"',
                          'val_str = f"{kpr},{kir},{kdr},{kps},{kis},{kds},{max_s},{min_s},{piv_s},{coff},{min_off},{rev_bb_int},{rev_sb_int}"')

# send_compass_offset
content = content.replace('val_str = f"{format(coff, \'.2f\')},{format(icmoff, \'.2f\')}"',
                          'val_str = f"{format(coff, \'.2f\')}"')

# Remove setting icm_offset_entry from 'data'
content = re.sub(r'\s*icm_offset_entry\.delete\(0, tk\.END\)\n', '\n', content)
content = re.sub(r'\s*icm_offset_entry\.insert\(0, data\.get\("icmCompassOffset", ""\)\)\n', '\n', content)


with open("../../Firmware/RoboPythonDisplay/RoboControl.py", "w") as f:
    f.write(content)
