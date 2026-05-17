import re
import os

top_html_path = "../../Firmware/RoboTop/data/index.html"

with open(top_html_path, "r") as f:
    content = f.read()

# Fix the inputs
content = content.replace('LSM Offset <input id="compassOffset"', 'Compass Offset <input id="compassOffset"')
content = re.sub(r'\s*<div class="inputRow" style="margin-top: 0;">ICM Offset <input\s+id="icmCompassOffset" type="number" step="0.1"></div>\n', '\n', content)

# Fix JS parser (where it fills the input)
content = re.sub(r'\s*"icmCompassOffset",', ',', content)
content = re.sub(r'\s*setVal\("icmCompassOffset", b\.icmCompassOffset\);\n', '\n', content)

# Fix JS sender
content = content.replace('&icmoffset=${document.getElementById(\'icmCompassOffset\').value || 0}', '')

with open(top_html_path, "w") as f:
    f.write(content)
