import re

with open("src/subwifi.cpp", "r") as f:
    content = f.read()

# Fix HTML to completely remove LSM
content = content.replace('<h2>Dual Compass Debug</h2>', '<h2>Compass Debug</h2>')
content = content.replace('<span class="lsm">LSM303: <span id="lsmVal">0.0</span>&deg;</span>', '')
content = content.replace('<span class="icm">ICM20948: <span id="icmVal">0.0</span>&deg;</span>', '<span>Heading: <span id="icmVal" class="icm">0.0</span>&deg;</span>')

# Remove the LSM Raw Box entirely
lsm_box = """    <div class="raw-box lsm">
        <div style="font-weight:bold; margin-bottom:5px;">LSM Raw (Mag)</div>
        <div class="axis-row"><div class="axis-label">X</div><div class="axis-bar-container"><div id="lsm_x_dot" class="axis-dot" style="left:50%; background:#f0ad4e;"></div></div><div id="lsm_x_val" class="axis-vals">0.0</div></div>
        <div class="axis-minmax"><span id="lsm_x_min" class="min-col">0.0</span><span id="lsm_x_max" class="max-col">0.0</span></div>
        <div class="axis-row"><div class="axis-label">Y</div><div class="axis-bar-container"><div id="lsm_y_dot" class="axis-dot" style="left:50%; background:#f0ad4e;"></div></div><div id="lsm_y_val" class="axis-vals">0.0</div></div>
        <div class="axis-minmax"><span id="lsm_y_min" class="min-col">0.0</span><span id="lsm_y_max" class="max-col">0.0</span></div>
        <div class="axis-row"><div class="axis-label">Z</div><div class="axis-bar-container"><div id="lsm_z_dot" class="axis-dot" style="left:50%; background:#f0ad4e;"></div></div><div id="lsm_z_val" class="axis-vals">0.0</div></div>
        <div class="axis-minmax"><span id="lsm_z_min" class="min-col">0.0</span><span id="lsm_z_max" class="max-col">0.0</span></div>
    </div>"""
content = content.replace(lsm_box, '')

# Make the ICM box look cleaner
content = content.replace('<div style="font-weight:bold; margin-bottom:5px;">ICM Raw (Mag)</div>', '<div style="font-weight:bold; margin-bottom:5px;">Magnetometer Raw</div>')

# Clean up JS session object and reset loop
content = content.replace("let session = {", "let session = { //", 1) # just comment it out to not mess with regex, let's do proper replace
session_old = """let session = {
    lsm: { x: {min: null, max: null}, y: {min: null, max: null}, z: {min: null, max: null} },
    icm: { x: {min: null, max: null}, y: {min: null, max: null}, z: {min: null, max: null} }
};"""
session_new = """let session = {
    icm: { x: {min: null, max: null}, y: {min: null, max: null}, z: {min: null, max: null} }
};"""
content = content.replace(session_old, session_new)

reset_old = """    session = {
        lsm: { x: {min: null, max: null}, y: {min: null, max: null}, z: {min: null, max: null} },
        icm: { x: {min: null, max: null}, y: {min: null, max: null}, z: {min: null, max: null} }
    };
    ['lsm', 'icm'].forEach(sensor => {"""
reset_new = """    session = {
        icm: { x: {min: null, max: null}, y: {min: null, max: null}, z: {min: null, max: null} }
    };
    ['icm'].forEach(sensor => {"""
content = content.replace(reset_old, reset_new)

with open("src/subwifi.cpp", "w") as f:
    f.write(content)

