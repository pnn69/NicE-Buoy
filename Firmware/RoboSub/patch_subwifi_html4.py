import re

with open("src/subwifi.cpp", "r") as f:
    content = f.read()

# Completely clean out the dead LSM JS variables/functions
content = content.replace("    lsm: { x: {min: null, max: null}, y: {min: null, max: null}, z: {min: null, max: null} },\n", "")
content = content.replace(".lsm { color: #f0ad4e; font-weight: bold; }\n", "")
content = content.replace("        // lsm removed\n", "")

old_rose = """function drawRose(lsm, icm) {
    ctx.clearRect(0, 0, 400, 400);

    // Draw background
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, 2*Math.PI); ctx.strokeStyle = '#555'; ctx.lineWidth = 2; ctx.stroke();

    // Draw N, E, S, W
    ctx.fillStyle = '#aaa'; ctx.font = 'bold 20px Arial'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText('N', cx, cy - r + 20); ctx.fillText('S', cx, cy + r - 20);
    ctx.fillText('E', cx + r - 20, cy); ctx.fillText('W', cx - r + 20, cy);

    // Draw LSM Vector (Orange)
    drawVector(lsm, '#f0ad4e', r - 30, 4);

    // Draw ICM Vector (Blue)
    drawVector(icm, '#5bc0de', r - 40, 4);
}"""

new_rose = """function drawRose(icm) {
    ctx.clearRect(0, 0, 400, 400);

    // Draw background
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, 2*Math.PI); ctx.strokeStyle = '#555'; ctx.lineWidth = 2; ctx.stroke();

    // Draw N, E, S, W
    ctx.fillStyle = '#aaa'; ctx.font = 'bold 20px Arial'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText('N', cx, cy - r + 20); ctx.fillText('S', cx, cy + r - 20);
    ctx.fillText('E', cx + r - 20, cy); ctx.fillText('W', cx - r + 20, cy);

    // Draw ICM Vector (Blue)
    drawVector(icm, '#5bc0de', r - 40, 4);
}"""

content = content.replace(old_rose, new_rose)
content = content.replace("drawRose(0, data.icm);", "drawRose(data.icm);")


with open("src/subwifi.cpp", "w") as f:
    f.write(content)
