import re

with open("src/subwifi.cpp", "r") as f:
    content = f.read()

# I see it left document.getElementById('lsmVal').innerText = data.icm.toFixed(1); in JS but we removed the element!
# That will cause a JS error, breaking the update loop.

content = content.replace("document.getElementById('lsmVal').innerText = data.icm.toFixed(1);", "")

with open("src/subwifi.cpp", "w") as f:
    f.write(content)
