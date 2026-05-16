import re

with open('C:/tmp/NicE-Buoy/Firmware/RoboDependency/RoboCompute/src/RoboCompute.cpp', 'r') as file:
    content = file.read()

new_rfDeCode = """void rfDeCode(String rfIn, RoboStruct *in)
{
    rfIn.trim();
    in->IDr = -1;
    in->IDs = -1;

    if (!rfIn.startsWith("$") || rfIn.indexOf('*') == -1) return;
    if (!verifyCRC(rfIn)) return;

    int starIndex = rfIn.indexOf('*');
    rfIn = rfIn.substring(1, starIndex);

    int comma1 = rfIn.indexOf(',');
    if (comma1 == -1) return;
    in->IDr = strtoull(rfIn.substring(0, comma1).c_str(), NULL, 16);

    rfIn = rfIn.substring(comma1 + 1);
    int comma2 = rfIn.indexOf(',');
    if (comma2 == -1) return;
    in->IDs = strtoull(rfIn.substring(0, comma2).c_str(), NULL, 16);

    rfIn = rfIn.substring(comma2 + 1);
    int comma3 = rfIn.indexOf(',');
    if (comma3 == -1) return;
    in->ack = rfIn.substring(0, comma3).toInt();

    rfIn = rfIn.substring(comma3 + 1);
    RoboDecode(rfIn, in);
}"""

content = re.sub(r'void rfDeCode\(String rfIn, RoboStruct \*in\).*?^}', new_rfDeCode, content, flags=re.DOTALL | re.MULTILINE)

with open('C:/tmp/NicE-Buoy/Firmware/RoboDependency/RoboCompute/src/RoboCompute.cpp', 'w') as file:
    file.write(content)
