import os
os.system("cd C:/tmp/NicE-Buoy/Firmware/RoboTop && git checkout src/main.cpp")

with open('C:/tmp/NicE-Buoy/Firmware/RoboTop/src/main.cpp', 'r') as file:
    content = file.read()

subdata_case = """
            case SUBDATA:
                RfOut->dirMag = RfIn.dirMag;
                RfOut->speedBb = RfIn.speedBb;
                RfOut->speedSb = RfIn.speedSb;
                RfOut->ip = RfIn.ip;
                RfOut->ir = RfIn.ir;
                RfOut->subAccuV = RfIn.subAccuV;
                RfOut->subAccuP = RfIn.subAccuP;
                RfOut->subAccuI = RfIn.subAccuI;

                mainPwrData.ledBb = RfOut->speedBb;
                mainPwrData.ledSb = RfOut->speedSb;
                xQueueSend(ledPwr, (void *)&mainPwrData, 0);
                break;
            case DOCKING:"""

start_idx = content.find('void handelRfData(RoboStruct *RfOut, RoboStruct *buoyPara[3])')
end_idx = content.find('void handelSerialData(RoboStruct *ser)', start_idx)

part1 = content[:start_idx]
part2 = content[start_idx:end_idx].replace('case DOCKING:', subdata_case)
part3 = content[end_idx:]

with open('C:/tmp/NicE-Buoy/Firmware/RoboTop/src/main.cpp', 'w') as file:
    file.write(part1 + part2 + part3)
