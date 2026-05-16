import re
with open('C:/tmp/NicE-Buoy/Firmware/RoboTop/src/main.cpp', 'r') as file:
    content = file.read()

subdata_case = """
            case SUBDATA:
                // Update Top unit telemetry from UDP stream!
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

# Revert my bad injection first!
content = re.sub(r'case SUBDATA:.*break;\s*case DOCKING:', 'case DOCKING:', content, flags=re.DOTALL)
content = content.replace('case DOCKING:', subdata_case)

with open('C:/tmp/NicE-Buoy/Firmware/RoboTop/src/main.cpp', 'w') as file:
    file.write(content)
