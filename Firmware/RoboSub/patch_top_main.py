import re
with open('C:/tmp/NicE-Buoy/Firmware/RoboTop/src/main.cpp', 'r') as file:
    content = file.read()

subdata_case = """
            case SUBDATA:
                // Update Top unit telemetry from UDP stream!
                stat->dirMag = RfIn.dirMag;
                stat->speedBb = RfIn.speedBb;
                stat->speedSb = RfIn.speedSb;
                stat->ip = RfIn.ip;
                stat->ir = RfIn.ir;
                stat->subAccuV = RfIn.subAccuV;
                stat->subAccuP = RfIn.subAccuP;
                stat->subAccuI = RfIn.subAccuI;

                mainPwrData.ledBb = stat->speedBb;
                mainPwrData.ledSb = stat->speedSb;
                xQueueSend(ledPwr, (void *)&mainPwrData, 0);
                break;
            case DOCKING:"""

content = content.replace('case DOCKING:', subdata_case)
with open('C:/tmp/NicE-Buoy/Firmware/RoboTop/src/main.cpp', 'w') as file:
    file.write(content)
