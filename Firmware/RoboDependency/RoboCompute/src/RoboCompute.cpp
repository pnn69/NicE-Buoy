#include <arduino.h>
#include "RoboCompute.h"

bool startsWithDollar(const String &str)
{
    return str.charAt(0) == '$';
}

String formatFloat(double val, int precision)
{
    if (val == 0.0) return "0";
    String s = String(val, precision);
    if (s.indexOf('.') != -1) {
        while (s.endsWith("0")) {
            s.remove(s.length() - 1);
        }
        if (s.endsWith(".")) {
            s.remove(s.length() - 1);
        }
    }
    return s;
}

void RoboDecode(String data, RoboStruct *dataStore)
{
    dataStore->cmd = -1;
    String numbers[25];
    int count = 0;
    String substring = data;
    while (count < 25)
    {
        int commaIndex = substring.indexOf(',');
        if (commaIndex == -1) { numbers[count++] = substring; break; }
        numbers[count++] = substring.substring(0, commaIndex);
        substring = substring.substring(commaIndex + 1);
    }
    if (count < 2) return;
    dataStore->cmd = numbers[0].toInt();
    dataStore->status = numbers[1].toInt();
    switch (dataStore->cmd)
    {
    case SETUPDATA:
          dataStore->Kpr = numbers[2].toDouble();
          dataStore->Kir = numbers[3].toDouble();
          dataStore->Kdr = numbers[4].toDouble();
          dataStore->Kps = numbers[5].toDouble();
          dataStore->Kis = numbers[6].toDouble();
          dataStore->Kds = numbers[7].toDouble();
          dataStore->maxSpeed = numbers[8].toInt();
          dataStore->minSpeed = numbers[9].toInt();
          dataStore->pivotSpeed = numbers[10].toDouble();
          dataStore->compassOffset = numbers[11].toDouble();
          dataStore->holdRad = numbers[12].toDouble();
          // Presence is decided by the field COUNT, not by whether the token is empty.
          // RoboCode() compresses any all-zero token to "" (see the note in the ADAPTIVE_TRIM
          // case), so an empty field here means the value is zero - testing .length() would make
          // it impossible to ever clear a flag or set a distance back to 0. A frame that is
          // simply shorter, from an older node or from the CYD's GET query, leaves the current
          // values untouched.
          if (count > 13) dataStore->revBB = (bool)numbers[13].toInt();
          if (count > 14) dataStore->revSB = (bool)numbers[14].toInt();
          if (count > 15) dataStore->swap_BB_SB = (bool)numbers[15].toInt();
          // The CYD has always put these four on the wire (send_buoy_setup() in
          // RoboCYD/src/buoy_data.cpp) and reads them back at the same offsets, but this decoder
          // stopped at swap_BB_SB, so they arrived as whatever the receive struct happened to
          // hold - zero. RoboTop then committed that zero to its NVM on a CYD "SAVE SETUP"
          // (handleRfData case SETUPDATA), which is how the dock approach settings silently
          // reset themselves to 0/0/false.
          if (count > 16) dataStore->compass_trim_enabled = (bool)numbers[16].toInt();
          if (count > 17) dataStore->dockApproachDist = numbers[17].toInt();
          if (count > 18) dataStore->dockApproachDir = numbers[18].toInt();
          if (count > 19) dataStore->dockingToWaypoint = (bool)numbers[19].toInt();
          break;
    case IDLE:
        dataStore->speed = 0;
        dataStore->tgDist = 0;
        break;
    case DOCKED:
    case LOCKED:
        dataStore->tgDir = numbers[2].toDouble();
        dataStore->tgDist = numbers[3].toDouble();
        dataStore->tgSpeed = numbers[4].toDouble();
        dataStore->wDir = numbers[5].toDouble();
        dataStore->wStd = numbers[6].toDouble();
        break;
    case REMOTE:
        dataStore->tgDir = numbers[2].toDouble();
        dataStore->tgSpeed = numbers[3].toDouble();
        break;
    case DIRSPEED:
        dataStore->dirMag = numbers[2].toDouble();
        dataStore->speed = numbers[3].toInt();
        dataStore->speedBb = numbers[4].toInt();
        dataStore->speedSb = numbers[5].toInt();
        break;
    case TGDIRSPEED:
        dataStore->tgDir = numbers[2].toDouble();
        dataStore->speedSet = numbers[3].toDouble();
        break;
    case SPBBSPSB:
        dataStore->speedBb = numbers[2].toInt();
        dataStore->speedSb = numbers[3].toInt();
        break;
    case CALCRUDDER:
        dataStore->tgDir = numbers[2].toDouble();
        dataStore->tgDist = numbers[3].toDouble();
        dataStore->speedSet = numbers[4].toDouble();
        break;
    case SPEED:
        dataStore->speed = numbers[2].toInt();
        break;
    case SUBSPEED:
        dataStore->speedBb = numbers[2].toInt();
        dataStore->speedSb = numbers[3].toInt();
        dataStore->speed = numbers[4].toInt();
        break;
    case SUBACCU:
        dataStore->subAccuV = numbers[2].toFloat();
        dataStore->subAccuP = numbers[3].toInt();
        if (count > 4) dataStore->subAccuI = numbers[4].toFloat();
        break;
    case PIDRUDDERSET:
    case PIDRUDDER:
        dataStore->Kpr = numbers[2].toDouble();
        dataStore->Kir = numbers[3].toDouble();
        dataStore->Kdr = numbers[4].toDouble();
        break;
    case PIDSPEEDSET:
    case PIDSPEED:
        dataStore->Kps = numbers[2].toDouble();
        dataStore->Kis = numbers[3].toDouble();
        dataStore->Kds = numbers[4].toDouble();
        break;
    case SUBPWR:
        dataStore->speedSet = numbers[2].toDouble();
        dataStore->speed = numbers[3].toInt();
        dataStore->speedBb = numbers[4].toInt();
        dataStore->speedSb = numbers[5].toInt();
        dataStore->subAccuV = numbers[6].toFloat();
        if (count > 7) dataStore->subAccuI = numbers[7].toFloat();
        break;
    case TOPPWR:
        dataStore->speedSet = numbers[2].toDouble();
        dataStore->speed = numbers[3].toInt();
        dataStore->speedBb = numbers[4].toInt();
        dataStore->speedSb = numbers[5].toInt();
        dataStore->topAccuV = numbers[6].toFloat();
        if (count > 7) dataStore->topAccuI = numbers[7].toFloat();
        break;
    case BUOYPOS:
        dataStore->lat = numbers[2].toDouble();
        dataStore->lng = numbers[3].toDouble();
        dataStore->dirMag = numbers[4].toDouble();
        dataStore->wDir = numbers[5].toDouble();
        dataStore->wStd = numbers[6].toDouble();
        dataStore->topAccuP = numbers[7].toInt();
        dataStore->subAccuP = numbers[8].toInt();
        dataStore->gpsFix = (bool)numbers[9].toInt();
        dataStore->gpsSat = numbers[10].toInt();
        break;
    case LOCKPOS:
    case DOCKPOS:
        dataStore->tgLat = numbers[2].toDouble();
        dataStore->tgLng = numbers[3].toDouble();
        dataStore->wDir = numbers[4].toDouble();
        dataStore->wStd = numbers[5].toDouble();
        break;
    case SETLOCKPOS:
    case SETDOCKPOS:
        dataStore->tgLat = numbers[2].toDouble();
        dataStore->tgLng = numbers[3].toDouble();
        break;
    case DIRDIST:
        dataStore->tgDir = numbers[2].toDouble();
        dataStore->tgDist = numbers[3].toDouble();
        break;
    case WINDDATA:
        dataStore->wDir = numbers[2].toDouble();
        dataStore->wStd = numbers[3].toDouble();
        break;
    case STORE_DECLINATION:
        // Retired, see RoboCompute.h. The label is kept so an old node's frame is recognised and
        // ignored quietly instead of tripping the "Unknown CMD" printf below.
        break;
    case MAXMINPWR:
    case MAXMINPWRSET:
        dataStore->maxSpeed = numbers[2].toInt();
        dataStore->minSpeed = numbers[3].toInt();
        if (count > 4) dataStore->pivotSpeed = numbers[4].toDouble();
        break;
    case DIRMDIRTGDIRG:
        dataStore->dirMag = numbers[2].toDouble();
        dataStore->tgDir = numbers[3].toDouble();
        dataStore->gpsDir = numbers[4].toInt();
        break;
    case SET_DECLINATION:
        dataStore->status = SET_DECLINATION;
        break;
    case SUBDATA:
        dataStore->dirMag = numbers[2].toDouble();
        dataStore->speedBb = numbers[3].toInt();
        dataStore->speedSb = numbers[4].toInt();
        dataStore->ip = numbers[5].toDouble();
        dataStore->ir = numbers[6].toDouble();
        dataStore->subAccuV = numbers[7].toDouble();
        dataStore->subAccuP = numbers[8].toInt();
        if (count > 9) dataStore->subAccuI = numbers[9].toFloat();
        // Iron corrected heading, see RoboStruct::imag. Count guarded: an older Sub sends a
        // shorter frame and the Top keeps whatever it already had.
        if (count > 10) dataStore->imag = numbers[10].toDouble();
        // Attitude. Count guarded like imag above: an older Sub sends a shorter frame and the Top
        // keeps whatever it had rather than reading a missing field as dead level.
        if (count > 12)
        {
            dataStore->pitch = numbers[11].toDouble();
            dataStore->roll = numbers[12].toDouble();
        }
        break;
    case TOPDATA:
        dataStore->dirMag = numbers[2].toDouble();
        dataStore->gpsDir = numbers[3].toInt();
        dataStore->tgDir = numbers[4].toInt();
        dataStore->tgDist = numbers[5].toDouble();
        dataStore->wDir = numbers[6].toDouble();
        dataStore->wStd = numbers[7].toDouble();
        dataStore->speedBb = numbers[8].toInt();
        dataStore->speedSb = numbers[9].toInt();
        dataStore->ip = numbers[10].toDouble();
        dataStore->ir = numbers[11].toDouble();
        dataStore->subAccuV = numbers[12].toDouble();
        dataStore->subAccuP = numbers[13].toInt();
        dataStore->lat = numbers[14].toDouble();
        dataStore->lng = numbers[15].toDouble();
        dataStore->gpsFix = (bool)numbers[16].toInt();
        dataStore->gpsSat = numbers[17].toInt();
        if (count > 18) dataStore->subAccuI = numbers[18].toFloat();
        // Iron corrected heading, see RoboStruct::imag.
        if (count > 19) dataStore->imag = numbers[19].toDouble();
        break;
    case RAWCOMPASSDATA:
        dataStore->magHard[0] = numbers[2].toDouble();
        dataStore->magHard[1] = numbers[3].toDouble();
        dataStore->magHard[2] = numbers[4].toDouble();
        break;
    case HARDIRONFACTORS:
        dataStore->magHard[0] = numbers[2].toDouble();
        dataStore->magHard[1] = numbers[3].toDouble();
        dataStore->magHard[2] = numbers[4].toDouble();
        break;
    case SOFTIRONFACTORS:
        dataStore->magSoft[0][0] = numbers[2].toDouble();
        dataStore->magSoft[0][1] = numbers[3].toDouble();
        dataStore->magSoft[0][2] = numbers[4].toDouble();
        dataStore->magSoft[1][0] = numbers[5].toDouble();
        dataStore->magSoft[1][1] = numbers[6].toDouble();
        dataStore->magSoft[1][2] = numbers[7].toDouble();
        dataStore->magSoft[2][0] = numbers[8].toDouble();
        dataStore->magSoft[2][1] = numbers[9].toDouble();
        dataStore->magSoft[2][2] = numbers[10].toDouble();
        break;
    case STORE_COMPASS_OFFSET:
        dataStore->compassOffset = numbers[2].toDouble();
        break;
    case ADAPTIVE_TRIM:
        // Both fields are assigned unconditionally: RoboCode() compresses any all-zero token to
        // an empty string, so "trim disabled" and "trim 0.0" arrive as "". Treating an empty
        // field as "keep previous value" here would make it impossible to ever clear the trim.
        dataStore->compass_trim = numbers[2].toDouble();
        dataStore->compass_trim_enabled = (bool)numbers[3].toInt();
        break;
    case SET_AS_NORTH:
    case SET_AS_LEVEL:
        break;
    case ATTITUDE:
        if (count > 3)
        {
            dataStore->pitch = numbers[2].toDouble();
            dataStore->roll = numbers[3].toDouble();
        }
        break;
    case LORA_LINK:
        if (count > 2) dataStore->linkOmitted = (uint8_t)numbers[2].toInt();
        dataStore->linkPeers = 0;
        // Walk the triples until they run out or the cap is reached. An empty id ends the list:
        // RoboCode() compresses an all-zero token to "", so a trailing empty field is padding
        // rather than a peer whose id happens to be zero.
        for (int i = 3; i + 2 < count && dataStore->linkPeers < LORA_LINK_MAX_PEERS; i += 3)
        {
            if (numbers[i].length() == 0) break;
            int p = dataStore->linkPeers;
            dataStore->linkPeerId[p] = (uint32_t)strtoul(numbers[i].c_str(), NULL, 16);
            dataStore->linkRssi[p] = (int16_t)numbers[i + 1].toInt();
            dataStore->linkCount[p] = (uint16_t)numbers[i + 2].toInt();
            dataStore->linkPeers++;
        }
        break;
    case CAL8_SESSION:
        // Count-guarded like everything else here. A short frame is a node that predates the
        // field, not an empty session, and reading it as one would make a running calibration
        // look cancelled on every screen watching it.
        if (count > 4)
        {
            dataStore->cal8Action = numbers[2].toInt();
            dataStore->cal8Active = (bool)numbers[3].toInt();
            dataStore->cal8Next = numbers[4].toInt();
        }
        if (count > 12)
        {
            for (int i = 0; i < 8; i++)
                dataStore->cal8Captured[i] = numbers[5 + i].toFloat();
        }
        // The captured mask and the press serial, both count-guarded. A node that predates them
        // sends a shorter frame; the mask then stays 0 and a reader falls back to treating
        // cal8Next as the progress, which is what the guided cursor alone used to mean.
        if (count > 13) dataStore->cal8Mask = (uint8_t)numbers[13].toInt();
        if (count > 14) dataStore->cal8Seq = (uint16_t)numbers[14].toInt();
        break;
    case STORE_INTERPOLATION_TABLE:
        // Presence is decided by the field COUNT, exactly as in SETUPDATA: RoboCode() compresses
        // an all-zero token to "", so an empty field here means 0.0 and not "absent". A GET
        // request that carries no table at all must leave the receiver's own copy untouched.
        if (count > 9)
        {
            for (int i = 0; i < 8; i++)
                dataStore->interpolationTable[i] = numbers[2 + i].toFloat();
        }
        // Trailing flag: whether this table can actually be USED. An out-of-order table, or one
        // measured under the old convention, is stored and then ignored - so "the buoy has a
        // table" and "the buoy is corrected" are different questions, and this answers the second.
        // The Sub reports the identity table when this is false, which a reader could not
        // otherwise tell apart from a genuinely uncalibrated buoy.
        if (count > 10) dataStore->interpUsable = (bool)numbers[10].toInt();
        break;
    default:
        printf("RoboDecode: Unknown CMD %d\r\n", dataStore->cmd);
        break;
    }
}

String RoboCode(const RoboStruct *dataOut)
{
    String out = String(dataOut->cmd);
    out += "," + String(dataOut->status);
    if (dataOut->ack == ACK) return out;
    switch (dataOut->cmd)
    {
    case SETUPDATA:
        out += "," + formatFloat(dataOut->Kpr, 5);
        out += "," + formatFloat(dataOut->Kir, 5);
        out += "," + formatFloat(dataOut->Kdr, 5);
        out += "," + formatFloat(dataOut->Kps, 5);
        out += "," + formatFloat(dataOut->Kis, 5);
        out += "," + formatFloat(dataOut->Kds, 5);
        out += "," + String(dataOut->maxSpeed);
        out += "," + String(dataOut->minSpeed);
        out += "," + formatFloat(dataOut->pivotSpeed, 2);
        out += "," + formatFloat(dataOut->compassOffset, 2);
        out += "," + formatFloat(dataOut->holdRad, 2);
        out += "," + String((int)dataOut->revBB);
        out += "," + String((int)dataOut->revSB);
        out += "," + String((int)dataOut->swap_BB_SB);
        // Same four fields the CYD sends, in the same order it parses them back
        // (fields[19..22] there, numbers[16..19] here). Without them a buoy's SETUPDATA reply
        // could never tell anyone what its dock approach actually is.
        out += "," + String((int)dataOut->compass_trim_enabled);
        out += "," + String(dataOut->dockApproachDist);
        out += "," + String(dataOut->dockApproachDir);
        out += "," + String((int)dataOut->dockingToWaypoint);
        break;
    case DIRSPEED:
        out += "," + formatFloat(dataOut->dirMag, 2);
        out += "," + String(dataOut->speed);
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        break;
    case SPEED:
        out += "," + String(dataOut->speed);
        break;
    case SUBSPEED:
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        out += "," + String(dataOut->speed);
        break;
    case SUBACCU:
        out += "," + formatFloat(dataOut->subAccuV, 2);
        out += "," + String(dataOut->subAccuP);
        out += "," + formatFloat(dataOut->subAccuI, 2);
        break;
    case PIDRUDDERSET:
    case PIDRUDDER:
        out += "," + formatFloat(dataOut->Kpr, 5);
        out += "," + formatFloat(dataOut->Kir, 5);
        out += "," + formatFloat(dataOut->Kdr, 5);
        break;
    case PIDSPEEDSET:
    case PIDSPEED:
        out += "," + formatFloat(dataOut->Kps, 5);
        out += "," + formatFloat(dataOut->Kis, 5);
        out += "," + formatFloat(dataOut->Kds, 5);
        break;
    case SUBPWR:
        out += "," + formatFloat(dataOut->speedSet, 2);
        out += "," + String(dataOut->speed);
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        out += "," + formatFloat(dataOut->subAccuV, 2);
        out += "," + formatFloat(dataOut->subAccuI, 2);
        break;
    case TOPPWR:
        out += "," + formatFloat(dataOut->speedSet, 2);
        out += "," + String(dataOut->speed);
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        out += "," + formatFloat(dataOut->topAccuV, 2);
        out += "," + formatFloat(dataOut->topAccuI, 2);
        break;
    case BUOYPOS:
        out += "," + formatFloat(dataOut->lat, 8);
        out += "," + formatFloat(dataOut->lng, 8);
        out += "," + formatFloat(dataOut->dirMag, 2);
        out += "," + formatFloat(dataOut->wDir, 1);
        out += "," + formatFloat(dataOut->wStd, 1);
        out += "," + String(dataOut->topAccuP);
        out += "," + String(dataOut->subAccuP);
        out += "," + String(dataOut->gpsFix);
        out += "," + String(dataOut->gpsSat);
        break;
    case TGDIRSPEED:
        out += "," + formatFloat(dataOut->tgDir, 2);
        out += "," + formatFloat(dataOut->speedSet, 2);
        break;
    case SUBDATA:
        out += "," + formatFloat(dataOut->dirMag, 2);
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        out += "," + formatFloat(dataOut->ip, 2);
        out += "," + formatFloat(dataOut->ir, 2);
        out += "," + formatFloat(dataOut->subAccuV, 2);
        out += "," + String(dataOut->subAccuP);
        out += "," + formatFloat(dataOut->subAccuI, 2);
        // Imag - the heading the compass table is indexed by. Two decimals: a calibration point is
        // read straight off this, so rounding it to a whole degree would put that error in the
        // table for good.
        out += "," + formatFloat(dataOut->imag, 2);
        // Attitude, so the handheld can show a level while the hull is being turned. Tilt is not a
        // cosmetic reading here: heading comes from the horizontal field component, so a listing
        // hull mixes in the vertical one and the error is stored as if it were deviation. One
        // decimal is ample for a bubble.
        out += "," + formatFloat(dataOut->pitch, 1);
        out += "," + formatFloat(dataOut->roll, 1);
        break;
    case TOPDATA:
        out += "," + formatFloat(dataOut->dirMag, 0);
        out += "," + String(dataOut->gpsDir);
        out += "," + formatFloat(dataOut->tgDir, 0);
        out += "," + formatFloat(dataOut->tgDist, 1);
        out += "," + formatFloat(dataOut->wDir, 0);
        out += "," + formatFloat(dataOut->wStd, 1);
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        out += "," + formatFloat(dataOut->ip, 2);
        out += "," + formatFloat(dataOut->ir, 2);
        out += "," + formatFloat(dataOut->subAccuV, 2);
        out += "," + String(dataOut->subAccuP);
        out += "," + formatFloat(dataOut->lat, 8);
        out += "," + formatFloat(dataOut->lng, 8);
        out += "," + String(dataOut->gpsFix);
        out += "," + String(dataOut->gpsSat);
        out += "," + formatFloat(dataOut->subAccuI, 2);
        // Imag, see the SUBDATA case above. dirMag goes out of TOPDATA rounded to whole degrees,
        // which is fine for a display but useless for a calibration - hence two decimals here.
        out += "," + formatFloat(dataOut->imag, 2);
        break;
    case SPBBSPSB:
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        break;
    case SETLOCKPOS:
    case SETDOCKPOS:
        out += "," + formatFloat(dataOut->tgLat, 10);
        out += "," + formatFloat(dataOut->tgLng, 10);
        break;
    case LOCKPOS:
    case DOCKPOS:
        out += "," + formatFloat(dataOut->tgLat, 10);
        out += "," + formatFloat(dataOut->tgLng, 10);
        out += "," + formatFloat(dataOut->wDir, 1);
        out += "," + formatFloat(dataOut->wStd, 1);
        break;
    case WINDDATA:
        out += "," + formatFloat(dataOut->wDir, 1);
        out += "," + formatFloat(dataOut->wStd, 1);
        break;
    case DIRDIST:
        out += "," + formatFloat(dataOut->tgDir, 1);
        out += "," + formatFloat(dataOut->tgDist, 1);
        break;
    case MAXMINPWRSET:
    case MAXMINPWR:
        out += "," + String(dataOut->maxSpeed);
        out += "," + String(dataOut->minSpeed);
        out += "," + formatFloat(dataOut->pivotSpeed, 2);
        break;
    case DIRMDIRTGDIRG:
        out += "," + formatFloat(dataOut->dirMag, 0);
        out += "," + formatFloat(dataOut->tgDir, 0);
        out += "," + String(dataOut->gpsDir);
        break;
    case STORE_DECLINATION:
        // Retired, see RoboCompute.h. Kept so it is not reported as an unknown formatter.
        break;
    case RAWCOMPASSDATA:
        out += "," + formatFloat(dataOut->magHard[0], 5);
        out += "," + formatFloat(dataOut->magHard[1], 5);
        out += "," + formatFloat(dataOut->magHard[2], 5);
        break;
    case SOFTIRONFACTORS:
        out += "," + formatFloat(dataOut->magSoft[0][0], 5);
        out += "," + formatFloat(dataOut->magSoft[0][1], 5);
        out += "," + formatFloat(dataOut->magSoft[0][2], 5);
        out += "," + formatFloat(dataOut->magSoft[1][0], 5);
        out += "," + formatFloat(dataOut->magSoft[1][1], 5);
        out += "," + formatFloat(dataOut->magSoft[1][2], 5);
        out += "," + formatFloat(dataOut->magSoft[2][0], 5);
        out += "," + formatFloat(dataOut->magSoft[2][1], 5);
        out += "," + formatFloat(dataOut->magSoft[2][2], 5);
        break;
    case HARDIRONFACTORS:
        out += "," + formatFloat(dataOut->magHard[0], 2);
        out += "," + formatFloat(dataOut->magHard[1], 2);
        out += "," + formatFloat(dataOut->magHard[2], 2);
        break;
    case STORE_COMPASS_OFFSET:
        out += "," + formatFloat(dataOut->compassOffset, 2);
        break;
    case ADAPTIVE_TRIM:
        out += "," + formatFloat(dataOut->compass_trim, 3);
        out += "," + String((int)dataOut->compass_trim_enabled);
        break;
    case SET_AS_NORTH:
    case SET_AS_LEVEL:
        break;
    case ATTITUDE:
        out += "," + formatFloat(dataOut->pitch, 1);
        out += "," + formatFloat(dataOut->roll, 1);
        break;
    case LORA_LINK:
        out += "," + String((int)dataOut->linkOmitted);
        for (int i = 0; i < dataOut->linkPeers && i < LORA_LINK_MAX_PEERS; i++)
        {
            out += "," + String(dataOut->linkPeerId[i], HEX);
            out += "," + String((int)dataOut->linkRssi[i]);
            out += "," + String((int)dataOut->linkCount[i]);
        }
        break;
    case CAL8_SESSION:
        // The action first, so a SET and the answer to it have the same shape - the Sub echoes what
        // it was asked to do, which is what lets a sender tell its own press apart from somebody
        // else's on the same run.
        out += "," + String(dataOut->cal8Action);
        out += "," + String((int)dataOut->cal8Active);
        out += "," + String(dataOut->cal8Next);
        for (int i = 0; i < 8; i++)
            out += "," + formatFloat(dataOut->cal8Captured[i], 2);
        out += "," + String((int)dataOut->cal8Mask);
        out += "," + String((int)dataOut->cal8Seq);
        break;
    case STORE_INTERPOLATION_TABLE:
        // A GET request still puts its (identity) table on the wire rather than sending a short
        // frame: the decoder keys on the field count, so a short frame would be indistinguishable
        // from a truncated one.
        for (int i = 0; i < 8; i++)
            out += "," + formatFloat(dataOut->interpolationTable[i], 2);
        // ... followed by whether it is usable at all. Stored but not usable is a real state: the
        // buoy ignores an out-of-order table and sails uncorrected, see the decoder.
        out += "," + String((int)dataOut->interpUsable);
        break;
    case DOCKED:
    case LOCKED:
    case DOCKING:
    case LOCKING:
        out += "," + formatFloat(dataOut->tgDir, 1);
        out += "," + formatFloat(dataOut->tgDist, 1);
        out += "," + formatFloat(dataOut->tgSpeed, 1);
        out += "," + formatFloat(dataOut->wDir, 1);
        out += "," + formatFloat(dataOut->wStd, 1);
        break;
    case REMOTE:
        out += "," + formatFloat(dataOut->tgDir, 0);
        out += "," + formatFloat(dataOut->tgSpeed, 0);
        break;
    case IDLE:
    case IDLING:
        out += ",0,0";
        break;
    case PING:
        return String(PING);
    case PONG:
        return String(PONG);
    default:
        printf("RoboCode: Unknown formatter <%d>\r\n", dataOut->cmd);
        break;
    }

    // Compress zeros to empty strings to save bandwidth
    String optimized = "";
    int lastComma = -1;
    for (unsigned int i = 0; i <= out.length(); i++) {
        if (i == out.length() || out[i] == ',') {
            String token = out.substring(lastComma + 1, i);
            if (token.length() > 0) {
                bool isZero = true;
                for (unsigned int j = 0; j < token.length(); j++) {
                    if (token[j] != '0' && token[j] != '.' && token[j] != '-') {
                        isZero = false;
                        break;
                    }
                }
                if (isZero && token != "-" && token != "." && token != "-.") {
                    token = "";
                }
            }
            if (lastComma != -1) optimized += ",";
            optimized += token;
            lastComma = i;
        }
    }
    return optimized;
}

String rfCode(RoboStruct *rfOut)
{
    String rfMsg = String(rfOut->IDr, HEX);
    rfMsg += "," + String(rfOut->IDs, HEX);
    rfMsg += "," + String(rfOut->ack);
    rfMsg += "," + RoboCode(rfOut);
    rfMsg = addCRCToString(rfMsg);
    return rfMsg;
}

String addCRCToString(String input)
{
    input.trim();
    input.replace(" ", "");
    byte crc = 0;
    for (int i = 0; i < input.length(); i++) crc ^= (byte)input.charAt(i);
    char crcHex[3];
    sprintf(crcHex, "%02X", crc);
    return "$" + input + "*" + String(crcHex);
}

void rfDeCode(String rfIn, RoboStruct *in)
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
}

bool verifyCRC(String input)
{
    int start = input.indexOf('$');
    int end = input.indexOf('*');
    if (start == -1 || end == -1 || end <= start || end + 2 >= input.length()) return false;
    byte calculatedCRC = 0;
    for (int i = start + 1; i < end; i++) calculatedCRC ^= input[i];
    String givenCRC = input.substring(end + 1, end + 3);
    char calculatedCRCHex[3];
    sprintf(calculatedCRCHex, "%02X", calculatedCRC);
    return givenCRC.equalsIgnoreCase(String(calculatedCRCHex));
}

void addNewSampleInBuffer(RoboWindStruct *wData, double nwdata)
{
    wData->data[wData->ptr] = nwdata;
    wData->ptr = (wData->ptr + 1) % SAMPELS;
}

void averageWindVector(RoboWindStruct *wData)
{
    double sumX = 0, sumY = 0;
    for (int i = 0; i < SAMPELS; ++i)
    {
        double angleRad = radians(wData->data[i]);
        sumX += cos(angleRad);
        sumY += sin(angleRad);
    }
    double meanAngle = atan2(sumY, sumX);
    wData->wDir = fmod((meanAngle * 180.0 / M_PI) + 360.0, 360.0);
}

void deviationWindRose(RoboWindStruct *wData)
{
    averageWindVector(wData);
    double sumSin = 0.0, sumCos = 0.0;
    for (int i = 0; i < SAMPELS; ++i)
    {
        double angleRad = wData->data[i] * M_PI / 180.0;
        sumCos += cos(angleRad);
        sumSin += sin(angleRad);
    }
    double R = sqrt(sumCos * sumCos + sumSin * sumSin) / SAMPELS;
    if (R > 1.0) R = 1.0;
    if (R < 0.000001) R = 0.000001;
    double circStdRad = sqrt(-2.0 * log(R));
    wData->wStd = circStdRad * 180.0 / M_PI;
}

void PidDecode(String data, int pid, RoboStruct *buoy)
{
    String numbers[20];
    int count = 0;
    int startIndex = data.indexOf('$') + 1;
    int endIndex = data.indexOf('*');
    String substring = data.substring(startIndex, endIndex);
    while (count < 20)
    {
        int commaIndex = substring.indexOf(',');
        if (commaIndex == -1) { numbers[count++] = substring; break; }
        numbers[count++] = substring.substring(0, commaIndex);
        substring = substring.substring(commaIndex + 1);
    }
    if (pid == PIDSPEED)
    {
        buoy->Kps = numbers[1].toDouble();
        buoy->Kis = numbers[2].toDouble();
        buoy->Kds = numbers[3].toDouble();
    }
    if (pid == PIDRUDDER)
    {
        buoy->Kpr = numbers[1].toDouble();
        buoy->Kir = numbers[2].toDouble();
        buoy->Kdr = numbers[3].toDouble();
    }
}

String PidEncode(int pid, const RoboStruct *buoy)
{
    String out = "";
    if (pid == PIDSPEED)
    {
        out = String(buoy->Kps);
        out += "," + String(buoy->Kis);
        out += "," + String(buoy->Kds);
    }
    if (pid == PIDRUDDER)
    {
        out = String(buoy->Kpr);
        out += "," + String(buoy->Kir);
        out += "," + String(buoy->Kdr);
    }
    return out;
}

void gpsGem(double &lat, double &lon)
{
    static double gpsgem[20][2];
    static int point = 0;
    gpsgem[point][0] = lat;
    gpsgem[point][1] = lon;
    point = (point + 1) % 20;
    double sumLat = 0, sumLon = 0;
    for (int i = 0; i < 20; i++) { sumLat += gpsgem[i][0]; sumLon += gpsgem[i][1]; }
    lat = sumLat / 20;
    lon = sumLon / 20;
}

double distanceBetween(double lat1, double lon1, double lat2, double lon2)
{
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    lon2 = lon2 * M_PI / 180.0;
    double dLat = lat2 - lat1, dLon = lon2 - lon1;
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_MEAN_RADIUS * c;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2)
{
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    return fmod((atan2(y, x) * (180.0 / M_PI) + 360.0), 360.0);
}

double smallestAngle(double heading1, double heading2)
{
    double angle = fmod(heading2 - heading1 + 360, 360);
    if (angle > 180) return angle - 360;
    return angle;
}

void adjustPositionDirDist(double bearing_deg, double distance,
                           double lat1_deg, double lon1_deg,
                           double *lat2_deg, double *lon2_deg)
{
    double lat1 = radians(lat1_deg), lon1 = radians(lon1_deg), bearing = radians(bearing_deg);
    // EARTH_MEAN_RADIUS, the same figure distanceBetween() measures with. It used to be 6371000
    // here against 6372795 there - 1 part in 3549 - which made placing a point at distance d and
    // then measuring back to it return 1.000282 * d. recalcStartLine() does exactly that round
    // trip: it takes the length off the two ends and lays the squared line out at the same length.
    // So every press of ALIGN STARTLINE grew the line by 0.028%. Far too little to be the reason a
    // line looks a different length - 9 mm on a 31 m line - but the two must agree, because the
    // whole design of that function is that measuring and placing are inverses.
    double ad = distance / EARTH_MEAN_RADIUS;
    double lat2 = asin(sin(lat1) * cos(ad) + cos(lat1) * sin(ad) * cos(bearing));
    double lon2 = lon1 + atan2(sin(bearing) * sin(ad) * cos(lat1), cos(ad) - sin(lat1) * sin(lat2));
    *lat2_deg = degrees(lat2);
    *lon2_deg = degrees(lon2);
    if (*lon2_deg > 180.0) *lon2_deg -= 360.0;
    else if (*lon2_deg < -180.0) *lon2_deg += 360.0;
}

double calculateAngle(double x1, double y1, double x2, double y2)
{
    double dotProduct = x1 * x2 + y1 * y2;
    double magnitudeA = sqrt(x1 * x1 + y1 * y1);
    double magnitudeB = sqrt(x2 * x2 + y2 * y2);
    double cosAngle = dotProduct / (magnitudeA * magnitudeB);
    cosAngle = fmax(-1.0, fmin(1.0, cosAngle));
    return acos(cosAngle) * (180.0 / M_PI);
}
double calculateAngleSigned(double x1, double y1, double x2, double y2)
{
    double dotProduct = x1 * x2 + y1 * y2;
    double magnitudeA = sqrt(x1 * x1 + y1 * y1);
    double magnitudeB = sqrt(x2 * x2 + y2 * y2);
    double cosAngle = dotProduct / (magnitudeA * magnitudeB);
    cosAngle = fmax(-1.0, fmin(1.0, cosAngle));
    double crossProduct = x1 * y2 - y1 * x2;
    double angle = atan2(crossProduct, dotProduct) * (180.0 / M_PI);
    return angle;
}

double computeWindAngle(double windDegrees, double lat, double lon, double centroidLat, double centroidLon)
{
    double bearing = calculateBearing(centroidLat, centroidLon, lat, lon);
    double angleDifference = fabs(windDegrees - bearing);
    if (angleDifference > 180)
        angleDifference = 360 - angleDifference;
    return angleDifference;
}

#define INVALID_POINT(p) ((p).tgLat == 0.0 || (p).tgLng == 0.0)

// Which of a pair takes the STARBOARD end of the freshly squared start line.
//
// This used to be read off trackPos - but calcTrackPos() clears all three trackPos fields to -1
// BEFORE it bails on a missing third buoy, so by the time recalcStartLine() runs on a two-buoy
// fleet the test was always false. The first slot therefore went to the BB end and the second to
// the SB end whatever the geometry was, and whenever that was the wrong way round the pair
// swapped ends and motored straight through each other. Measured on a real 31 m line: 61.5 m of
// travel as assigned against 9.3 m for the other pairing.
//
// Picking the pairing with the shorter TOTAL travel cannot cross: for two points and two targets
// the crossing assignment is always the longer one, by the triangle inequality.
static bool starboardGoesToFirst(double aLat, double aLng, double bLat, double bLng,
                                 double sbLat, double sbLng, double bbLat, double bbLng)
{
    double aTakesSb = distanceBetween(aLat, aLng, sbLat, sbLng)
                    + distanceBetween(bLat, bLng, bbLat, bbLng);
    double aTakesBb = distanceBetween(aLat, aLng, bbLat, bbLng)
                    + distanceBetween(bLat, bLng, sbLat, sbLng);
    return aTakesSb <= aTakesBb;
}

// See the header for why this is a vector mean and why 0/0 is "no reading".
double meanWindDir(double dirA, double stdA, double dirB, double stdB, double fallback)
{
    bool okA = (dirA != 0.0 || stdA != 0.0);
    bool okB = (dirB != 0.0 || stdB != 0.0);
    if (!okA && !okB) return fallback;
    if (!okB) return dirA;
    if (!okA) return dirB;

    double ra = dirA * M_PI / 180.0, rb = dirB * M_PI / 180.0;
    double x = cos(ra) + cos(rb), y = sin(ra) + sin(rb);
    // Exactly opposite readings have no mean direction, and the perpendicular atan2 would hand
    // back is not one either. That is a fault rather than a wind, so keep the reading the
    // computing buoy was going to use anyway.
    if (fabs(x) < 1e-9 && fabs(y) < 1e-9) return fallback;

    double m = atan2(y, x) * 180.0 / M_PI;
    return fmod(m + 360.0, 360.0);
}

void threePointAverage(struct RoboStruct p3[3], double *latgem, double *lnggem)
{
    *latgem = (p3[0].tgLat + p3[1].tgLat + p3[2].tgLat) / 3;
    *lnggem = (p3[0].tgLng + p3[1].tgLng + p3[2].tgLng) / 3;
}

void twoPointAverage(double lat1, double lon1, double lat2, double lon2, double *latgem, double *longem)
{
    *latgem = (lat1 + lat2) / 2;
    *longem = (lon1 + lon2) / 2;
}

// Returns true only when a start line was actually computed.
// The caller used to infer success from the trackPos fields, but calcTrackPos() sets those
// before this runs, so a bail-out here still looked like success: you got the confirmation beep
// and a SENDTRACK while nothing had been calculated.
bool recalcStartLine(struct RoboStruct rsl[3])
{
    double d0 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng);
    double d1 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng);
    double d2 = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng);

    double midLat, midLng;
    double angleSb, angleBb;

    printf("# Winddir 0:%.2f 1:%.3f 2:%.2f \r\n", rsl[0].wDir, rsl[1].wDir, rsl[2].wDir);

    if (d0 < d1 && d0 < d2)
    {
        if (INVALID_POINT(rsl[0]) || INVALID_POINT(rsl[1]))
        {
            printf("# No data to compute with (0-1)\r\n");
            return false;
        }

        twoPointAverage(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng, &midLat, &midLng);
        // The mean of what BOTH ends report, not slot 0 alone - see meanWindDir(). Slot 0 stays
        // the fallback: handleStatus() puts the computing buoy's filtered wind there, and it has
        // already refused to compute at all if that reading is missing.
        angleSb = fmod(meanWindDir(rsl[0].wDir, rsl[0].wStd, rsl[1].wDir, rsl[1].wStd,
                                   rsl[0].wDir) + 90.0, 360.0);
        angleBb = fmod(angleSb + 180.0, 360.0);

        // Both candidate ends first, then hand each buoy the one it is nearer to - see
        // starboardGoesToFirst(). They have to be computed before anything is written back,
        // because the answer depends on where the buoys are NOW.
        double sbLat, sbLng, bbLat, bbLng;
        adjustPositionDirDist(angleSb, d0 / 2, midLat, midLng, &sbLat, &sbLng);
        adjustPositionDirDist(angleBb, d0 / 2, midLat, midLng, &bbLat, &bbLng);
        bool firstIsStarboard = starboardGoesToFirst(rsl[0].tgLat, rsl[0].tgLng,
                                                     rsl[1].tgLat, rsl[1].tgLng,
                                                     sbLat, sbLng, bbLat, bbLng);
        rsl[0].tgLat = firstIsStarboard ? sbLat : bbLat;
        rsl[0].tgLng = firstIsStarboard ? sbLng : bbLng;
        rsl[1].tgLat = firstIsStarboard ? bbLat : sbLat;
        rsl[1].tgLng = firstIsStarboard ? bbLng : sbLng;

        rsl[0].trackPos = firstIsStarboard ? STARBOARD : PORT;
        rsl[1].trackPos = firstIsStarboard ? PORT : STARBOARD;
        return true;
    }
    else if (d1 < d0 && d1 < d2)
    {
        if (INVALID_POINT(rsl[0]) || INVALID_POINT(rsl[2]))
        {
            printf("# No data to compute with (0-2)\r\n");
            return false;
        }

        twoPointAverage(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng, &midLat, &midLng);
        // Mean of both ends, same as the branch above.
        angleSb = fmod(meanWindDir(rsl[0].wDir, rsl[0].wStd, rsl[2].wDir, rsl[2].wStd,
                                   rsl[0].wDir) + 90.0, 360.0);
        angleBb = fmod(angleSb + 180.0, 360.0);

        // Nearest end wins, same as the branch above.
        double sbLat, sbLng, bbLat, bbLng;
        adjustPositionDirDist(angleSb, d1 / 2, midLat, midLng, &sbLat, &sbLng);
        adjustPositionDirDist(angleBb, d1 / 2, midLat, midLng, &bbLat, &bbLng);
        bool firstIsStarboard = starboardGoesToFirst(rsl[0].tgLat, rsl[0].tgLng,
                                                     rsl[2].tgLat, rsl[2].tgLng,
                                                     sbLat, sbLng, bbLat, bbLng);
        rsl[0].tgLat = firstIsStarboard ? sbLat : bbLat;
        rsl[0].tgLng = firstIsStarboard ? sbLng : bbLng;
        rsl[2].tgLat = firstIsStarboard ? bbLat : sbLat;
        rsl[2].tgLng = firstIsStarboard ? bbLng : sbLng;

        rsl[0].trackPos = firstIsStarboard ? STARBOARD : PORT;
        rsl[2].trackPos = firstIsStarboard ? PORT : STARBOARD;
        return true;
    }
    else if (d2 < d0 && d2 < d1)
    {
        if (INVALID_POINT(rsl[1]) || INVALID_POINT(rsl[2]))
        {
            printf("# No data to compute with (1-2)\r\n");
            return false;
        }

        twoPointAverage(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng, &midLat, &midLng);
        // The two ENDS here are slots 1 and 2 - slot 0 is the upwind mark, and its reading is no
        // part of the line. It stays the fallback for the same reason as the other branches:
        // handleStatus() copies the computing buoy's filtered wind into slot 0, and that is the
        // reading the compute guards have already tested.
        angleSb = fmod(meanWindDir(rsl[1].wDir, rsl[1].wStd, rsl[2].wDir, rsl[2].wStd,
                                   rsl[0].wDir) + 90.0, 360.0);
        angleBb = fmod(angleSb + 180.0, 360.0);

        // Nearest end wins, same as the branches above.
        double sbLat, sbLng, bbLat, bbLng;
        adjustPositionDirDist(angleSb, d2 / 2, midLat, midLng, &sbLat, &sbLng);
        adjustPositionDirDist(angleBb, d2 / 2, midLat, midLng, &bbLat, &bbLng);
        bool firstIsStarboard = starboardGoesToFirst(rsl[1].tgLat, rsl[1].tgLng,
                                                     rsl[2].tgLat, rsl[2].tgLng,
                                                     sbLat, sbLng, bbLat, bbLng);
        rsl[1].tgLat = firstIsStarboard ? sbLat : bbLat;
        rsl[1].tgLng = firstIsStarboard ? sbLng : bbLng;
        rsl[2].tgLat = firstIsStarboard ? bbLat : sbLat;
        rsl[2].tgLng = firstIsStarboard ? bbLng : sbLng;

        rsl[1].trackPos = firstIsStarboard ? STARBOARD : PORT;
        rsl[2].trackPos = firstIsStarboard ? PORT : STARBOARD;
        return true;
    }

    // No branch matched: the three separations are equal (typically all-zero targets), so
    // there is no shortest leg to use as the start line.
    printf("# No usable buoy pair for a start line\r\n");
    return false;
}

// Same line, further apart. See the header for why this is not recalcStartLine() with a different
// length: that one squares to the wind, and a race officer asking for ten more metres is not asking
// to have the line rotated as well. Nothing here reads a wind direction.
bool extendStartLine(struct RoboStruct rsl[3], double metres)
{
    double d0 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng);
    double d1 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng);
    double d2 = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng);

    // The shortest leg is the start line, exactly as recalcStartLine() decides it - the two have to
    // agree about which pair they are talking about or a press of one undoes a press of the other.
    int a, b;
    double len;
    if (d0 < d1 && d0 < d2)      { a = 0; b = 1; len = d0; }
    else if (d1 < d0 && d1 < d2) { a = 0; b = 2; len = d1; }
    else if (d2 < d0 && d2 < d1) { a = 1; b = 2; len = d2; }
    else
    {
        printf("# No usable buoy pair to extend\r\n");
        return false;
    }

    if (INVALID_POINT(rsl[a]) || INVALID_POINT(rsl[b]))
    {
        printf("# No data to extend with (%d-%d)\r\n", a, b);
        return false;
    }

    double newLen = len + metres;
    if (newLen < MIN_START_LINE_M)
    {
        // Two ends on the same spot have no bearing between them to preserve, so a line shrunk to
        // nothing cannot be grown again - it would come back on whatever bearing the rounding
        // happened to produce.
        printf("# Start line not extended: %.1f m + %.1f m is below the %.1f m floor\r\n",
               len, metres, (double)MIN_START_LINE_M);
        return false;
    }

    double midLat, midLng;
    twoPointAverage(rsl[a].tgLat, rsl[a].tgLng, rsl[b].tgLat, rsl[b].tgLng, &midLat, &midLng);

    // Both bearings are read BEFORE either end is written back. Each buoy is pushed straight out
    // along the bearing it is already on, which is what keeps it on its own side - recompute the
    // second bearing after moving the first end and the pair can swap ends and motor through each
    // other, which is the failure starboardGoesToFirst() exists to prevent in recalcStartLine().
    double brgA = calculateBearing(midLat, midLng, rsl[a].tgLat, rsl[a].tgLng);
    double brgB = calculateBearing(midLat, midLng, rsl[b].tgLat, rsl[b].tgLng);

    adjustPositionDirDist(brgA, newLen / 2.0, midLat, midLng, &rsl[a].tgLat, &rsl[a].tgLng);
    adjustPositionDirDist(brgB, newLen / 2.0, midLat, midLng, &rsl[b].tgLat, &rsl[b].tgLng);

    // trackPos is deliberately left alone. The ends keep the roles they were given when the line
    // was squared; moving a buoy further out along its own bearing cannot change which side it is.
    printf("# Start line extended %.1f m -> %.1f m, bearing and midpoint unchanged\r\n",
           len, newLen);
    return true;
}

bool reCalcTrack(struct RoboStruct rsl[3])
{
    double d0, d1, d2;
    double startLineL, centerPont2Startline, centerPoint2Head;
    double angleSb, angleBb, angle180;
    double lat2gem, lng2gem;
    double lat3gem, lng3gem;

    // Reset track positions
    for (int i = 0; i < 3; i++)
    {
        rsl[i].trackPos = -1;
        if (rsl[i].tgLng == 0 || rsl[i].tgLat == 0)
        {
            // Needs all THREE buoys to have a valid lock position; with only two deployed this
            // always bails, so COMPUTE TRACK is a three-buoy operation.
            printf("# No data to compute with (buoy %d has no lock position)\r\n", i);
            return false;
        }
    }

    // Midpoint of three buoys
    threePointAverage(rsl, &lat3gem, &lng3gem);
    printf("midpoint =(%.12f,%.12f)\r\n", lat3gem, lng3gem);

    // Distances between buoys
    d0 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng);
    d1 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng);
    d2 = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng);

    // Determine start line and midpoint of that line
    // windDir is the mean of what the two START LINE buoys report - see meanWindDir(). The whole
    // course hangs off this one figure: the head mark's bearing from the centre, the offset back
    // down to the line, and which end of the line is starboard. It used to be rsl[0].wDir alone,
    // which is whichever buoy the command happened to reach, and the upwind mark's own reading
    // carried the same weight as the line's when it is the furthest from where the start happens.
    // Slot 0 stays the fallback: handleStatus() puts the computing buoy's filtered wind there.
    double windDir;
    if (d0 < d1 && d0 < d2)
    {
        startLineL = d0;
        twoPointAverage(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng, &lat2gem, &lng2gem);
        windDir = meanWindDir(rsl[0].wDir, rsl[0].wStd, rsl[1].wDir, rsl[1].wStd, rsl[0].wDir);
    }
    else if (d1 < d2 && d1 < d0)
    {
        startLineL = d1;
        twoPointAverage(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng, &lat2gem, &lng2gem);
        windDir = meanWindDir(rsl[0].wDir, rsl[0].wStd, rsl[2].wDir, rsl[2].wStd, rsl[0].wDir);
    }
    else
    {
        startLineL = d2;
        twoPointAverage(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng, &lat2gem, &lng2gem);
        windDir = meanWindDir(rsl[1].wDir, rsl[1].wStd, rsl[2].wDir, rsl[2].wStd, rsl[0].wDir);
    }

    // Compute wind angles to midpoint
    double b0 = computeWindAngle(windDir, rsl[0].tgLat, rsl[0].tgLng, lat3gem, lng3gem);
    double b1 = computeWindAngle(windDir, rsl[1].tgLat, rsl[1].tgLng, lat3gem, lng3gem);
    double b2 = computeWindAngle(windDir, rsl[2].tgLat, rsl[2].tgLng, lat3gem, lng3gem);

    printf("winddir =(%.1f) mean of the two line buoys, ends reported %.1f / %.1f / %.1f\r\n",
           windDir, rsl[0].wDir, rsl[1].wDir, rsl[2].wDir);

    angle180 = windDir + 180;
    if (angle180 > 360)
        angle180 -= 360;

    angleSb = windDir + 90;
    if (angleSb > 360)
        angleSb -= 360;

    angleBb = angleSb + 180;
    if (angleBb > 360)
        angleBb -= 360;

    // Determine which buoy is HEAD (closest to wind direction)
    if (b0 < b1 && b0 < b2)
    {
        centerPoint2Head = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, lat3gem, lng3gem);
        centerPont2Startline = distanceBetween(lat2gem, lng2gem, lat3gem, lng3gem);
        adjustPositionDirDist(windDir, centerPoint2Head, lat3gem, lng3gem, &rsl[0].tgLat, &rsl[0].tgLng); // HEAD
        adjustPositionDirDist(angle180, centerPont2Startline, lat3gem, lng3gem, &lat2gem, &lng2gem);
        adjustPositionDirDist(angleBb, startLineL / 2, lat2gem, lng2gem, &rsl[1].tgLat, &rsl[1].tgLng); // PORT
        adjustPositionDirDist(angleSb, startLineL / 2, lat2gem, lng2gem, &rsl[2].tgLat, &rsl[2].tgLng); // STARBOARD
        rsl[0].trackPos = HEAD;
        rsl[1].trackPos = PORT;
        rsl[2].trackPos = STARBOARD;
    }
    else if (b1 < b0 && b1 < b2)
    {
        centerPoint2Head = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, lat3gem, lng3gem);
        centerPont2Startline = distanceBetween(lat2gem, lng2gem, lat3gem, lng3gem);
        adjustPositionDirDist(windDir, centerPoint2Head, lat3gem, lng3gem, &rsl[1].tgLat, &rsl[1].tgLng); // HEAD
        adjustPositionDirDist(angle180, centerPont2Startline, lat3gem, lng3gem, &lat2gem, &lng2gem);
        adjustPositionDirDist(angleBb, startLineL / 2, lat2gem, lng2gem, &rsl[0].tgLat, &rsl[0].tgLng); // PORT
        adjustPositionDirDist(angleSb, startLineL / 2, lat2gem, lng2gem, &rsl[2].tgLat, &rsl[2].tgLng); // STARBOARD
        rsl[1].trackPos = HEAD;
        rsl[0].trackPos = PORT;
        rsl[2].trackPos = STARBOARD;
    }
    else
    {
        centerPoint2Head = distanceBetween(rsl[2].tgLat, rsl[2].tgLng, lat3gem, lng3gem);
        centerPont2Startline = distanceBetween(lat2gem, lng2gem, lat3gem, lng3gem);
        adjustPositionDirDist(windDir, centerPoint2Head, lat3gem, lng3gem, &rsl[2].tgLat, &rsl[2].tgLng); // HEAD
        adjustPositionDirDist(angle180, centerPont2Startline, lat3gem, lng3gem, &lat2gem, &lng2gem);
        adjustPositionDirDist(angleBb, startLineL / 2, lat2gem, lng2gem, &rsl[0].tgLat, &rsl[0].tgLng); // PORT
        adjustPositionDirDist(angleSb, startLineL / 2, lat2gem, lng2gem, &rsl[1].tgLat, &rsl[1].tgLng); // STARBOARD
        rsl[2].trackPos = HEAD;
        rsl[0].trackPos = PORT;
        rsl[1].trackPos = STARBOARD;
    }
    return true;
}

void trackPosPrint(int c)
{
    if (c == HEAD) printf("HEAD");
    else if (c == PORT) printf("PORT");
    else if (c == STARBOARD) printf("STARBOARD");
    else printf("NON");
}

RoboStruct calcTrackPos(RoboStruct rsl[3])
{
    double dir = 0;
    double d0 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng); // compute length p0 p1
    double d1 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng); // compute length p0 p2
    double d2 = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng); // compute length p1 p2
    if (d0 < d1 && d0 < d2)
    {
        dir = calculateBearing(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng);
        // Mean of the two buoys the line runs between, not slot 0 alone - see meanWindDir().
        if (smallestAngle(meanWindDir(rsl[0].wDir, rsl[0].wStd, rsl[1].wDir, rsl[1].wStd,
                                      rsl[0].wDir), dir) >= 0)
        {
            rsl[0].trackPos = PORT;
            rsl[1].trackPos = STARBOARD;
            rsl[2].trackPos = HEAD;
        }
        else
        {
            rsl[0].trackPos = STARBOARD;
            rsl[1].trackPos = PORT;
            rsl[2].trackPos = HEAD;
        }
    }
    if (d1 < d0 && d1 < d2)
    {
        dir = calculateBearing(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng);
        if (smallestAngle(meanWindDir(rsl[0].wDir, rsl[0].wStd, rsl[2].wDir, rsl[2].wStd,
                                      rsl[0].wDir), dir) >= 0)
        {
            rsl[0].trackPos = PORT;
            rsl[1].trackPos = HEAD;
            rsl[2].trackPos = STARBOARD;
        }
        else
        {
            rsl[0].trackPos = STARBOARD;
            rsl[1].trackPos = HEAD;
            rsl[2].trackPos = PORT;
        }
    }
    if (d2 < d0 && d2 < d1)
    {
        dir = calculateBearing(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng);
        // The two ends are slots 1 and 2 here; slot 0 is the upwind mark and stays the fallback.
        if (smallestAngle(meanWindDir(rsl[1].wDir, rsl[1].wStd, rsl[2].wDir, rsl[2].wStd,
                                      rsl[0].wDir), dir) >= 0)
        {
            rsl[0].trackPos = HEAD;
            rsl[1].trackPos = PORT;
            rsl[2].trackPos = STARBOARD;
        }
        else
        {
            rsl[0].trackPos = HEAD;
            rsl[1].trackPos = STARBOARD;
            rsl[2].trackPos = PORT;
        }
    }
    printf("# dir= %.0f\r\n", dir);
    return rsl[0];
}

void MergeBuoyData(RoboStruct *dst, const RoboStruct &src)
{
    // Copy ONLY the fields the incoming message actually carries on the wire.
    //
    // This used to be a plain "*dst = src". Every message type carries only a subset of the
    // struct, so a wholesale copy blanked everything the message did not contain. Because the
    // buoys alternate BUOYPOS and TOPDATA roughly twice a second, any field carried by one but
    // not the other visibly flickered between its real value and 0 - the sub voltage is only in
    // TOPDATA/SUBDATA, so every BUOYPOS zeroed it. The same defect wiped a remote buoy's set
    // point one telemetry cycle after it locked, and its SETUPDATA reply before the web UI could
    // read it.
    //
    // The field lists below mirror RoboDecode() above; keep them in step when the wire format
    // changes. Anything not listed keeps its previous value, so a stale reading is shown rather
    // than a fake zero.
    //
    // NOTE ON STACK USAGE: src is a reference and nothing here copies a whole RoboStruct.
    // RoboStruct is ~500 bytes and this runs in the loop task below handleRfData() and
    // AddDataToBuoyBase(); copying it here overflowed the stack, which reboots the ESP32.

    // Envelope - rfDeCode()/RoboDecode() always fill these in
    dst->IDs = src.IDs;
    dst->IDr = src.IDr;
    dst->ack = src.ack;
    dst->cmd = src.cmd;
    dst->status = src.status;

    switch (src.cmd)
    {
    case BUOYPOS:
        dst->lat = src.lat;             dst->lng = src.lng;
        dst->dirMag = src.dirMag;
        dst->wDir = src.wDir;           dst->wStd = src.wStd;
        dst->topAccuP = src.topAccuP;   dst->subAccuP = src.subAccuP;
        dst->gpsFix = src.gpsFix;       dst->gpsSat = src.gpsSat;
        break;

    case TOPDATA:
        dst->dirMag = src.dirMag;       dst->gpsDir = src.gpsDir;
        dst->imag = src.imag;
        dst->tgDir = src.tgDir;         dst->tgDist = src.tgDist;
        dst->wDir = src.wDir;           dst->wStd = src.wStd;
        dst->speedBb = src.speedBb;     dst->speedSb = src.speedSb;
        dst->ip = src.ip;               dst->ir = src.ir;
        dst->subAccuV = src.subAccuV;   dst->subAccuP = src.subAccuP;
        dst->subAccuI = src.subAccuI;
        dst->lat = src.lat;             dst->lng = src.lng;
        dst->gpsFix = src.gpsFix;       dst->gpsSat = src.gpsSat;
        break;

    case SUBDATA:
        dst->dirMag = src.dirMag;
        dst->imag = src.imag;
        dst->speedBb = src.speedBb;     dst->speedSb = src.speedSb;
        dst->ip = src.ip;               dst->ir = src.ir;
        dst->subAccuV = src.subAccuV;   dst->subAccuP = src.subAccuP;
        dst->subAccuI = src.subAccuI;
        dst->pitch = src.pitch;         dst->roll = src.roll;
        break;

    case SUBACCU:
        dst->subAccuV = src.subAccuV;   dst->subAccuP = src.subAccuP;
        dst->subAccuI = src.subAccuI;
        break;

    case SUBPWR:
        dst->speedSet = src.speedSet;   dst->speed = src.speed;
        dst->speedBb = src.speedBb;     dst->speedSb = src.speedSb;
        dst->subAccuV = src.subAccuV;   dst->subAccuI = src.subAccuI;
        break;

    case LOCKPOS:
    case DOCKPOS:
        dst->tgLat = src.tgLat;         dst->tgLng = src.tgLng;
        dst->wDir = src.wDir;           dst->wStd = src.wStd;
        break;

    case SETLOCKPOS:
    case SETDOCKPOS:
        dst->tgLat = src.tgLat;         dst->tgLng = src.tgLng;
        break;

    case DIRDIST:
        dst->tgDir = src.tgDir;         dst->tgDist = src.tgDist;
        break;

    case SETUPDATA:
        dst->Kpr = src.Kpr; dst->Kir = src.Kir; dst->Kdr = src.Kdr;
        dst->Kps = src.Kps; dst->Kis = src.Kis; dst->Kds = src.Kds;
        dst->maxSpeed = src.maxSpeed;   dst->minSpeed = src.minSpeed;
        dst->pivotSpeed = src.pivotSpeed;
        dst->compassOffset = src.compassOffset;
        dst->holdRad = src.holdRad;
        dst->revBB = src.revBB;         dst->revSB = src.revSB;
        dst->swap_BB_SB = src.swap_BB_SB;
        dst->compass_trim_enabled = src.compass_trim_enabled;
        dst->dockApproachDist = src.dockApproachDist;
        dst->dockApproachDir = src.dockApproachDir;
        dst->dockingToWaypoint = src.dockingToWaypoint;
        break;

    case ADAPTIVE_TRIM:
        dst->compass_trim = src.compass_trim;
        dst->compass_trim_enabled = src.compass_trim_enabled;
        break;

    // Listed so these can never reach the default below, which is a wholesale copy and would
    // blank the buoy's position, battery and set point - the exact defect this function exists
    // to prevent. Neither is fed into the buoy base today; that is a property of the callers,
    // not something worth relying on here.
    case STORE_INTERPOLATION_TABLE:
        for (int i = 0; i < 8; i++) dst->interpolationTable[i] = src.interpolationTable[i];
        dst->interpUsable = src.interpUsable;
        break;

    case ATTITUDE:
        dst->pitch = src.pitch;         dst->roll = src.roll;
        break;

    case LORA_LINK:
        dst->linkPeers = src.linkPeers;
        dst->linkOmitted = src.linkOmitted;
        for (int i = 0; i < LORA_LINK_MAX_PEERS; i++)
        {
            dst->linkPeerId[i] = src.linkPeerId[i];
            dst->linkRssi[i] = src.linkRssi[i];
            dst->linkCount[i] = src.linkCount[i];
        }
        break;

    case CAL8_SESSION:
        dst->cal8Action = src.cal8Action;
        dst->cal8Active = src.cal8Active;
        dst->cal8Next = src.cal8Next;
        for (int i = 0; i < 8; i++) dst->cal8Captured[i] = src.cal8Captured[i];
        dst->cal8Mask = src.cal8Mask;
        dst->cal8Seq = src.cal8Seq;
        break;

    default:
        // Unknown / not a telemetry message: fall back to the original behaviour.
        *dst = src;
        break;
    }
}

// dataIn is a const reference: it used to be taken by value, which put another ~500 byte
// RoboStruct on the stack of the loop task on every single telemetry packet.
void AddDataToBuoyBase(const RoboStruct &dataIn, RoboStruct *buoyPara[3])
{
    // An ID of 0 is not a buoy, so never allocate a slot for it.
    if (dataIn.IDs == 0) return;

    // Two passes, deliberately. The old single pass accepted either an ID match OR the first
    // free slot, whichever came first in index order - so if a buoy was already held in slot 2
    // and slot 1 happened to be free, its next message was filed in slot 1 and the base ended up
    // with the same buoy twice. That duplicate then broke the start line calculation: the distance
    // between a buoy and its own phantom copy is zero, which always wins the "shortest leg"
    // comparison in calcTrackPos()/recalcStartLine().
    for (int i = 0; i < 3; i++)
    {
        if (buoyPara[i] && dataIn.IDs == buoyPara[i]->IDs)
        {
            MergeBuoyData(buoyPara[i], dataIn);
            return;
        }
    }
    for (int i = 0; i < 3; i++)
    {
        if (buoyPara[i] && buoyPara[i]->IDs == 0)
        {
            MergeBuoyData(buoyPara[i], dataIn);
            return;
        }
    }
}

int GetDataPosFromBuoyBase(uint64_t id, RoboStruct buoyPara[3])
{
    for (int i = 0; i < 3; i++) if (id == buoyPara[i].IDs) return i;
    return -1;
}
