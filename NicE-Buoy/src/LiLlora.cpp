/*
 *   https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
 *   Lora message: Destination,SenderAddres,MSGID,Length,Message
 */
#include <Arduino.h>
#include <SPI.h> // include libraries
#include <LoRa.h>
#include "io.h"
#include "LiLlora.h"
#include "datastorage.h"
#include "general.h"
#include "calculate.h"
#include "gps.h"
#include "webinterface.h"
#include "../../dependency/command.h"

QueueHandle_t loradataout;
unsigned int loracmnd = 0;
static unsigned long lasttransmission = 0;
int counter = 0;

String outgoing; // outgoing message

struct loraDataType loraIn;
struct loraDataType loraOut;

byte msgCount = 0;       // count of outgoing messages
byte destination = 0xFF; // destination to send to

bool loraOK = false;

bool InitLora(void)
{
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    Serial.print("LoRa setup ");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LoRa_frequency))
    {
        Serial.println("Failed!");
        return false;
    }
    Serial.println("Succes!");
    loraOK = true;
    return true;
}

String removeWhitespace(String str)
{
    String result = "";
    for (int i = 0; i < str.length(); i++)
    {
        if (str.charAt(i) != ' ')
        {
            result += str.charAt(i);
        }
    }
    return result;
}

/*
 *Lora message: Destination,SenderAddres,Status,mg_id,gsia,Length,Message
 */
bool sendLora(void)
{
    if (lasttransmission + 175 > millis())
    { // prefend fast transmissions
        return 1;
    }
    loraOut.message = removeWhitespace(loraOut.message);
    LoRa.beginPacket();              // start packet
    LoRa.write(loraOut.destination); // add destination address
    LoRa.write(buoyID);              // add sender address
    LoRa.write(status);              // status
    LoRa.write(loraOut.msgid);       // id
    LoRa.write(loraOut.gsia);        // get set inf
    LoRa.write(loraOut.message.length());
    LoRa.print(loraOut.message);
    LoRa.endPacket(); // finish packet and send it
    lasttransmission = millis();
    // ledstatus = true;
    String gsia = "";
    if (loraOut.gsia == ACK)
    {
        gsia = "ACK";
    }
    else if (loraOut.gsia == NAK)
    {
        gsia = "NAK";
    }
    else if (loraOut.gsia == GET)
    {
        gsia = "GET";
    }
    else if (loraOut.gsia == SET)
    {
        gsia = "SET";
    }
    else if (loraOut.gsia == INF)
    {
        gsia = "INF";
    }
    Serial.print("Lora out desst>:" + String(loraOut.destination) + " status:" + String(status) + " msg_id:" + loraOut.msgid + " gsia:" + gsia + " msg <" + String(loraOut.message) + ">\r\n");
    return 0;
}

void sendLoraAck(byte msg_id, int cmnd)
{
    loraOut.msgid = msg_id; // set bit if is answer
    loraOut.gsia = ACK;
    String msg = "";
    loraOut.messagelength = msg.length();
    loraOut.message = msg;
    while (sendLora())
        ;
}
/*
decoding incomming lora messages
<buoy><sender><status><msgID><msglength><msg>
*/
int decodeMsg()
{
    // read packet header bytes:

    int recipient_l = LoRa.read(); // recipient address
    // if the recipient isn't this device or broadcast,
    if (recipient_l != buoyID && recipient_l != 0x0)
    {
        return 0; // skip rest of function
    }
    byte sender_l = LoRa.read();         // sender address
    byte status_l = LoRa.read();         // status
    byte incomingMsgId_l = LoRa.read();  // msg ID
    byte gsia_l = LoRa.read();           // get set info
    byte incomingLength_l = LoRa.read(); // msg length
    String incoming_l = "";
    while (LoRa.available())
    {
        incoming_l += (char)LoRa.read();
    }
    if (incomingLength_l != incoming_l.length()) // check length for error
    {
        Serial.println("Length error skipping message!");
        return 0; // skip rest of function
    }
    // Data valid
    // Get and store the data
    loraIn.recipient = recipient_l;
    loraIn.sender = sender_l;
    loraIn.status = status_l;
    loraIn.msgid = incomingMsgId_l;
    loraIn.gsia = gsia_l;
    loraIn.messagelength = incomingLength_l;
    loraIn.message = incoming_l;
    loraIn.rssi = LoRa.packetRssi();
    loraIn.snr = LoRa.packetSnr();
    return 1;
}

void sendACKNAKINF(String t, Status_t inp)
{
    delay(100);
    loraOut.message = t;
    loraOut.sender = loraIn.sender;
    loraOut.msgid = loraIn.msgid;
    loraOut.gsia = inp;
    while (sendLora())
        ;
}

int polLora(void)
{
    String msg = "";
    int tmpint = 0;
    if (LoRa.parsePacket() == 0)
    {
        return 0; // if there's no packet, return
    }
    if (decodeMsg() == 0)
    {
        return 0;
    }
    char messageArr[100];
    Serial.println("Lora msg> id:" + String(loraIn.msgid) + " gsia:" + String(loraIn.gsia) + " <" + loraIn.message + ">");
    loraIn.message.toCharArray(messageArr, loraIn.message.length() + 1);
    loraOut.destination = loraIn.sender;
    loraOut.sender = buoyID;
    switch (loraIn.msgid)
    {
    case POSITION:
        if (loraIn.gsia == GET)
        {
            msg = String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8) + "," + String(status);
            sendACKNAKINF(msg, ACK);
        }
        break;

    case DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_M_HEADING:
        if (loraIn.gsia == GET)
        {
            msg = String(buoy.tgdir) + "," + String(buoy.tgdistance) + "," + buoy.speed + "," + buoy.speedbb + "," + buoy.speedsb + "," + String(buoy.mheading, 0);
            sendACKNAKINF(msg, ACK);
        }
        break;

    case STORE_DOC_POSITION:
        if (loraIn.gsia == SET) // store dock positon
        {
            if (gpsdata.fix == true)
            {
                sscanf(messageArr, "%lf,%lf", &buoy.tglatitude, &buoy.tglongitude);
                MemoryDockPos(&buoy.tglatitude, &buoy.tglongitude, false);
                sendACKNAKINF("", ACK);
            }
            else
            {
                sendACKNAKINF("", NAK);
            }
        }
        break;
    case STORE_POS_AS_DOC_POSITION:
        if (loraIn.gsia == SET) // store dock positon
        {
            status = IDLE;
            if (gpsdata.fix == true)
            {
                MemoryDockPos(&gpsdata.lat, &gpsdata.lon, false);
                msg = String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8);
                sendACKNAKINF(msg, ACK);
                MemoryDockPos(&buoy.tglatitude, &buoy.tglongitude, true);
                Serial.printf("\r\n\r\nDoc pos set to: %lfN %lfW\r\n\r\n", buoy.tglatitude, buoy.tglongitude);
            }
            else
            {
                sendACKNAKINF("", NAK);
            }
        }

        break;
    case DOC_POSITION:
        if (loraIn.gsia == SET) // sail to doc positon
        {
            initRudderPid();
            MemoryDockPos(&buoy.tglatitude, &buoy.tglongitude, true);
            msg = String(buoy.tglatitude, 8) + "," + String(buoy.tglongitude, 8);
            RouteToPoint(gpsdata.lat, gpsdata.lon, buoy.tglatitude, buoy.tglongitude, &buoy.tgdistance, &buoy.tgdir); // calculate heading and
            rudderpid.iintergrate = 0;
            speedpid.armIntergrator = false;
            status = DOCKED;
            sendACKNAKINF(msg, ACK);
            Serial.printf("Doc positon: https://www.google.nl/maps/@%2.8lf,%2.8lf,16z?entry=ttu\r\n", buoy.tglatitude, buoy.tglongitude);
        }
        else if (loraIn.gsia == GET) // request dock positon
        {
            double tmplat;
            double tmplon;
            MemoryDockPos(&tmplat, &tmplon, true);
            msg = String(tmplat, 8) + "." + String(tmplon, 8);
            sendACKNAKINF(msg, ACK);
        }
        break;
    case TARGET_POSITION:
        if (loraIn.gsia == SET)
        {
            if ((gpsdata.lat != 0 || gpsdata.lon != 0) && gpsdata.fix == true)
            {
                initRudderPid();
                buoy.tglatitude = gpsdata.lat;
                buoy.tglongitude = gpsdata.lon;
                rudderpid.iintergrate = 0;
                speedpid.armIntergrator = false;
                sendACKNAKINF("", ACK);
                status = LOCKED;
            }
            else
            {
                sendACKNAKINF("", NAK);
                status = IDLE;
            }
        }
        else if (loraIn.gsia == GET)
        {
            msg = String(buoy.tglatitude, 8) + "," + String(buoy.tglongitude, 8);
            sendACKNAKINF(msg, SET);
        }
        break;
    case SAIL_DIR_SPEED:
        if (loraIn.gsia == SET)
        {
            status = REMOTE;
            sscanf(messageArr, "%d,%d", &buoy.cdir, &buoy.cspeed);
            CalcRemoteRudderBuoy(buoy.cdir, 0, buoy.cspeed, &buoy.speedbb, &buoy.speedsb); // calculate power to thrusters
            msg = String(buoy.mheading, 0) + "," + String(buoy.cspeed) + "," + String(buoy.speedbb) + "," + String(buoy.speedsb);
            sendACKNAKINF(msg, ACK);
        }
        else if (loraIn.gsia == GET)
        {
            msg = String(buoy.mheading, 0) + "," + String(buoy.cspeed) + "," + String(buoy.speedbb) + "," + String(buoy.speedsb);
            sendACKNAKINF(msg, ACK);
        }
        break;

    case BUOY_MODE_IDLE:
        if (loraIn.gsia == SET)
        {
            status = IDLE;
            sendACKNAKINF("", ACK);
        }
        break;

    case NO_POSITION:
        if (loraIn.gsia == SET)
        {
            status = IDLE;
            sendACKNAKINF("", ACK);
        }
        break;

    case SYSTEM_STASTUS:
        if (loraIn.gsia == SET)
        {
            int tmp;
            sscanf(messageArr, "%d", &tmp);
            status = tmp;
            sendACKNAKINF("", ACK);
        }
        break;

    case GPS_DUMMY: // Enable,Disable gps reciever
        if (loraIn.gsia == SET)
        {
            int tmp;
            sendACKNAKINF("", ACK);
            sscanf(messageArr, "%d", &tmp);
            Serial.println(tmp);
            if (tmp == 1)
            {
                gpsactive = false;
                Serial.println("GPS disabled");
            }
            else
            {
                gpsactive = true;
                Serial.println("GPS enabled");
            }
        }
        break;

    case GPS_DUMMY_DELTA_LAT_LON: // add offset to gps position
        if (loraIn.gsia == SET)
        {
            sendACKNAKINF("", ACK);
            Serial.print("Old value>");
            msg = String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8) + "," + String(gpsdata.fix) + "," + String((int)gpsdata.cource) + "," + String((int)gpsdata.speed) + "," + String(buoy.mheading, 0);
            Serial.println(msg);
            double lat, lon;
            sscanf(messageArr, "%lf,%lf", &lat, &lon);
            gpsdata.lat += lat;
            gpsdata.lon += lon;
            msg = String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8) + "," + String(gpsdata.fix) + "," + String((int)gpsdata.cource) + "," + String((int)gpsdata.speed) + "," + String(buoy.mheading, 0);
            Serial.print("New value>");
            Serial.println(msg);
            Serial.println();
        }
        break;
    case COMPUTE_PARAMETERS:
        if (loraIn.gsia == SET)
        {
            int mmind;
            int mmaxd;
            int mminsp;
            int mmaxsp;

            sscanf(messageArr, "%d,%d,%d,%d", &mmind, &mmaxd, &mminsp, &mmaxsp);
            /*
            sanety check
            */
            if (mmind > mmaxd)
            {
                break;
            }
            setparameters(&mmind, &mmaxd, &mminsp, &mmaxsp);
            msg = String(buoy.minOfsetDist) + "," + String(buoy.maxOfsetDist) + "," + String(buoy.minSpeed) + "," + String(buoy.maxSpeed);
            sendACKNAKINF(msg, ACK);
        }
        else if (loraIn.gsia == GET)
        {
            msg = String(buoy.minOfsetDist) + "," + String(buoy.maxOfsetDist) + "," + String(buoy.minSpeed) + "," + String(buoy.maxSpeed);
            sendACKNAKINF(msg, ACK);
        }
        break;
    case PID_SPEED_PARAMETERS:
        if (loraIn.gsia == SET)
        {
            double tp;
            double ti;
            double td;
            double n;
            sscanf(messageArr, "%lf,%lf,%lf,%lf", &tp, &ti, &td, &n);
            /*
            sanety check
            */
            if (tp < 0 || tp > 30)
            {
                break;
            }
            if (ti < 0 || ti > 30)
            {

                break;
            }
            if (td < 0 || td > 30)
            {
                break;
            }

            speedpid.kp = tp;
            speedpid.ki = ti;
            speedpid.kd = td;
            speedpid.i = 0;
            speedpid.d = 0;
            pidSpeedParameters(&speedpid.kp, &speedpid.ki, &speedpid.kd, false);
            msg = String(speedpid.kp, 3) + "," + String(speedpid.ki, 3) + "," + String(speedpid.kd, 3), "," + String(speedpid.i, 3);
            sendACKNAKINF(msg, ACK);
        }
        else if (loraIn.gsia == GET)
        {
            msg = String(speedpid.kp, 3) + "," + String(speedpid.ki, 3) + "," + String(speedpid.kd, 3), "," + String(speedpid.i, 3);
            sendACKNAKINF(msg, ACK);
        }
        break;
    case PID_RUDDER_PARAMETERS:
        if (loraIn.gsia == SET)
        {
            double tp;
            double ti;
            double td;
            double n;
            sscanf(messageArr, "%lf,%lf,%lf,%lf", &tp, &ti, &td, &n);
            /*
            sanety check
            */
            if (tp < 0 || tp > 30)
            {
                break;
            }
            if (ti < 0 || ti > 30)
            {

                break;
            }
            if (td < 0 || td > 30)
            {
                break;
            }

            rudderpid.kp = tp;
            rudderpid.ki = ti;
            rudderpid.kd = td;
            rudderpid.i = 0;
            rudderpid.d = 0;
            pidRudderParameters(&rudderpid.kp, &rudderpid.ki, &rudderpid.kd, false);
            msg = String(rudderpid.kp, 3) + "," + String(rudderpid.ki, 3) + "," + String(rudderpid.kd, 3), "," + String(rudderpid.i, 3);
            sendACKNAKINF(msg, ACK);
        }
        else if (loraIn.gsia == GET)
        {
            msg = String(rudderpid.kp, 3) + "," + String(rudderpid.ki, 3) + "," + String(rudderpid.kd, 3), "," + String(rudderpid.i, 3);
            sendACKNAKINF(msg, ACK);
        }
        break;
    case CHANGE_LOCK_POS_DIR_DIST:
        if (loraIn.gsia == SET)
        {
#ifndef DEBUG
            if (gpsdata.fix == true)
#endif
            {
                int direction;
                double distance;
                sscanf(messageArr, "%d,%lf", &direction, &distance);
                direction = buoy.mheading + direction;
                if (direction >= 360)
                {
                    direction -= 360;
                }
                else if (direction < 0)
                {
                    direction += 360;
                }
                double tlat = gpsdata.lat;
                double tlon = gpsdata.lon;
                adjustPositionDirDist(direction, distance, &tlat, &tlon);
                buoy.tglatitude = tlat;
                buoy.tglongitude = tlon;
                rudderpid.iintergrate = 0;
                msg = String(buoy.tglatitude, 8) + "," + String(buoy.tglongitude, 8);
                sendACKNAKINF(msg, ACK);
                delay(250);
                Serial.printf("Current magnetic heading=%0.0f° adjust angle=%d° Distance=%0.2lf Meter\r\n", buoy.mheading, direction, distance);
                loraOut.message = String((int)buoy.tgdir) + "," + String(buoy.tgdistance, 1);
                loraOut.destination = loraIn.recipient;
                loraOut.msgid = DIR_DISTANSE_TO_TARGET_POSITION;
                loraOut.gsia = SET;
                status = LOCKED;
                speedpid.armIntergrator = false;
                while (sendLora())
                    ;
            }
            else
            {
                sendACKNAKINF("", NAK);
            }
        }
        break;

    case ESC_ON_OFF:
        int tmp;
        sscanf(messageArr, "%d", &tmp);
        buoy.muteEsc = tmp;
        if (tmp)
        {
            Serial.println("ESC off");
        }
        else
        {
            Serial.println("ESC on");
        }
        sendACKNAKINF("", ACK);
        break;
    case LINEAR_CALIBRATE_MAGNETIC_COMPASS:
        if (gpsdata.fix == true)
        {
            status = LINEAR_CAL;
            sendACKNAKINF("", ACK);
        }
        else
        {
            sendACKNAKINF("", NAK);
        }
        break;
    case COMPASS_OFSET:
        sscanf(messageArr, "%d", &tmpint);
        if (tmpint >= -180 && tmpint <= 180)
        {
            buoy.magneticCorrection = tmpint;
            CompassOffsetCorrection(&buoy.magneticCorrection, false); // store new offset
            sendACKNAKINF("", ACK);
        }
        else
        {
            sendACKNAKINF("", NAK);
        }
        break;
    case MECANICAL_OFSET:
        sscanf(messageArr, "%d", &tmpint);
        if (tmpint >= -30 && tmpint <= 30)
        {
            buoy.mechanicCorrection = tmpint;
            MechanicalCorrection(&buoy.mechanicCorrection, false); // store new offset
            sendACKNAKINF("", ACK);
        }
        else
        {
            sendACKNAKINF("", NAK);
        }
        break;

    case RESET:
        ESP.restart();
        break;
    default:
        Serial.println("unknown command");
        break;
    }
    return 0;
}

void LoraTask(void *arg)
{
    Mlora ldata;
    loradataout = xQueueCreate(10, sizeof(ldata));
    while (1)
    {
        if (loraOK)
        {
            polLora();
        }
        if (xQueueReceive(loradataout, (void *)&ldata, 0) == pdTRUE)
        {
            int cmnd = ldata.data;
            switch (cmnd)
            {
            case GPS_LAT_LON_NRSAT_FIX_HEADING_SPEED_MHEADING:
                loraOut.message = String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8) + "," + String(gpsdata.nrsats) + "," + String(gpsdata.fix) + "," + String((int)gpsdata.cource) + "," + String((int)gpsdata.speed) + "," + String(buoy.mheading);
                loraOut.destination = loraIn.recipient;
                loraOut.msgid = cmnd;
                loraOut.gsia = SET;
                while (sendLora())
                    delay(10);
                ;
                break;
            case BATTERY_VOLTAGE_PERCENTAGE:
                loraOut.message = String(buoy.vbatt, 1) + "," + String(buoy.vperc);
                loraOut.destination = loraIn.recipient;
                loraOut.msgid = cmnd;
                loraOut.gsia = SET;
                while (sendLora())
                    delay(10);
                ;
                break;
            case DIR_DISTANSE_TO_TARGET_POSITION:
                loraOut.message = String((int)buoy.tgdir) + "," + String(buoy.tgdistance, 1);
                loraOut.destination = loraIn.recipient;
                loraOut.msgid = cmnd;
                loraOut.gsia = SET;
                while (sendLora())
                    delay(10);
                ;
                break;
            case DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_M_HEADING:
                loraOut.message = String(buoy.tgdir) + "," + String(buoy.tgdistance) + "," + buoy.speed + "," + buoy.speedbb + "," + buoy.speedsb + "," + String(buoy.mheading, 0);
                loraOut.destination = loraIn.recipient;
                loraOut.msgid = cmnd;
                loraOut.gsia = SET;
                while (sendLora())
                    delay(10);
                ;
                break;
            case SBPWR_BBPWR:
                loraOut.message = String(buoy.speedbb) + "," + String(buoy.speedsb);
                loraOut.destination = loraIn.recipient;
                loraOut.msgid = cmnd;
                loraOut.gsia = SET;
                while (sendLora())
                    delay(10);
                ;
                break;
            case COMPUTE_PARAMETERS:
                loraOut.msgid = cmnd;
                loraOut.message = String(buoy.minOfsetDist) + "," + String(buoy.maxOfsetDist) + "," + String(buoy.minSpeed) + "," + String(buoy.maxSpeed) + "," + String(buoy.magneticCorrection) + "," + String(buoy.mechanicCorrection);
                loraOut.gsia = SET;
                while (sendLora())
                    delay(10);
                ;
                break;
            case PID_SPEED_PARAMETERS:
                loraOut.msgid = cmnd;
                loraOut.message = String(speedpid.kp) + "," + String(speedpid.ki, 4) + "," + String(speedpid.kd) + "," + String(speedpid.i);
                loraOut.gsia = SET;
                while (sendLora())
                    delay(10);
                ;
                break;
            case PID_RUDDER_PARAMETERS:
                loraOut.msgid = cmnd;
                loraOut.message = String(rudderpid.kp) + "," + String(rudderpid.ki, 4) + "," + String(rudderpid.kd) + "," + String(rudderpid.i);
                loraOut.gsia = SET;
                while (sendLora())
                    delay(10);
                ;
                break;
            case WIND_DIR_DEV:
                deviationWindRose(buoy.winddir, BUFLENMHRG);
                loraOut.msgid = cmnd;
                loraOut.message = String(buoy.winddir[0], 0) + "," + String(buoy.winddir[1], 0);
                loraOut.gsia = SET;
                while (sendLora())
                    delay(10);
                ;
                break;
            case MAGNETIC_HEADING:
                loraOut.msgid = cmnd;
                loraOut.message = String(buoy.mheading, 0);
                loraOut.gsia = SET;
                while (sendLora())
                    delay(10);
                ;
                break;
            }
        }
        delay(1);
    }
}
