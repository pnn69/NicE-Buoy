/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
*/
#include <Arduino.h>
#include <SPI.h> // include libraries
#include <LoRa.h>
#include "io.h"
#include "LiLlora.h"
#include "gps.h"
#include "general.h"
#include "webinterface.h"
//#include "RoboCompute.h"
#include "../../dependency/command.h"


int counter = 0;

const int csPin = 18;    // LoRa radio chip select
const int resetPin = 23; // LoRa radio reset
const int irqPin = 26;   // change for your board; must be a hardware interrupt pin

static unsigned long lasttransmission = 0;

String outgoing;          // outgoing message
byte msgCount = 0;        // count of outgoing messages
byte localAddress = 0xFE; // address of this device
byte destination = 0x01;  // destination to send to

struct loraDataType loraIn;
struct loraDataType loraOut;


bool loraOK = false;

bool InitLora(void)
{
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    Serial.print("LoRa setup ");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LoRa_frequency))
    {
        Serial.println(" failed!");
        return false;
    }
    Serial.println(" Succes!");
    loraOK = true;
    return true;
}

void sendMessage(String outgoing, byte dest, byte msg_id, byte gsia)
{
    Serial.println("<Lora out dest:" + String(dest) + "><id:" + String(msg_id) + "><gsia:" + String(gsia) + "><" + outgoing + ">");
    ledstatus = true;
    LoRa.beginPacket();            // start packet
    LoRa.write(dest);              // add destination address
    LoRa.write(localAddress);      // add sender address
    LoRa.write(0);                 // status
    LoRa.write(msg_id);            // add message ID
    LoRa.write(gsia);              // status
    LoRa.write(outgoing.length()); // add payload length
    LoRa.print(outgoing);          // add payload
    LoRa.endPacket();              // finish packet and send it
    lasttransmission = millis();
}

int decodeMsg()
{
    bool ret = false;
    // read packet header bytes:
    int recipient = LoRa.read();                                  // address send to
    // if (recipient != 0xFF && recipient != 0xFE && recipient != 1) // if the recipient isn't this device or broadcast,
    // {
    //     // Serial.print("<");
    //     // Serial.print(recipient);
    //     // Serial.println(":Not for me!>");
    //     // ret = false; // skip rest of function
    // }
    // else
    // {
    ret = true;
    // }
    byte sender_l = LoRa.read();         // sender address
    byte status_l = LoRa.read();         // status
    byte incomingMsgId_l = LoRa.read();  // msg ID
    byte gsia_l = LoRa.read();           // get set info
    byte incomingLength_l = LoRa.read(); // msg length
    String incoming = "";
    while (LoRa.available())
    {
        incoming += (char)LoRa.read();
    }
    if (incomingLength_l != incoming.length())
    {                 // check length for error
        return false; // skip rest of function
    }
    // Get and store the data
    loraIn.sender = sender_l;
    loraIn.status = status_l;
    loraIn.msgid = incomingMsgId_l;
    loraIn.gsia = gsia_l;
    loraIn.messagelength = incomingLength_l;
    loraIn.message = incoming;
    loraIn.rssi = LoRa.packetRssi();
    loraIn.snr = LoRa.packetSnr();
    Serial.printf("<%d><%d><%d><%d><", sender_l, status_l, incomingMsgId_l, gsia_l);
    Serial.println(incoming + ">");
    // if (SerialBT.connected())
    // {
    //     SerialBT.printf("<%d><%d><%d><%d><", sender_l, status_l, incomingMsgId_l, gsia_l);
    //     SerialBT.println(incoming + ">");
    // }
    return ret;
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

int polLora(void)
{
    if (loraOK == false)
    {
        return 0;
    }
    if (LoRa.parsePacket() == 0)
        return 0; // if there's no packet, return
    if (decodeMsg() == 0)
    {
        return 0;
    }

    String decode = loraIn.message;
    char messarr[100];
    double dir, dist;
    int sp, sb, bb;
    float lhe;
    String gsia = "";
    if (loraIn.gsia == ACK)
    {
        gsia = "ACK";
    }
    else if (loraIn.gsia == NAK)
    {
        gsia = "NAK";
    }
    else if (loraIn.gsia == GET)
    {
        gsia = "GET";
    }
    else if (loraIn.gsia == SET)
    {
        gsia = "SET";
    }
    else if (loraIn.gsia == INF)
    {
        gsia = "INF";
    }
    // Serial.println("Lora in Buoy:" + String(loraIn.sender) + " status:" + String(loraIn.status) + " gsia:" + gsia + " msgid:" + String(loraIn.msgid) + " <" + loraIn.message + ">");
    loraIn.message = removeWhitespace(loraIn.message);
    loraIn.message.toCharArray(messarr, loraIn.message.length() + 1);
    if (NR_BUOYS > loraIn.sender)
    {
        buoy[loraIn.sender].remotestatus = loraIn.status;
        buoy[loraIn.sender].status = loraIn.status;
        buoy[loraIn.sender].rssi = loraIn.rssi;
        if (loraIn.status == IDLE)
        {
            buoy[loraIn.sender].tgdir = 0;
            buoy[loraIn.sender].tgdistance = 0;
            buoy[loraIn.sender].speed = 0;
            buoy[loraIn.sender].speedbb = 0;
            buoy[loraIn.sender].speedsb = 0;
        }
        switch (loraIn.msgid)
        {
        case DIR_DISTANSE_TO_TARGET_POSITION:
            if (loraIn.gsia == SET)
            {
                sscanf(messarr, "%lf,%lf", &dir, &dist);
                buoy[loraIn.sender].tgdir = dir;
                buoy[loraIn.sender].tgdistance = dist;
            }
            break;
        case DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_M_HEADING:
            if (loraIn.gsia == SET)
            {
                // Serial.println("direction and distance target recieved!");
                sscanf(messarr, "%lf,%lf,%d,%d,%d,%d",
                       &buoy[loraIn.sender].tgdir,
                       &buoy[loraIn.sender].tgdistance,
                       &buoy[loraIn.sender].speed,
                       &buoy[loraIn.sender].speedbb,
                       &buoy[loraIn.sender].speedsb,
                       &buoy[loraIn.sender].mdir);
            }
            else if (loraIn.gsia == ACK)
            {
                // Serial.println("direction and distance target recieved!");
                sscanf(messarr, "%lf,%lf,%d,%d,%d,%d",
                       &buoy[loraIn.sender].tgdir,
                       &buoy[loraIn.sender].tgdistance,
                       &buoy[loraIn.sender].speed,
                       &buoy[loraIn.sender].speedbb,
                       &buoy[loraIn.sender].speedsb,
                       &buoy[loraIn.sender].mdir);
                buoy[loraIn.sender].ackOK = true;
            }
            break;

        case GPS_LAT_LON_NRSAT_FIX_HEADING_SPEED_MHEADING:
            if (loraIn.gsia == SET)
            {
                sscanf(messarr, "%lf,%lf,%d,%d,%d,%d,%d",
                       &buoy[loraIn.sender].gpslatitude,
                       &buoy[loraIn.sender].gpslongitude,
                       &buoy[loraIn.sender].nrsats,
                       &buoy[loraIn.sender].fix,
                       &buoy[loraIn.sender].gpscource,
                       &buoy[loraIn.sender].gpsspeed,
                       &buoy[loraIn.sender].mdir);
            }
            else if (loraIn.gsia == ACK)
            {
                buoy[loraIn.sender].ackOK = true;
            }
            break;

        case SAIL_DIR_SPEED:
            if (loraIn.gsia == SET || loraIn.gsia == ACK)
            {
                sscanf(messarr, "%f,%d,%d,%d", &lhe, &sp, &bb, &sb);
                buoy[loraIn.sender].tgdir = 0;
                buoy[loraIn.sender].mdir = (int)lhe;
                buoy[loraIn.sender].tgdistance = 0;
                buoy[loraIn.sender].speed = sp;
                buoy[loraIn.sender].speedsb = sb;
                buoy[loraIn.sender].speedbb = bb;
                if (buoy[loraIn.sender].cmnd == SAIL_DIR_SPEED)
                {
                    buoy[loraIn.sender].ackOK = true;
                }
                // Serial.printf("direction and speed bb:%d sb:%d!",buoy[loraIn.sender].speedbb,buoy[loraIn.sender].speedsb);
            }
            break;

        case SBPWR_BBPWR:
            if (loraIn.gsia == SET || loraIn.gsia == ACK)
            {

                sscanf(messarr, "%d,%d", &sb, &bb);
                buoy[loraIn.sender].speedsb = sb;
                buoy[loraIn.sender].speedbb = bb;
            }
            break;

        case (TARGET_POSITION):
            if (loraIn.gsia == ACK && buoy[loraIn.sender].cmnd == TARGET_POSITION)
            {
                Serial.println("<Position locked!>");
                buoy[loraIn.sender].ackOK = true;
            }
            break;

        case (GOTO_TARGET_POSITION):
            if (loraIn.gsia == ACK && buoy[loraIn.sender].cmnd == GOTO_TARGET_POSITION)
            {
                buoy[loraIn.sender].ackOK = true;
            }
            break;

        case STORE_DOC_POSITION:
            if (loraIn.gsia == ACK)
            {
                Serial.println("<Doc position SET!>");
                buoy[loraIn.sender].ackOK = true;
                buoy[loraIn.sender].status = IDLE;
            }
            break;

        case STORE_POS_AS_DOC_POSITION:
            if (loraIn.gsia == ACK && buoy[loraIn.sender].cmnd == STORE_POS_AS_DOC_POSITION)
            {
                Serial.println("<Doc position stored!>");
                buoy[loraIn.sender].ackOK = true;
                buoy[loraIn.sender].status = IDLE;
            }
            break;

        case (BUOY_MODE_IDLE):
            if (loraIn.gsia == ACK)
            {
                buoy[loraIn.sender].status = IDLE;
                if (buoy[loraIn.sender].cmnd == BUOY_MODE_IDLE)
                {
                    buoy[loraIn.sender].ackOK = true;
                }
            }
            break;
        case DOC_POSITION:
            if (loraIn.gsia == ACK)
            {
                Serial.println("<Doc position locked!>");
                sscanf(messarr, "%lf,%lf",
                       &buoy[loraIn.sender].gpslatitude,
                       &buoy[loraIn.sender].gpslongitude);
                Serial.printf("<Doc positon: https://www.google.nl/maps/@%0.8lf,%0.8lf,16z?entry=ttu>\r\n", buoy[loraIn.sender].gpslatitude, buoy[loraIn.sender].gpslongitude);
                buoy[loraIn.sender].status = DOCKED;
                if (buoy[loraIn.sender].cmnd == DOC_POSITION)
                {
                    buoy[loraIn.sender].ackOK = true;
                }
            }
            break;

        case (BATTERY_VOLTAGE_PERCENTAGE):
            if (loraIn.gsia == SET)
            {
                sscanf(messarr, "%f,%d", &lhe, &sp);
                buoy[loraIn.sender].voltage = lhe;
                buoy[loraIn.sender].percentage = sp;
            }
            break;
        case GPS_DUMMY:
            if (loraIn.gsia == ACK)
            {
                if (buoy[loraIn.sender].cmnd == GPS_DUMMY)
                {
                    buoy[loraIn.sender].ackOK = true;
                }
            }
            break;
        case GPS_DUMMY_DELTA_LAT_LON:
            if (loraIn.gsia == ACK)
            {
                buoy[loraIn.sender].ackOK = true;
            }
            break;
        case ESC_ON_OFF:
            if (loraIn.gsia == ACK)
            {
                buoy[loraIn.sender].ackOK = true;
            }
            break;

        case COMPUTE_PARAMETERS:
            if (loraIn.gsia == ACK)
            {
                sscanf(messarr, "%d,%d,%d,%d", &buoy[loraIn.sender].minOfsetDist, &buoy[loraIn.sender].maxOfsetDist, &buoy[loraIn.sender].minSpeed, &buoy[loraIn.sender].maxSpeed);
                if (buoy[loraIn.sender].cmnd == COMPUTE_PARAMETERS)
                {
                    // Serial.printf("Stored Parameters: Minimum offset distance: %dM Maxumum offset distance: %dM, buoy minimum speed: %d%%, buoy maximum speed: %d%%\r\n", buoy[loraIn.sender].minOfsetDist, buoy[loraIn.sender].maxOfsetDist, buoy[loraIn.sender].minSpeed, buoy[loraIn.sender].maxSpeed);
                    buoy[loraIn.sender].ackOK = true;
                }
            }
            break;
        case PID_SPEED_PARAMETERS:
            if (loraIn.gsia == ACK)
            {
                sscanf(messarr, "%lf,%lf,%lf,%lf", &buoy[loraIn.sender].p, &buoy[loraIn.sender].i, &buoy[loraIn.sender].d, &buoy[loraIn.sender].ki);
                if (buoy[loraIn.sender].cmnd == PID_SPEED_PARAMETERS)
                {
                    buoy[loraIn.sender].ackOK = true;
                }
            }
            if (loraIn.gsia == SET)
            {
                sscanf(messarr, "%lf,%lf,%lf,%lf", &buoy[loraIn.sender].p, &buoy[loraIn.sender].i, &buoy[loraIn.sender].d, &buoy[loraIn.sender].ki);
                if (buoy[loraIn.sender].cmnd == PID_RUDDER_PARAMETERS)
                {
                    buoy[loraIn.sender].ackOK = true;
                }
            }
            break;
        case CHANGE_LOCK_POS_DIR_DIST:
            if (loraIn.gsia == ACK)
            {
                if (buoy[loraIn.sender].cmnd == CHANGE_LOCK_POS_DIR_DIST)
                {
                    buoy[loraIn.sender].ackOK = true;
                }
            }
            break;
        case COMPASS_OFSET:
            if (loraIn.gsia == ACK)
            {
                buoy[loraIn.sender].ackOK = true;
            }

            break;
        case MECANICAL_OFSET:
            if (loraIn.gsia == ACK)
            {
                buoy[loraIn.sender].ackOK = true;
            }

            break;

        default:
            // Serial.println("<unknown command:" + loraIn.msgid + '>');
            break;
        }
        notify = loraIn.sender;
    }

    return 1;
}

/*
    Menu structure
    Send command to buoy
    but prefent to many transmissions in a short time
*/

bool loraMenu(int buoy_nr)
{
    if (lasttransmission + 175 > millis())
    { // prefend fast transmissions
        return 1;
    }
    String msg = "";
    buoy[buoy_nr].ackOK = false;
    checkAckStamp = 500 + millis();
    switch (buoy[buoy_nr].cmnd)
    {
    case LOCKED:
        break;

    case SAIL_DIR_SPEED:
        if (buoy[buoy_nr].gsa == SET)
        {
            msg = String(buoy[buoy_nr].cdir) + "," + String(buoy[buoy_nr].cspeed);
        }
        else
        {
            msg = "";
        }
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case SBPWR_BBPWR:
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_M_HEADING:
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case TARGET_POSITION:
        msg = "";
        loraOut.gsia = SET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case GOTO_TARGET_POSITION:
        msg = "";
        loraOut.gsia = SET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case STORE_DOC_POSITION:
        msg = String(buoy[buoy_nr].doclatitude, 8) + "," + String(buoy[buoy_nr].doclongitude, 8);
        loraOut.msgid = STORE_DOC_POSITION;
        loraOut.gsia = SET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case STORE_POS_AS_DOC_POSITION:
        loraOut.msgid = STORE_POS_AS_DOC_POSITION;
        loraOut.gsia = SET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case DOC_POSITION:
        msg = "";
        loraOut.gsia = SET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case RESET:
        loraOut.msgid = RESET;
        loraOut.gsia = INF;
        msg = "";
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case BUOY_MODE_IDLE:
        msg = "";
        loraOut.gsia = SET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case NO_POSITION:
        msg = "";
        loraOut.gsia = SET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;
    case GPS_DUMMY:
        if (buoy[1].dataout == 1)
        {
            msg = "1";
            buoy[1].ackOK = true;
            buoy[1].gsa = SET;
            sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        }
        if (buoy[1].dataout == 0)
        {
            msg = "0";
            buoy[1].ackOK = true;
            buoy[1].gsa = SET;
            sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        }
    case GPS_DUMMY_DELTA_LAT_LON:
        if (buoy[1].dataout == 1)
        {
            msg = "0.00001,0.00001";
            buoy[1].ackOK = true;
            buoy[1].gsa = SET;
            sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        }
        if (buoy[1].dataout == 0)
        {
            msg = "-0.00001,-0.00001";
            buoy[1].ackOK = true;
            buoy[1].gsa = SET;
            sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        }
        break;
    case CHANGE_LOCK_POS_DIR_DIST:
        msg = buoy[1].string;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;
    case ESC_ON_OFF:
        buoy[1].ackOK = false;
        buoy[1].gsa = SET;
        if (buoy[1].dataout == 1)
        {
            msg = "1";
        }
        if (buoy[1].dataout == 0)
        {
            msg = "0";
        }
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case PID_SPEED_PARAMETERS:
        msg = buoy[1].string;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;
    case PID_RUDDER_PARAMETERS:
        msg = buoy[1].string;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case COMPUTE_PARAMETERS:
        msg = buoy[1].string;
        buoy[1].gsa = SET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case LINEAR_CALIBRATE_MAGNETIC_COMPASS:
        msg = buoy[1].string;
        buoy[1].gsa = SET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    case COMPASS_OFSET:
        msg = buoy[1].string;
        buoy[1].gsa = SET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;
    case MECANICAL_OFSET:
        msg = buoy[1].string;
        buoy[1].gsa = SET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].cmnd, buoy[buoy_nr].gsa);
        break;

    default:
        Serial.printf("Unkown Lora command! buoynr: %d cmnd:%d\r\n", buoy_nr, buoy[buoy_nr].cmnd);
        buoy[buoy_nr].ackOK = true;
        break;
    }
    return 0;
}


// //*********************************************************************************************************************************************
// // New lora functions
// //*********************************************************************************************************************************************
// static RoboStruct loraMsgout = {};
// static RoboStruct loraMsgin = {};
// QueueHandle_t loraOut2;
// QueueHandle_t loraIn2;


// //***************************************************************************************************
// //  initialise lora queue
// //***************************************************************************************************
// void initloraqueue(void)
// {
//     loraIn2 = xQueueCreate(10, sizeof(RoboStruct));
//     loraOut2 = xQueueCreate(10, sizeof(RoboStruct));
//     InitLora();
//     Serial.print("#BuoyId=");
//     Serial.println(espMac(), HEX);
// }