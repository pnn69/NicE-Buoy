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
#include "../../dependency/command.h"

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
/*
 *Lora message: Destination,SenderAddres,Status,mg_id,gsia,Length,Message
 */
bool sendLora(void)
{
    if (lasttransmission + 175 > millis())
    { // prefend fast transmissions
        return 1;
    }
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
    Serial.print("Lora out desst>:" + String(loraOut.destination) + " status:" + String(status) + " msg_id:" + loraOut.msgid + " gsia:" + String(loraOut.gsia) + " msg <" + String(loraOut.message) + ">\r\n");
    return 0;
}

void sendLoraAck(byte msg_id, int cmnd)
{
    loraOut.msgid = msg_id; // set bit if is answer
    loraOut.gsia = ACK;
    String msg = "";
    loraOut.messagelength = msg.length();
    loraOut.message = msg;
    sendLora();
}

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
    // int index = loraIn.message.indexOf(",");
    // loraIn.message = loraIn.message.substring(index + 1);
    // loraIn.message.toCharArray(messageArr, loraIn.message.length() + 1);
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
                delay(5000);
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
            MemoryDockPos(&buoy.tglatitude, &buoy.tglongitude, true);
            msg = String(buoy.tglatitude, 8) + "." + String(buoy.tglongitude, 8);
            sendACKNAKINF(msg, ACK);
            Serial.printf("\r\n\r\nSaling to: %lfN %lfW\r\n\r\n", buoy.tglatitude, buoy.tglongitude);
            status = LOCKED;
            delay(100);
        }
        if (loraIn.gsia == GET) // request dock positon
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
            if ((gpsdata.lat != 0 || gpsdata.lon != 0) && gpsdata.fix > 0)
            {
                buoy.tglatitude = gpsdata.lat;
                buoy.tglongitude = gpsdata.lon;
                sendACKNAKINF("", ACK);
                status = LOCKED;
            }
            else
            {
                sendACKNAKINF("", NAK);
            }
            loraOut.message = msg;
            while (sendLora())
                ;
        }
        if (loraIn.gsia == GET)
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
            CalcEngingSpeed(buoy.cdir, 0, buoy.cspeed, &buoy.speedbb, &buoy.speedsb);
            msg = String(buoy.mheading, 0) + "," + String(buoy.cspeed) + "," + String(buoy.speedbb) + "," + String(buoy.speedsb);
            sendACKNAKINF(msg, SET);
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
            sendACKNAKINF("", ACK);
        }
        break;

    case GPS_DUMMY_DELTA_LAT_LON: // add offset to gps position
        if (loraIn.gsia == SET)
        {
            sendACKNAKINF("", ACK);
            Serial.println();
            msg = String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8) + "," + String(gpsdata.fix) + "," + String((int)gpsdata.cource) + "," + String((int)gpsdata.speed) + "," + String(buoy.mheading, 0);
            Serial.println(msg);
            double lat, lon;
            sscanf(messageArr, "%lf,%lf", &lat, &lon);
            gpsdata.lat += lat;
            gpsdata.lon += lon;
            msg = String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8) + "," + String(gpsdata.fix) + "," + String((int)gpsdata.cource) + "," + String((int)gpsdata.speed) + "," + String(buoy.mheading, 0);
            Serial.println(msg);
            Serial.println();
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

bool loraMenu(int cmnd)
{
    switch (cmnd)
    {
    case GPS_LAT_LON_FIX_HEADING_SPEED_MHEADING:
        loraOut.message = String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8) + "," + String(gpsdata.fix) + "," + String((int)gpsdata.cource) + "," + String((int)gpsdata.speed) + "," + String(buoy.mheading, 0);
        loraOut.destination = loraIn.recipient;
        loraOut.msgid = cmnd;
        loraOut.gsia = SET;
        while (sendLora())
            ;
        break;
    case BATTERY_VOLTAGE_PERCENTAGE:
        loraOut.message = String(buoy.vbatt, 1) + "," + String(buoy.vperc, 0);
        loraOut.destination = loraIn.recipient;
        loraOut.msgid = cmnd;
        loraOut.gsia = SET;
        while (sendLora())
            ;
        break;
    case DIR_DISTANSE_TO_TARGET_POSITION:
        loraOut.message = String((int)buoy.tgdir) + "," + String((int)buoy.tgdistance);
        loraOut.destination = loraIn.recipient;
        loraOut.msgid = cmnd;
        loraOut.gsia = SET;
        while (sendLora())
            ;
        break;
    case SBPWR_BBPWR:
        loraOut.message = String(buoy.speedsb) + "," + String(buoy.speedbb);
        loraOut.destination = loraIn.recipient;
        loraOut.msgid = cmnd;
        loraOut.gsia = SET;
        while (sendLora())
            ;
        break;
    }
    return 0;
}