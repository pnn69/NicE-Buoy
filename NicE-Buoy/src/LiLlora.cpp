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
    if (lasttransmission + 150 > millis())
    { // prefend fast transmissions
        return 1;
    }
    LoRa.beginPacket();              // start packet
    LoRa.write(loraOut.destination); // add destination address
    LoRa.write(buoyID);              // add sender address
    LoRa.write(status);              // status
    LoRa.write(loraOut.id);          // id
    LoRa.Write(loraOut.gsi);         // get set inf
    LoRa.write(loraOut.messagelength);
    LoRa.print(loraOut.message);
    LoRa.endPacket(); // finish packet and send it
    lasttransmission = millis();
    ledstatus = true;
    Serial.print("Lora out Dest:" + String(loraOut.destination) + " status: " + String(status) + " msg_id:" + loraOut.id + "gsia" + String(loraOut.gsi) " msg <" + String(loraOut.message) + ">\r\n");
    return 0;
}

void sendLoraAck(byte msg_id, int cmnd)
{
    loraOut.id = msg_id; // set bit if is answer
    loraOut.gsia = ACK;
    loraOut.messagelength = msg.length();
    loraOut.message = msg;
    sendLora();
}

int decodeMsg()
{
    // read packet header bytes:
    int recipient = LoRa.read(); // recipient address
    // if the recipient isn't this device or broadcast,
    if (recipient != buoyID && recipient != 0x0)
    {
        return 0; // skip rest of function
    }
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
    {             // check length for error
        return 0; // skip rest of function
    }

    // Get and store the data
    loraIn.recipient = recipient;
    loraIn.sender = sender_l;
    loraIn.id = incomingMsgId_l;
    loraIn.status = status_l;
    loraIn.messagelength = incomingLength_l;
    loraIn.message = incoming;
    loraIn.rssi = LoRa.packetRssi();
    loraIn.snr = LoRa.packetSnr();
    return incomingMsgId_l;
}

int polLora(void)
{
    String msg = "";
    if (LoRa.parsePacket() == 0)
        return 0; // if there's no packet, return
    // int msg = decodeMsg();
    if (decodeMsg() == 0)
    {
        return 0;
    }
    char messageArr[100];
    Serial.println("Lora in for:" + String(loraIn.recipient) + " from:" + String(loraIn.sender) + " RSSI:" + String(loraIn.rssi) + " msg <" + loraIn.message +">");
    loraIn.message.toCharArray(messageArr, loraIn.message.length() + 1);
    int index = loraIn.message.indexOf(",");
    //loraIn.message = loraIn.message.substring(index + 1);
    //loraIn.message.toCharArray(messageArr, loraIn.message.length() + 1);
    loraOut.destination = loraIn.sender;
    loraOut.sender = buoyID;
    switch (loraIn.id)
    {
    case POSITION:
        if (loraIn.gsia == GET)
        {
            msg = String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8) + "," + String(status);
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.sender = loraIn.sender;
            loraOut.id = loraIn.id;
            loraOut.gsia = ACK;
            while (sendLora())
                ;
        }
        break;

    case DGPSPOSITION:
        if (loraIn.gsia == GET)
        {
            msg = String(gpsdata.dlat, 8) + "," + String(gpsdata.dlon, 8);
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.sender = loraIn.sender;
            loraOut.id = loraIn.id;
            loraOut.gsia = ACK;
            while (sendLora())
                ;
        }
        break;

    case DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION:
        if (loraIn.gsia == GET)
        {
            msg = String(buoy.tgdir) + "," + String(buoy.tgdistance) + "," + buoy.speed + "," + buoy.speedbb + "," + buoy.speedsb + "," + buoy.mheading;
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.sender = loraIn.sender;
            loraOut.id = loraIn.id;
            loraOut.gsia = ACK;
            while (sendLora())
                ;
        }
        break;

    case GOTO_DOC_POSITION:
        if (loraIn.gsia == SET)
        {
            MemoryDockPos(&buoy.tglatitude, &buoy.tglongitude, true);
            msg = String(GOTO_DOC_POSITION);
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.sender = loraIn.sender;
            loraOut.id = loraIn.id;
            loraOut.gsia = ACK;
            status = LOCKED;
            while (sendLora())
                ;
        }
        break;

    case DOC_POSITION:
        if (loraIn.gsia == SET)
        {
            if (gpsdata.fix == true)
            {
                MemoryDockPos(&gpsdata.lat, &gpsdata.lon, false);
                msg = String(gpsdata.lat, 8) + "." + String(gpsdata.lon, 8);
                loraOut.messagelength = msg.length();
                loraOut.message = msg;
                loraOut.sender = loraIn.sender;
                loraOut.id = loraIn.id;
                loraOut.gsia = ACK;
                status = IDLE;
                while (sendLora())
                    ;
            }
        }
        break;

    case TARGET_POSITION:
        if (loraIn.gsia == SET)
        {
            if ((gpsdata.lat != 0 || gpsdata.lon != 0) && gpsdata.fix > 0)
            {
                buoy.tglatitude = gpsdata.lat;
                buoy.tglongitude = gpsdata.lon;
                msg = String();
                loraOut.sender = loraIn.sender;
                loraOut.id = loraIn.id;
                loraOut.gsia = ACK;
                status = LOCKED;
            }
            else
            {
                msg = String();
                loraOut.sender = loraIn.sender;
                loraOut.id = loraIn.id;
                loraOut.gsia = NACK;
                status = IDLE;
            }
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            while (sendLora())
                ;
        }
        if (loraIn.gsia == GET)
        {
            msg = String(gpsdata.tglat, 8) + "," + String(gpsdata.tglon, 8);
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.sender = loraIn.sender;
            loraOut.id = loraIn.id;
            loraOut.gsia = INF;
        }
        break;
    case DGPS:
        if (loraIn.gsia == SET)
        {
            double dlat, dlon;
            sscanf(messageArr, "%lf,%lf", &dlat, &dlon);
            if (dlat < 0.01 && dlon < 0.01)
            {                           // do sanity check
                gpsdata.corrlat = dlat; // correction parameters
                gpsdata.corrlon = dlon;
                loraOut.sender = loraIn.sender;
                loraOut.id = loraIn.id;
                loraOut.gsia = ACK;
                while (sendLora())
                    ;
            }
        }
        break;
    case SAIL_DIR_SPEED:
        if (loraIn.gsia == SET)
        {
            status = REMOTE;
            sscanf(messageArr, "%d,%d", &buoy.cdir, &buoy.cspeed);
            CalcEngingSpeed(buoy.cdir, 0, buoy.cspeed, &buoy.speedbb, &buoy.speedsb);
            msg = String();
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.sender = loraIn.sender;
            loraOut.id = loraIn.id;
            loraOut.gsia = ACK;
            while (sendLora())
                ;
        }
        else if (loraIn.gsia == GET)
        {
            msg = buoy.mheading + "," + buoy.cspeed + "," + buoy.speedbb + "," + buoy.speedsb;
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.sender = loraIn.sender;
            loraOut.id = loraIn.id;
            loraOut.gsia = ACK;
            while (sendLora())
                ;
        }
        break;

    case BUOY_MODE_IDLE:
        if (loraIn.gsia == SET)
        {
            status = IDLE;
            msg = String();
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.sender = loraIn.sender;
            loraOut.id = loraIn.id;
            loraOut.gsia = ACK;
            while (sendLora())
                ;
        }
        break;

    case SYSTEM_STASTUS:
        if (loraIn.gsia == SET)
        {
            sscanf(messageArr, "%d", &status);
            msg = String();
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.sender = loraIn.sender;
            loraOut.id = loraIn.id;
            loraOut.gsia = ACK;
            while (sendLora())
                ;
        }
        break;

    case RESET:
        ESP.restart();
        break;
    default:
        // Serial.println("unknown command");
        break;
    }
    return 0;
}
bool loraMenu(int cmnd)
{
    String msg = "";
    switch (cmnd)
    {
    case GPS_LAT_LON_FIX_HEADING_SPEED_MHEADING:
        msg = String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8) + "," + String(gpsdata.fix) + "," + String((int)gpsdata.cource) + "," + String((int)gpsdata.speed) + "," + String(buoy.mheading, 0);
        loraOut.messagelength = msg.length();
        loraOut.message = msg;
        loraOut.destination = loraIn.recipient;
        loraOut.id = cmnd;
        loraOut.gsia = INF;
        while (sendLora())
            ;
        break;
    case BATTERY_VOLTAGE_PERCENTAGE:
        msg = String(buoy.vbatt, 1) + "," + String(buoy.vperc, 0);
        loraOut.messagelength = msg.length();
        loraOut.message = msg;
        loraOut.destination = loraIn.recipient;
        loraOut.id = cmnd;
        loraOut.gsia = INF;
        while (sendLora())
            ;
        break;
    case DIR_DISTANSE_TO_TARGET_POSITION:
        msg = String(buoy.tgdir, 0) + "," + String(buoy.tgdistance, 0);
        loraOut.messagelength = msg.length();
        loraOut.message = msg;
        loraOut.destination = loraIn.recipient;
        loraOut.id = cmnd;
        loraOut.gsia = INF;
        while (sendLora())
            ;
        break;
    }
    return 0;
}