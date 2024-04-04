/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
Lora message: Destination,SenderAddres,MSGID,Length,Message
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
        Serial.println(" failed!");
        return false;
    }
    Serial.println(" Succes!");
    loraOK = true;
    return true;
}

//  Lora message: Destination,SenderAddres,Status,MSGID,gis,Length,Message
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
    Serial.print("Lora in for:" + String(loraIn.recipient) + " from:" + String(loraIn.sender) + " RSSI:" + String(loraIn.rssi) + " msg <" + loraIn.message);
    loraIn.message.toCharArray(messageArr, loraIn.message.length() + 1);
    int index = loraIn.message.indexOf(",");
    loraIn.message = loraIn.message.substring(index + 1);
    loraIn.message.toCharArray(messageArr, loraIn.message.length() + 1);
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
            msg = String(gpsdata.dlat, 8) + "," + String(gpsdata.dlon, 8) + "," + String(status);
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

        // case ANCHOR_POSITION_AS_TARGET_POSITION:
        //     if (loraIn.id == SET)
        //     {
        //         Serial.print("Set target to  ANCHOR position");
        //         GetMemoryAnchorPos(&tglatitude, &tglongitude);
        //         sendLoraPos(UPD, TARGET_POSITION, tglatitude, tglongitude, status);
        //         status = LOCKED;
        //     }
        //     sendLoraPos(UPD, TARGET_POSITION, tglatitude, tglongitude, status);
        //     break;
        // case ANCHOR_POSITION:
        //     if (loraIn.id == SET || gpslatitude != 0 || gpslongitude != 0)
        //     {
        //         SetMemoryAnchorPos(gpslatitude, gpslongitude);
        //         tglatitude = gpslatitude;
        //         tglongitude = gpslongitude;
        //         status = LOCKED;
        //     }
        //     sendLoraPos(UPD, ANCHOR_POSITION, tglatitude, tglongitude, status);
        //     break;

    case GOTO_DOC_POSITION:
        if (loraIn.gsia == SET)
        {
            GetMemoryDockPos(&buoy.tglatitude, &buoy.tglongitude);
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

    // case SET_CURREND_POSITION_AS_ANCHOR_POSITION:
    //     if (gpslatitude != 0 || gpslongitude != 0)
    //     {
    //         SetMemoryAnchorPos(gpslatitude, gpslongitude); // put current position in memory as anchor position
    //         sendLoraAck(UPD, SET_CURREND_POSITION_AS_ANCHOR_POSITION, status);
    //         status = LOCKED;
    //     }
    //     break;
    case TARGET_POSITION:
        if (loraIn.gsia == SET)
        {
            if ((gpsdata.lat != 0 || gpsdata.lon != 0) && gpsdata.fix > 0)
            {
                buoy.tglatitude = gpsdata.lat;
                buoy.tglongitude = gpsdata.lon;
                msg = String(TARGET_POSITION);
                loraOut.sender = loraIn.sender;
                loraOut.id = loraIn.id;
                loraOut.gsia = ACK;
                status = LOCKED;
            }
            else
            {
                msg = String(TARGET_POSITION);
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
            // Serial.printf("ddir:%03d speed:%03d\r\n",ddir,buoy.cspeed);
        }
        msg = String(SAIL_DIR_SPEED) + "," + buoy.mheading + "," + buoy.cspeed + "," + buoy.speedbb + "," + buoy.speedsb;
        // Serial.printf("Mdir:%d cdir:%d BB:%d SB:%d\r\n", buoy.mheading, buoy.cdir, buoy.speedbb, buoy.speedsb);
        loraOut.messagelength = msg.length();
        loraOut.message = msg;
        loraOut.sender = loraIn.sender;
        loraOut.id = loraIn.id;
        loraOut.gsia = ACK;
        while (sendLora())
            ;
        break;

    case BUOY_MODE_IDLE:
        if (loraIn.gsia == SET)
        {
            status = IDLE;
        }
        msg = String(BUOY_MODE_IDLE);
        loraOut.messagelength = msg.length();
        loraOut.message = msg;
        loraOut.sender = loraIn.sender;
        loraOut.id = loraIn.id;
        loraOut.gsia = ACK;
        while (sendLora())
            ;
        break;

    case SYSTEM_STASTUS:
        if (loraIn.gsia == SET)
        {
            sscanf(messageArr, "%d", &status);
        }
        msg = String(SYSTEM_STASTUS);
        loraOut.messagelength = msg.length();
        loraOut.message = msg;
        loraOut.sender = loraIn.sender;
        loraOut.id = loraIn.id;
        loraOut.gsia = INF;
        while (sendLora())
            ;
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
        msg = String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8) + "," + String(gpsdata.fix) + "," + String((int)gpsdata.cource) + "," + String(buoy.mheading);
        loraOut.messagelength = msg.length();
        loraOut.message = msg;
        loraOut.sender = 0xFF; // broadcast addres
        LoraOut.id = GPS_LAT_LON_FIX_HEADING_SPEED_MHEADING;
        loraOut.gsia = INF;
        loraOut.destination = 254;
        while (sendLora())
            ;
        break;
    }
    return 0;
}