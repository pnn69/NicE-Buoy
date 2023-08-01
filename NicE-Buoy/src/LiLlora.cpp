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

//  Lora message: Destination,SenderAddres,MSGID,Length,Message
bool sendLora(void)
{
    if (lasttransmission + 150 > millis())
    { // prefend fast transmissions
        return 1;
    }
    LoRa.beginPacket();              // start packet
    LoRa.write(loraOut.destination); // add destination address
    LoRa.write(loraOut.sender);      // add sender address
    LoRa.write(loraOut.id);
    LoRa.write(status);
    LoRa.write(buoy.mheading);
    LoRa.write(loraOut.messagelength);
    LoRa.print(loraOut.message);
    LoRa.endPacket(); // finish packet and send it
    ledstatus = true;
    Serial.print("Lora out Dest:" + String(loraOut.destination) + " id:" + loraOut.id + " status: " + String(status) + " msg <" + String(loraOut.message) + ">\r\n");
    // if (loraOut.id == ACK)
    // {
    //     Serial.printf(" ACK");
    // }
    // else if (loraOut.id == SET)
    // {
    //     Serial.printf(" SET");
    // }
    // else if (loraOut.id == GET)
    // {
    //     Serial.printf(" GET");
    // }
    // Serial.println();
    lasttransmission = millis();
    return 0;
}

void sendLoraAck(byte id, int cmnd, int status)
{
    String msg = cmnd + String(buoy.mheading);
    loraOut.destination = loraIn.sender;
    loraOut.sender = buoyID;
    loraOut.id = id; // set bit if is answer
    loraOut.messagelength = msg.length();
    loraOut.message = msg;
    sendLora();
}

int decodeMsg()
{
    // read packet header bytes:
    int recipient = LoRa.read();       // recipient address
    byte sender = LoRa.read();         // sender address
    byte incomingMsgId = LoRa.read();  // incoming msg ID
    int sstatus = LoRa.read();         // incoming status (not used yet)
    byte incomingLength = LoRa.read(); // incoming msg length
    String incoming = "";
    while (LoRa.available())
    {
        incoming += (char)LoRa.read();
    }
    if (incomingLength != incoming.length())
    {             // check length for error
        return 0; // skip rest of function
    }
    // if the recipient isn't this device or broadcast,
    if (recipient != buoyID && recipient != 0x0)
    {
        return 0; // skip rest of function
    }
    // Get and store the data
    loraIn.recipient = recipient;
    loraIn.sender = sender;
    loraIn.id = incomingMsgId;
    loraIn.status = sstatus;
    loraIn.messagelength = incomingLength;
    loraIn.message = incoming;
    loraIn.rssi = LoRa.packetRssi();
    loraIn.snr = LoRa.packetSnr();
    return incomingMsgId;
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
    Serial.printf("> Remote status: %d", loraIn.status);
    //  if (loraIn.id == ACK)
    //  {
    //      Serial.printf(" ACK");
    //  }
    //  else if (loraIn.id == SET)
    //  {
    //      Serial.printf(" SET");
    //  }
    //  else if (loraIn.id == GET)
    //  {
    //      Serial.printf(" GET");
    //  }
    //  Serial.println();
    loraIn.message.toCharArray(messageArr, loraIn.message.length() + 1);
    int cmnd = 0;
    sscanf(messageArr, "%d", &cmnd);
    int index = loraIn.message.indexOf(",");
    loraIn.message = loraIn.message.substring(index + 1);
    loraIn.message.toCharArray(messageArr, loraIn.message.length() + 1);
    loraOut.destination = loraIn.sender;
    loraOut.sender = buoyID;
    switch (cmnd)
    {
    case POSITION:
        if (loraIn.id == GET)
        {
            msg = String(cmnd) + String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8) + "," + String(status);
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.id = ACK;
            while (sendLora())
                ;
        }
        break;

    case DGPSPOSITION:
        if (loraIn.id == GET)
        {
            msg = String(cmnd) + String(gpsdata.dlat, 8) + "," + String(gpsdata.dlon, 8) + "," + String(status);
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.id = ACK;
            while (sendLora())
                ;
        }
        break;

    case DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION:
        if (loraIn.id == GET)
        {
            msg = String(cmnd) + "," + String(buoy.tgdir) + "," + String(buoy.tgdistance) + "," + buoy.speed + "," + buoy.speedbb + "," + buoy.speedsb + "," + buoy.mheading;
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.id = ACK;
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
        if (loraIn.id == SET)
        {
            GetMemoryDockPos(&buoy.tglatitude, &buoy.tglongitude);
            msg = String(GOTO_DOC_POSITION);
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.id = ACK;
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
        if (loraIn.id == SET)
        {
            if ((gpsdata.lat != 0 || gpsdata.lon != 0) && gpsdata.fix > 0)
            {

                buoy.tglatitude = gpsdata.lat;
                buoy.tglongitude = gpsdata.lon;
                msg = String(TARGET_POSITION);
                loraOut.id = ACK;
                status = LOCKED;
            }
            else
            {
                msg = String(TARGET_POSITION);
                loraOut.id = NACK;
                status = IDLE;
            }
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            sendLora();
        }
        break;
    case DGPS:
        if (loraIn.id == SET)
        {
            double dlat, dlon;
            sscanf(messageArr, "%lf,%lf", &dlat, &dlon);
            if (dlat < 0.01 && dlon < 0.01)
            {                           // do sanity check
                gpsdata.corrlat = dlat; // correction parameters
                gpsdata.corrlon = dlon;
            }
        }
        break;
    case SAIL_DIR_SPEED:
        if (loraIn.id == SET)
        {
            status = REMOTE;
            sscanf(messageArr, "%d,%d", &buoy.cdir, &buoy.cspeed);
            CalcEngingSpeed(buoy.cdir, 0, buoy.cspeed, &buoy.speedbb, &buoy.speedsb);
            // Serial.printf("ddir:%03d speed:%03d\r\n",ddir,buoy.cspeed);
        }
        msg = String(SAIL_DIR_SPEED) + "," + buoy.mheading + "," + buoy.cspeed + "," + buoy.speedbb + "," + buoy.speedsb;
        loraOut.id = ACK;
        // Serial.printf("Mdir:%d cdir:%d BB:%d SB:%d\r\n", buoy.mheading, buoy.cdir, buoy.speedbb, buoy.speedsb);
        loraOut.messagelength = msg.length();
        loraOut.message = msg;
        while (sendLora())
            ;
        break;

    case BUOY_MODE_IDLE:
        if (loraIn.id == SET)
        {
            status = IDLE;
        }
        msg = String(BUOY_MODE_IDLE);
        loraOut.id = ACK;
        loraOut.messagelength = msg.length();
        loraOut.message = msg;
        while (sendLora())
            ;
        break;

    case SYSTEM_STASTUS:
        if (loraIn.id == SET)
        {
            sscanf(messageArr, "%d", &status);
        }
        msg = String(SYSTEM_STASTUS);
        loraOut.id = INF;
        loraOut.messagelength = msg.length();
        loraOut.message = msg;
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
        msg = String(GPS_LAT_LON_FIX_HEADING_SPEED_MHEADING) + "," + String(gpsdata.lat, 8) + "," + String(gpsdata.lon, 8) + "," + String(gpsdata.fix) + "," + String((int)gpsdata.cource) + "," + String(buoy.mheading);
        loraOut.messagelength = msg.length();
        loraOut.message = msg;
        loraOut.sender = buoyID;
        loraOut.id = ACK;
        loraOut.destination = 254;
        while (sendLora())
            ;
        break;
    }
    return 0;
}