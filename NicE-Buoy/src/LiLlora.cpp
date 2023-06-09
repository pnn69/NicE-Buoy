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
#include "../../dependency/command.h"

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

// Lora message: Destination,SenderAddres,MSGID,Length,Message
void sendLora(void)
{
    LoRa.beginPacket();              // start packet
    LoRa.write(loraOut.destination); // add destination address
    LoRa.write(loraOut.sender);      // add sender address
    LoRa.write(loraOut.id);
    LoRa.write(buoy.mheading);
    LoRa.write(loraOut.messagelength);
    LoRa.print(loraOut.message);
    LoRa.endPacket(); // finish packet and send it
    ledstatus = true;
    Serial.print("Lora out Dest:" + String(loraOut.destination) + " id:" + loraOut.id + " status: " + String(status) + " msg <" + String(loraOut.message) + ">");
    if (loraOut.id == ACK)
    {
        Serial.printf(" ACK");
    }
    else if (loraOut.id == SET)
    {
        Serial.printf(" SET");
    }
    else if (loraOut.id == GET)
    {
        Serial.printf(" GET");
    }
    Serial.println();
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
    if (recipient != buoyID && recipient != 0xFF)
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
    float heading = 0;
    char messageArr[100];
    int ddir;
    Serial.print("Lora in for:" + String(loraIn.recipient) + " from:" + String(loraIn.sender) + " RSSI:" + String(loraIn.rssi) + " msg <" + loraIn.message);
    Serial.printf("> Remote status: %d", loraIn.status);
    if (loraIn.id == ACK)
    {
        Serial.printf(" ACK");
    }
    else if (loraIn.id == SET)
    {
        Serial.printf(" SET");
    }
    else if (loraIn.id == GET)
    {
        Serial.printf(" GET");
    }
    Serial.println();
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
            msg = String(cmnd) + String(buoy.gpslatitude, 8) + "," + String(buoy.gpslongitude, 8) + "," + String(status);
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.id = ACK;
            sendLora();
        }
        break;

    case DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION:
        if (loraIn.id == GET)
        {
            msg = String(cmnd) + "," + String(buoy.tgdir) + "," + String(buoy.tgdistance) + "," + buoy.speed + "," + buoy.speedbb + "," + buoy.speedsb + "," + buoy.mheading;
            loraOut.messagelength = msg.length();
            loraOut.message = msg;
            loraOut.id = ACK;
            sendLora();
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
            sendLora();
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
            if (buoy.gpslatitude != 0 || buoy.gpslongitude != 0)
            {
                buoy.tglatitude = buoy.gpslatitude;
                buoy.tglongitude = buoy.gpslongitude;
                msg = String(TARGET_POSITION);
                loraOut.messagelength = msg.length();
                loraOut.message = msg;
                loraOut.id = ACK;
                sendLora();
                status = LOCKED;
            }
        }
        break;
    case SAIL_DIR_SPEED:
        if (loraIn.id == SET)
        {
            sscanf(messageArr, "%d,%d", &ddir, &buoy.cspeed);
            ddir = buoy.mheading - ddir;
            if (ddir < 0)
            {
                ddir = 360 - ddir;
            }
            if (ddir > 360)
            {
                ddir = ddir - 360;
            }
            buoy.cdir = ddir;
            status = REMOTE;
        }
        CalcEngingSpeed(buoy.cdir, (unsigned long)buoy.mheading, buoy.cspeed, &buoy.speedbb, &buoy.speedsb);
        msg = String(SAIL_DIR_SPEED) + "," + buoy.cdir + "," + buoy.cspeed + "," + buoy.speedbb + "," + buoy.speedsb;
        loraOut.id = ACK;
        Serial.printf("cdir:%d Mdir:%d BB:%d SB:%d\r\n",buoy.cdir, (unsigned long)buoy.mheading, buoy.speedbb, buoy.speedsb);
        loraOut.messagelength = msg.length();
        loraOut.message = msg;
        sendLora();
        delay(5000);
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
        sendLora();
        break;

    case SYSTEM_STASTUS:
        if (loraIn.id == SET)
        {
            sscanf(messageArr, "%d", &status);
        }
        msg = String(SYSTEM_STASTUS);
        loraOut.id = ACK;
        loraOut.messagelength = msg.length();
        loraOut.message = msg;
        sendLora();
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
