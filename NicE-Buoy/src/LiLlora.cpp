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
    LoRa.write(loraOut.messagelength);
    LoRa.print(loraOut.message);
    LoRa.endPacket(); // finish packet and send it
    //Serial.println(loraOut.message);
}

void sendLoraPos(int id, double lat, double lon)
{
    String msg = String(lat, 8) + "," + String(lon, 8);
    loraOut.destination = loraIn.sender;
    loraOut.sender = buoyID;
    loraOut.id = id; // set bit if is answer
    loraOut.messagelength = msg.length();
    loraOut.message = msg;
    sendLora();
}

void sendLoraDirDistanceTarget(unsigned long tgdir, unsigned long tgdistance)
{
    String msg = String(tgdir) + "," + String(tgdistance);
    // loraOut.destination = loraIn.sender;
    loraOut.destination = 0xFF;
    loraOut.sender = buoyID;
    loraOut.id = DIR_DISTANSE_TO_TARGET_POSITION;
    loraOut.messagelength = msg.length();
    loraOut.message = msg;
    sendLora();
}

void sendLoraDirDistanceSbSpeedBbSpeedTarget(unsigned long tgdir, unsigned long tgdistance, int sp, int sb, int bb, int heading)
{
    String msg = String(tgdir) + "," + String(tgdistance) + "," + sp + "," + sb + "," + bb + "," + String(heading);
    loraOut.destination = 0xFF;
    loraOut.sender = buoyID;
    loraOut.id = DIR_DISTANSE_SPEED_SBSPPEED_BBSPEED_TARGET_POSITION;
    loraOut.messagelength = msg.length();
    loraOut.message = msg;
    sendLora();
}

int polLora(void)
{
    if (LoRa.parsePacket() == 0)
        return 0; // if there's no packet, return

    // read packet header bytes:
    int recipient = LoRa.read();       // recipient address
    byte sender = LoRa.read();         // sender address
    byte incomingMsgId = LoRa.read();  // incoming msg ID
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
    loraIn.messagelength = incomingLength;
    loraIn.message = incoming;
    loraIn.rssi = LoRa.packetRssi();
    loraIn.snr = LoRa.packetSnr();
    return incomingMsgId;
}
