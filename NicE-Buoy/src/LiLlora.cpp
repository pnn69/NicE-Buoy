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
    LoRa.write(loraOut.rssi);
    LoRa.write(loraOut.snr);
    LoRa.write(loraOut.messagelength);
    LoRa.print(loraOut.message);
    LoRa.endPacket(); // finish packet and send it
}

void sendLoraPos(bool answ, double lat, double lon)
{
    String msg = String(lat, 8) + "," + String(lon, 8);
    Serial.println(msg);
    loraOut.destination = loraIn.sender;
    loraOut.sender = buoyID;
    if (answ)
    {
        loraOut.id = loraIn.id |= 1UL << 7 * answ; // set bit if is answer
        loraOut.rssi = loraIn.rssi;
        loraOut.snr = loraIn.snr;
    }
    else
    {
        loraOut.id = loraIn.id; // set bit if is answer
        loraOut.rssi = 0;
        loraOut.snr = -1;
    }
    loraOut.messagelength = msg.length();
    loraOut.message = msg;
    sendLora();
}

void sendLoraDirHeadingAncher(bool answ, unsigned long tgdir, unsigned long tgdistance)
{
    String msg = String(tgdir) + "," + String(tgdistance);
    Serial.println(msg);
    loraOut.destination = loraIn.sender;
    loraOut.sender = buoyID;
    if (answ)
    {
        loraOut.id = loraIn.id |= 1UL << 7 * answ; // set bit if is answer
        loraOut.rssi = loraIn.rssi;
        loraOut.snr = loraIn.snr;
    }
    else
    {
        loraOut.id = loraIn.id; // set bit if is answer
        loraOut.rssi = 0;
        loraOut.snr = -1;
    }
    loraOut.messagelength = msg.length();
    loraOut.message = msg;
    sendLora();

}

int onReceive(int packetSize)
{
    if (packetSize == 0)
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

int polLora(void)
{
    return (onReceive(LoRa.parsePacket()));
}