/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
*/
#include <Arduino.h>
#include <SPI.h> // include libraries
#include <LoRa.h>
#include "io.h"
#include "LiLlora.h"
#include "general.h"

int counter = 0;

const int csPin = 18;    // LoRa radio chip select
const int resetPin = 23; // LoRa radio reset
const int irqPin = 26;   // change for your board; must be a hardware interrupt pin

String outgoing; // outgoing message

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

void sendMessage(String outgoing, int dest, int id)
{
    Serial.print("Lora id:");
    Serial.print(id);
    Serial.print(" msg>");
    Serial.println(outgoing);
    LoRa.beginPacket();            // start packet
    LoRa.write(dest);              // add destination address
    LoRa.write(localAddress);      // add sender address
    LoRa.write(id);                // add message ID
    LoRa.write(outgoing.length()); // add payload length
    LoRa.print(outgoing);          // add payload
    LoRa.endPacket();              // finish packet and send it
}

bool sendLora(int buoy)
{
    String msg = "HeLoRa World!"; // send a msg
    sendMessage(msg, buoy, 0xfe);
    return 1;
}

void sendLoraSetTargetPosition(int buoy)
{
    String msg = ""; // send a message
    sendMessage(msg, buoy, SET_TARGET_POSITION);
}

void sendLoraSetDocPosition(int buoy)
{
    String msg = ""; // send a message
    sendMessage(msg, buoy, SET_DOC_POSITION);
}

void sendLoraGoToDocPosition(int buoy)
{
    String msg = ""; // send a message
    sendMessage(msg, buoy, GOTO_DOC_POSITION);
}

void sendLoraSetSailDirSpeed(int buoy, int tgdir, int speed)
{
    String msg = String(tgdir) + "," + speed;
    sendMessage(msg, buoy, SET_SAIL_DIR_SPEED);
}

void sendLoraSetIdle(int buoy)
{
    String msg = ""; // send a message
    sendMessage(msg, buoy, SET_BUOY_MODE_IDLE);
}

void sendLoraReset(int buoy)
{
    String msg = ""; // send a message
    sendMessage(msg, buoy, RESET);
}

int polLora(void)
{
    if (LoRa.parsePacket() == 0)
        return 0; // if there's no packet, return

    // read packet header bytes:
    int recipient = LoRa.read();       // recipient address
    int sender = LoRa.read();          // sender address
    char incomingMsgId = LoRa.read();  // incoming msg ID
    byte incomingLength = LoRa.read(); // incoming msg length
    String incoming = "";

    while (LoRa.available())
    {
        incoming += (char)LoRa.read();
    }

    if (incomingLength != incoming.length())
    { // check length for error
        Serial.println("error: message length does not match length");
        return 0; // skip rest of function
    }

    // if the recipient isn't this device or broadcast,
    if (recipient != localAddress && recipient != 0xFF)
    {
        Serial.println("This message is not for me.");
        return 0; // skip rest of function
    }

    loraIn.recipient = recipient;
    if (sender < 4)
    {
        loraIn.sender = sender;
    }
    else
    {
        loraIn.sender = 0;
    }
    loraIn.id = incomingMsgId;
    loraIn.messagelength = incomingLength;
    loraIn.message = incoming;
    loraIn.rssi = LoRa.packetRssi();
    loraIn.snr = LoRa.packetSnr();
    return incomingMsgId;
}
