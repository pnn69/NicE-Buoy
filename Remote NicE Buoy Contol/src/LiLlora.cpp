/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
*/
#include <Arduino.h>
#include <SPI.h> // include libraries
#include <LoRa.h>
#include "io.h"
#include "LiLlora.h"
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

void sendMessage(String outgoing)
{
    LoRa.beginPacket();            // start packet
    LoRa.write(destination);       // add destination address
    LoRa.write(localAddress);      // add sender address
    //LoRa.write(msgCount);          // add message ID
    LoRa.write(4);          // add message ID
    LoRa.write(outgoing.length()); // add payload length
    LoRa.print(outgoing);          // add payload
    LoRa.endPacket();              // finish packet and send it
    msgCount++;                    // increment message ID
    if (msgCount > 5)
        msgCount = 0;
}

bool sendLora(void)
{
    String message = "HeLoRa World!"; // send a message
    sendMessage(message);
    return 1;
}

int onReceive(int packetSize)
{
    if (packetSize == 0)
        return 0; // if there's no packet, return

    // read packet header bytes:
    int recipient = LoRa.read();       // recipient address
    byte sender = LoRa.read();         // sender address
    char incomingMsgId = LoRa.read();  // incoming msg ID
    char rssi = LoRa.read();            // rssi address
    float snr = LoRa.read();           // rssi address
    byte incomingLength = LoRa.read(); // incoming msg length
    String incoming = "";

    while (LoRa.available())
    {
        incoming += (char)LoRa.read();
    }
    Serial.println("Incomming data");

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

    // if message is for this device, or broadcast, print details:
    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent to: 0x" + String(recipient, HEX));
    if(incomingMsgId & ( 1 << 7)){
        Serial.println("Is answer on request");
    }
    Serial.println("Message ID: " + String(incomingMsgId,DEC));
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message: " + incoming);
    Serial.println();
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
    return onReceive(LoRa.parsePacket());
}