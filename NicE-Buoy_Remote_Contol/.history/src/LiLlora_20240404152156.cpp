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

bool sendMessage(String outgoing, byte dest, byte msg_id, byte gsia)
{
    if (lasttransmission + 150 > millis())
    { // prefend fast transmissions
        return 1;
    }
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
    Serial.println("Lora out Dest:" + String(dest) + "Status 0,id:" + String(msg_id) + "gsia:" + String(gsia) + " msg:<" + outgoing + ">");
    return 0;
}

bool sendLoraHello(int buoy)
{
    String msg = String(TXT) + "HeLoRa World!"; // send a msg
    sendMessage(msg, buoy, GET);
    return 1;
}

int decodeMsg()
{
    // read packet header bytes:
    int recipient = LoRa.read(); // address send to
    // if the recipient isn't this device or broadcast,
    if (recipient != 0xFF && recipient != 0xFE)
    {
        Serial.print(recipient);
        Serial.println(" Not for me!");
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
    loraIn.sender = sender_l;
    loraIn.status = status_l;
    loraIn.msgid = incomingMsgId_l;
    loraIn.gsia = gsia_l;
    loraIn.messagelength = incomingLength_l;
    loraIn.message = incoming;
    loraIn.rssi = LoRa.packetRssi();
    loraIn.snr = LoRa.packetSnr();
    return 1;
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
    int dir, dist;
    int sp, sb, bb;
    float lhe;
    Serial.println("Lora:" + String(loraIn.sender) + " RSSI:" + String(loraIn.rssi) + "ms_id" + String(loraIn.msgid) + "gsia" + String(loraIn.gsia) + "msg <" + loraIn.message + "> status:" + String(loraIn.status) + "\r\n");
    loraIn.message.toCharArray(messarr, loraIn.message.length() + 1);
    // int index = loraIn.message.indexOf(",");
    // loraIn.message = loraIn.message.substring(index + 1);             // strip msg ID
    // loraIn.message.toCharArray(messarr, loraIn.message.length() + 1); // conver to string

    if (NR_BUOYS > loraIn.sender)
    {
        buoy[loraIn.sender].remotestatus = loraIn.status;
        switch (loraIn.msgid)
        {
        case DIR_DISTANSE_TO_TARGET_POSITION:
            // Serial.println("direction and distance target recieved!");
            sscanf(messarr, "%d,%d", &dir, &dist);
            buoy[loraIn.sender].tgdir = dir;
            buoy[loraIn.sender].tgdistance = dist;
            buoy[loraIn.sender].rssi = loraIn.rssi;
            buoy[loraIn.sender].ackOK = loraIn.gsia;
            break;

        case DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION:
            // Serial.println("direction and distance target recieved!");
            sscanf(messarr, "%ld,%ld,%d,%d,%d,%d",
                   &buoy[loraIn.sender].tgdir,
                   &buoy[loraIn.sender].tgdistance,
                   &buoy[loraIn.sender].speed,
                   &buoy[loraIn.sender].speedbb,
                   &buoy[loraIn.sender].speedsb,
                   &buoy[loraIn.sender].mdir);
            buoy[loraIn.sender].rssi = loraIn.rssi;
            buoy[loraIn.sender].ackOK = loraIn.gsia;
            break;

        case GPS_LAT_LON_FIX_HEADING_SPEED_MHEADING:
            sscanf(messarr, "%lf,%lf,%d,%d,%d,%d",
                   &buoy[loraIn.sender].gpslatitude,
                   &buoy[loraIn.sender].gpslongitude,
                   &buoy[loraIn.sender].fix,
                   &buoy[loraIn.sender].gpscource,
                   &buoy[loraIn.sender].gpsspeed,
                   &buoy[loraIn.sender].mdir);
            buoy[loraIn.sender].rssi = loraIn.rssi;
            buoy[loraIn.sender].ackOK = loraIn.gsia;
            break;

        case SAIL_DIR_SPEED:
            sscanf(messarr, "%f,%d,%d,%d", &lhe, &sp, &bb, &sb);
            buoy[loraIn.sender].tgdir = 0;
            buoy[loraIn.sender].mdir = (int)lhe;
            buoy[loraIn.sender].tgdistance = 0;
            buoy[loraIn.sender].speed = sp;
            buoy[loraIn.sender].speedsb = sb;
            buoy[loraIn.sender].speedbb = bb;
            buoy[loraIn.sender].rssi = loraIn.rssi;
            buoy[loraIn.sender].ackOK = loraIn.gsia;
            break;

        case (TARGET_POSITION):
            if (loraIn.gsia == ACK)
            {
                buoy[loraIn.sender].cmnd = DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION;
                buoy[loraIn.sender].status = LOCKED;
                buoy[loraIn.sender].ackOK = loraIn.gsia;
            }
            break;

        case (GOTO_TARGET_POSITION):
            buoy[loraIn.sender].cmnd = DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION;
            buoy[loraIn.sender].status = LOCKED;
            buoy[loraIn.sender].ackOK = loraIn.gsia;
            break;

        case STORE_DOC_POSITION:
            if (loraIn.gsia == ACK)
            {
                Serial.println("Doc position SET!");
                buoy[loraIn.sender].ackOK = true;
            }
            break;

        case (BUOY_MODE_IDLE):
            if (loraIn.gsia == ACK)
            {
                buoy[loraIn.sender].ackOK = true;
            }
            buoy[loraIn.sender].cmnd = BUOY_MODE_IDLE;
            buoy[loraIn.sender].status = IDLE;
            break;

        case (BATTERY_VOLTAGE_PERCENTAGE):
            sscanf(messarr, "%f,%d", &lhe, &sp);
            buoy[loraIn.sender].voltage = lhe;
            buoy[loraIn.sender].percentage = sp;
            break;
        default:
            Serial.println("unknown command: " + loraIn.msgid);
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
    if (lasttransmission + 150 > millis())
    { // prefend fast transmissions
        return 1;
    }
    String msg = "";
    buoy[buoy_nr].ackOK = false;

    switch (buoy[buoy_nr].cmnd)
    {
    case LOCKED:
        loraOut.msgid = SAIL_DIR_SPEED;
        loraOut.gsia = GET;
        msg = String(buoy[buoy_nr].cdir) + "," + String(buoy[buoy_nr].cspeed);
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION:
        loraOut.msgid = DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION;
        loraOut.gsia = SET;
        msg = String(DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION) + "," + String(buoy[buoy_nr].cdir) + "," + String(buoy[buoy_nr].cspeed);
        buoy[buoy_nr].gsa = GET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case SAIL_DIR_SPEED:
        loraOut.msgid = SAIL_DIR_SPEED;
        loraOut.gsia = SET;
        msg = String(SAIL_DIR_SPEED) + "," + String(buoy[buoy_nr].cdir) + "," + String(buoy[buoy_nr].cspeed);
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case TARGET_POSITION:
        msg = String(TARGET_POSITION);
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case GOTO_TARGET_POSITION:
        msg = String(GOTO_TARGET_POSITION);
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case STORE_DOC_POSITION:
        loraOut.msgid = STORE_DOC_POSITION;
        loraOut.gsia = SET;
        msg = buoy[buoy_nr].doclatitude + "," + buoy[buoy_nr].doclongitude;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case DOC_POSITION:
        loraOut.msgid = DOC_POSITION;
        loraOut.gsia = SET;
        msg = "";
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case RESET:
        loraOut.msgid = DOC_POSITION;
        loraOut.gsia = INF;
        msg = "";
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case BUOY_MODE_IDLE:
        loraOut.msgid = BUOY_MODE_IDLE;
        loraOut.gsia = SET;
        msg = "";
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case NO_POSITION:
        break;

    default:
        Serial.printf("Unkown Lora command! buoynr: %d cmnd:%d\r\n", buoy_nr, buoy[buoy_nr].cmnd);
        buoy[buoy_nr].ackOK = true;
        break;
    }
    return 0;
}
