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

bool sendMessage(String outgoing, byte dest, byte id)
{
    if (lasttransmission + 150 > millis())
    { // prefend fast transmissions
        return 1;
    }
    ledstatus = true;
    LoRa.beginPacket();            // start packet
    LoRa.write(dest);              // add destination address
    LoRa.write(localAddress);      // add sender address
    LoRa.write(id);                // add message ID
    LoRa.write(buoy[dest].status); // status
    LoRa.write(outgoing.length()); // add payload length
    LoRa.print(outgoing);          // add payload
    LoRa.endPacket();              // finish packet and send it
    Serial.print("Lora out Dest:" + String(dest) + " id:" + id + " status: " + String(buoy[dest].status) + " msg:<" + outgoing + ">");
    if (id == ACK)
    {
        Serial.printf(" ACK");
    }
    else if (id == SET)
    {
        Serial.printf(" SET");
    }
    else if (id == GET)
    {
        Serial.printf(" GET");
    }
    Serial.println();

    lasttransmission = millis();
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
    byte status_l = LoRa.read();         // sender address
    byte incomingMsgId_l = LoRa.read();  // incoming msg ID
    byte gsi_l = LoRa.read();            // get set info
    byte incomingLength_l = LoRa.read(); // incoming msg length
    String incoming = "";
    while (LoRa.available())
    {
        incoming += (char)LoRa.read();
    }
    if (incomingLength != incoming.length())
    {             // check length for error
        return 0; // skip rest of function
    }
    // Get and store the data
    loraIn.sender = sender;
    loraIn.id = incomingMsgId;
    loraIn.status = bstatus;
    loraIn.heading = mheading;
    loraIn.messagelength = incomingLength;
    loraIn.message = incoming;
    loraIn.rssi = LoRa.packetRssi();
    loraIn.snr = LoRa.packetSnr();
    return incomingMsgId;
}

int polLora(void)
{
    if (loraOK == false)
    {
        return 0;
    }
    if (LoRa.parsePacket() == 0)
        return 0; // if there's no packet, return
    int msg = decodeMsg();
    if (msg == 0)
    {
        return 0;
    }
    if (msg)
    {
        String decode = loraIn.message;
        char messarr[100];
        int dir, dist;
        int sp, sb, bb;
        float lhe;
        Serial.print("Lora in from:" + String(loraIn.sender) + " RSSI:" + String(loraIn.rssi) + " msg <" + loraIn.message + "> status:" + String(loraIn.status) + "\r\n");
        // Serial.printf(">Heading: %d", loraIn.heading);
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
        //  else if (loraIn.id == INF)
        //  {
        //      Serial.printf(" INF");
        //  }
        // Serial.println();
        loraIn.message.toCharArray(messarr, loraIn.message.length() + 1);
        int cmnd = 0;
        sscanf(messarr, "%d", &cmnd);
        int index = loraIn.message.indexOf(",");
        loraIn.message = loraIn.message.substring(index + 1);             // strip msg ID
        loraIn.message.toCharArray(messarr, loraIn.message.length() + 1); // conver to string

        if (NR_BUOYS > loraIn.sender)
        {
            buoy[loraIn.sender].remotestatus = loraIn.status;
            // buoy[loraIn.sender].mdir = loraIn.heading;
            switch (cmnd)
            {
            case DIR_DISTANSE_TO_TARGET_POSITION:
                // Serial.println("direction and distance target recieved!");
                sscanf(messarr, "%d,%d", &dir, &dist);
                buoy[loraIn.sender].tgdir = dir;
                buoy[loraIn.sender].tgdistance = dist;
                buoy[loraIn.sender].rssi = loraIn.rssi;
                buoy[loraIn.sender].ackOK = loraIn.id;
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
                buoy[loraIn.sender].ackOK = loraIn.id;
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
                buoy[loraIn.sender].ackOK = loraIn.id;
                break;

            case (TARGET_POSITION):
                if (loraIn.id == ACK)
                {
                    buoy[loraIn.sender].cmnd = DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION;
                    buoy[loraIn.sender].status = LOCKED;
                    buoy[loraIn.sender].ackOK = loraIn.id;
                }
                break;

            case (GOTO_TARGET_POSITION):
                buoy[loraIn.sender].cmnd = DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION;
                buoy[loraIn.sender].status = LOCKED;
                buoy[loraIn.sender].ackOK = loraIn.id;
                break;

            case GOTO_DOC_POSITION:
                if (loraIn.id == ACK)
                {
                    Serial.println("Doc position SET!");
                    buoy[loraIn.sender].cmnd = DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION;
                    buoy[loraIn.sender].status = LOCKED;
                    buoy[loraIn.sender].ackOK = loraIn.id;
                    buoy[loraIn.sender].ackOK = true;
                }
                break;

            case ANCHOR_POSITION:
                buoy[loraIn.sender].ackOK = true;
                Serial.println("New anchor position recieved!");
                Serial.println(String(loraIn.message));
                buoy[loraIn.sender].rssi = loraIn.rssi;
                buoy[loraIn.sender].ackOK = loraIn.id;
                break;

            case (BUOY_MODE_IDLE):
                if (loraIn.id == ACK)
                {
                    buoy[loraIn.sender].ackOK = true;
                }
                buoy[loraIn.sender].cmnd = BUOY_MODE_IDLE;
                buoy[loraIn.sender].status = IDLE;
                buoy[loraIn.sender].ackOK = loraIn.id;
                break;
            case (NO_POSITION):
                if (loraIn.id == ACK)
                {
                    buoy[loraIn.sender].ackOK = true;
                }
                buoy[loraIn.sender].cmnd = NO_POSITION;
                buoy[loraIn.sender].status = IDLE;
                buoy[loraIn.sender].ackOK = loraIn.id;
                break;
            case (BATTERY_VOLTAGE_PERCENTAGE):
                sscanf(messarr, "%f,%d", &lhe, &sp);
                buoy[loraIn.sender].voltage = lhe;
                buoy[loraIn.sender].percentage = sp;
                break;
            default:
                Serial.println("unknown command: " + msg);
                break;
            }
            notify = loraIn.sender;
        }
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
        msg = String(SAIL_DIR_SPEED) + "," + String(buoy[buoy_nr].cdir) + "," + String(buoy[buoy_nr].cspeed);
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION:
        msg = String(DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION) + "," + String(buoy[buoy_nr].cdir) + "," + String(buoy[buoy_nr].cspeed);
        buoy[buoy_nr].gsa = GET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case SAIL_DIR_SPEED:
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

    case DOC_POSITION:
        msg = String(DOC_POSITION) + "," + buoy[buoy_nr].doclatitude + "," + buoy[buoy_nr].doclongitude;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case GOTO_DOC_POSITION:
        msg = String(GOTO_DOC_POSITION);
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case RESET:
        msg = String(RESET);
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case BUOY_MODE_IDLE:
        msg = String(BUOY_MODE_IDLE);
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case NO_POSITION:
        msg = String(NO_POSITION);
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        break;

    case DGPS:
        if (gpsdata.corrlat == 0 && gpsdata.corrlon == 0)
        {
            buoy[buoy_nr].ackOK = true;
            break;
        }
        msg = String(DGPS) + "," + String(gpsdata.corrlat, 8) + "," + String(gpsdata.corrlon, 8);
        buoy[buoy_nr].gsa = SET;
        sendMessage(msg, buoy_nr, buoy[buoy_nr].gsa);
        buoy[buoy_nr].ackOK = true;
        break;

    default:
        Serial.printf("Unkown Lora command! buoynr: %d cmnd:%d\r\n", buoy_nr, buoy[buoy_nr].cmnd);
        buoy[buoy_nr].ackOK = true;
        break;
    }
    return 0;
}
