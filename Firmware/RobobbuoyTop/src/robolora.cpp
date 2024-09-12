#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "main.h"
#include "io_top.h"
#include "robolora.h"
#include "../../RobobuoyDependency\RobobuoyDefaults.h"

struct loraDataType loraOut;
bool lora = false;
#define RADIO_DIO0_PIN 4
bool InitLora(void)
{
    Serial.println("LoRa setup");
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LoRa_frequency))
    {
        Serial.println("Lora setup Failed!");
        return false;
    }
    Serial.println("Lora setup Succes!");
    lora = true;
    return true;
}

String removeWhitespace(String str)
{
    String result = "";
    for (int i = 0; i < str.length(); i++)
    {
        if (str.charAt(i) != ' ')
        {
            result += str.charAt(i);
        }
    }
    return result;
}

void LoraTask(void *arg)
{
    unsigned long nextMsg = millis();
    int msgCnt = 0;
    lora = InitLora();
    printf("Lora task running!\r\n");
    while (1)
    {
        if (LoRa.parsePacket() != 0)
        {
            int recipient_l = LoRa.read();       // recipient address
            byte sender_l = LoRa.read();         // sender address
            byte status_l = LoRa.read();         // status
            byte incomingMsgId_l = LoRa.read();  // msg ID
            byte gsia_l = LoRa.read();           // get set info
            byte incomingLength_l = LoRa.read(); // msg length
            String incoming_l = "";
            Serial.println(incomingMsgId_l);
            while (LoRa.available())
            {
                incoming_l += (char)LoRa.read();
            }
            if (incomingLength_l != incoming_l.length()) // check length for error
            {
                Serial.println("Length error skipping message!");
            }
            if (incomingLength_l > 0)
            {
                Serial.println(incoming_l);
            }
        }
        if (nextMsg < millis())
        {
            nextMsg = millis() + 1000;

            loraOut.message = String(random(19, 25)) + "," + String(random(0, 100));
            loraOut.destination = 0xFE;
            loraOut.msgid = 24;
            loraOut.gsia = 1;

            loraOut.message = removeWhitespace(loraOut.message);
            LoRa.beginPacket();              // start packet
            LoRa.write(loraOut.destination); // add destination address
            LoRa.write(1);                   // add sender address
            LoRa.write(7);                   // status
            LoRa.write(loraOut.msgid);       // id
            LoRa.write(loraOut.gsia);        // get set inf
            LoRa.write(loraOut.message.length());
            LoRa.print(loraOut.message);
            LoRa.endPacket(); // finish packet and send it
        }
        delay(100);
    }
}