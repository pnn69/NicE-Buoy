#ifndef LORA_H_
#define LORA_H_
#include "Arduino.h"
/*
    [Designator][Reciever][MSG_ID][lengt_msg][msg]
*/

struct loraDataType
{
    byte destination; // destinatioon
    byte sender;      // sender
    byte recipient;   // reciever
    byte gsi;         // get set info
    byte id;
    byte status;
    int heading;
    byte messagelength;
    String message;
    int rssi;
    float snr;
};

extern loraDataType loraIn;
extern loraDataType loraOut;
extern bool loraOK;

bool sendMessage(String outgoing, byte dest, byte id);
bool InitLora(void);
int polLora(void);
bool sendLoraHello(int buoy);
bool loraMenu(int buoy_nr);

#endif /* LORA_H_ */
