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
    byte status;      // status zender
    byte msgid;       // id
    byte gsia;        // get set info ack
    byte messagelength;
    String message;
    int rssi;
    float snr;
};

extern loraDataType loraIn;
extern loraDataType loraOut;
extern bool loraOK;

void sendMessage(String outgoing, byte dest, byte msg_id, byte gsia);
bool InitLora(void);
int polLora(void);
bool loraMenu(int buoy_nr);

#endif /* LORA_H_ */
