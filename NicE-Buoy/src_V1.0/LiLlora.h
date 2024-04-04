#ifndef LORA_H_
#define LORA_H_
#include "Arduino.h"

/*
    MSG_ID (127 msg possible)
    [Designator][Reciever][MSG_ID][lengt_msg][msg][rssi_reciever][snr_reciever]
    msb msg_ID is answer or request. 1 means request. zerro answer from divice
    rssi and snr are 1 and -1 at request (No rssi known)

*/

struct loraDataType
{
    byte destination; // destinatioon
    byte sender;      // sender
    byte status;      // status zender
    byte msgid;       // id
    byte gsia;        // get set info
    byte messagelength;
    String message;
    int rssi;
    float snr;
};

extern loraDataType loraIn;
extern loraDataType loraOut;

extern bool loraOK;
bool InitLora(void);
bool sendLora(void);
int polLora(void);
bool loraMenu(int cmnd);

#endif /* LORA_H_ */
