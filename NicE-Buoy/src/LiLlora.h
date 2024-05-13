#ifndef LORA_H_
#define LORA_H_
#include "Arduino.h"

/*
    MSG_ID (127 msg possible)
    [Designator][Reciever][MSG_ID][lengt_msg][msg][rssi_reciever][snr_reciever]
    msb msg_ID is answer or request. 1 means request. zerro answer from divice
    rssi and snr are 1 and -1 at request (No rssi known)

*/
extern QueueHandle_t loradataout;
typedef struct Mlora
{
    unsigned int data;
} Mlora;

struct loraDataType
{
    byte recipient;   // destinatioon
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
bool InitLora(void);
void LoraTask(void *arg);
bool sendLora(void);

#endif /* LORA_H_ */
