#ifndef ROBOLORA_H_
#define ROBOLORA_H_
bool InitLora(void);
void LoraTask(void *arg);

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

#endif /* ROBOLORA_H_ */
