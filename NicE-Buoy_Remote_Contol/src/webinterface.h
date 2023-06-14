#ifndef WEBINTERFACE_H_
#define WEBINTERFACE_H_

extern int notify;
extern int radiobutton[NR_BUOYS];
void initWebSocket(void);
void websetup(void);
void webloop(void);

#endif /* WEBINTERFACE_H_ */
