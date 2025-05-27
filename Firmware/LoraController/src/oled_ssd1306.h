#ifndef OLED_SSD1306_H_
#define OLED_SSD1306_H_

#include <RoboCompute.h>

extern bool displayOK;
bool initSSD1306(void);
void speedbars(int sb, int bb);
void showDip(char s, String p);
void updateDisplay(String showdata);
void updateOled(RoboStruct* data);

#endif /* OLED_SSD1306_H_ */
