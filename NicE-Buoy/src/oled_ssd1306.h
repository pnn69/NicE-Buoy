#ifndef OLED_SSD1306_H_
#define OLED_SSD1306_H_

extern bool displayOK;
bool initSSD1306(void);
void speedbars(int sb, int bb);
void udateDisplay(int sb, int bb, unsigned long distance, unsigned int direction, unsigned int mdirection, bool fix);

#endif /* OLED_SSD1306_H_ */
