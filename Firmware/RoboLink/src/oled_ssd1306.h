#ifndef OLED_SSD1306_H_
#define OLED_SSD1306_H_

#include <RoboCompute.h>
#include "adc.h"

bool initSSD1306(void);
void updateOled(RoboStruct *buoy, adcDataType *adc);
void showDip(char s, String p);

#endif /* OLED_SSD1306_H_ */
