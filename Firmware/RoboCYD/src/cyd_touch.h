#ifndef CYD_TOUCH_H
#define CYD_TOUCH_H

#include <stdint.h>

extern uint16_t ts_minx;
extern uint16_t ts_maxx;
extern uint16_t ts_miny;
extern uint16_t ts_maxy;

void init_touch();
bool get_touch_point(int &x, int &y);
bool get_raw_touch_point(int &rawX, int &rawY);
void apply_calibration(uint16_t minx, uint16_t maxx, uint16_t miny, uint16_t maxy);

#endif // CYD_TOUCH_H
