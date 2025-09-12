#ifndef DATASTORAGE_H_
#define DATASTORAGE_H_

#define GET true
#define SET false
#include <RoboCompute.h>

void initMemory(void);
void memDockPos(RoboStruct *buoy, bool get);
void memBuoyId(int8_t *id, bool get);
void CompassCallibrationFactors(RoboStruct *buoy, bool get);
void Declination(RoboStruct *buoy, bool get);
void CompasOffset(RoboStruct *buoy, bool get);
void computeParameters(RoboStruct *buoy, bool get);
void speedMaxMin(RoboStruct *buoy, bool get);
void pidSpeedParameters(RoboStruct *buoy, bool get);
void pidRudderParameters(RoboStruct *buoy, bool get);
void apParameters(String *ap, String *ww, bool get);
void hardIron(RoboStruct *buoy, bool get);
void softIron(RoboStruct *buoy, bool get);
void CompassCallibrationFactorsFloat(float *MaxX, float *MaxY, float *MaxZ, float *MinX, float *MinY, float *MinZ, bool get);

#endif /* DATASTORAGE_H_ */