#ifndef DATASTORAGE_H_
#define DATASTORAGE_H_

#define GET true
#define SET false

void initMemory(void);
void memDockPos(RoboStruct *buoy, bool get);
void memBuoyId(int8_t *id, bool get);
void CompassCallibrationFactorsFloat(float *MaxX, float *MaxY, float *MaxZ, float *MinX, float *MinY, float *MinZ, bool get);
void Declination(double *declination, bool get);
void MechanicalCorrection(double *delta, bool get);
RoboStruct computeParameters(RoboStruct buoy, bool get);
RoboStruct pidSpeedParameters(RoboStruct buoy, bool get);
RoboStruct pidRudderParameters(RoboStruct buoy, bool get);
void apParameters(String *ap, String *ww, bool get);

#endif /* DATASTORAGE_H_ */