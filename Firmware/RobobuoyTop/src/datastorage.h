#ifndef DATASTORAGE_H_
#define DATASTORAGE_H_

void initMemory(void);
void memDockPos(RoboStruct buoy, bool get);
void memBuoyId(int8_t *id, bool get);
void computeParameters(RoboStruct buoy, bool get);
void CompassCallibrationFactorsFloat(float *MaxX, float *MaxY, float *MaxZ, float *MinX, float *MinY, float *MinZ, bool get);
void CompassOffsetCorrection(int *delta, bool get);
void MechanicalCorrection(int *delta, bool get);
void computeParameters(RoboStruct buoy, bool get);
void pidSpeedParameters(RoboStruct buoy, bool get);
void pidRudderParameters(RoboStruct buoy, bool get);
void apParameters(String *ap, String *ww, bool get);

#endif /* DATASTORAGE_H_ */