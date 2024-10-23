#ifndef DATASTORAGE_H_
#define DATASTORAGE_H_

#define GET true
#define SET false

void initMemory(void);
void memDockPos(double *lat, double *lon, bool get);
void memBuoyId(int8_t *id, bool get);
void memComputeParameters(int *minOfsetDist, int *maxOfsetDist, int *minSpeed, int *maxSpeed, bool get);
void memPidSpeedParameters(double *p, double *i, double *d, bool get);
void CompassCallibrationFactorsFloat(float *MaxX, float *MaxY, float *MaxZ, float *MinX, float *MinY, float *MinZ, bool get);
void CompassOffsetCorrection(int *delta, bool get);
void MechanicalCorrection(int *delta, bool get);
void computeParameters(int *minOfsetDist, int *maxOfsetDist, int *minSpeed, int *maxSpeed, bool get);
void pidSpeedParameters(double *p, double *i, double *d, bool get);
void pidRudderParameters(double *p, double *i, double *d, bool get);
void apParameters(String *ap, String *ww, bool get);
void memStuct(pid &data, bool get);

#endif /* DATASTORAGE_H_ */