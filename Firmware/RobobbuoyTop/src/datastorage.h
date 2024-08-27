#ifndef DATASTORAGE_H_
#define DATASTORAGE_H_

void initMemory(void);
void memDockPos(double *lat, double *lon, bool get);
void memBuoyId(int8_t *id, bool get);
void memComputeParameters(int *minOfsetDist, int *maxOfsetDist, int *minSpeed, int *maxSpeed, bool get);
void memPidSpeedParameters(double *p, double *i, double *d, bool get);

#endif /* DATASTORAGE_H_ */