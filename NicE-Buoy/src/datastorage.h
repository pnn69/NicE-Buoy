#ifndef DATASTORAGE_H_
#define DATASTORAGE_H_
#include <Arduino.h>

void InitMemory(void);
void MemoryAnchorPos(double *lat, double *lon, bool get);
void MemoryDockPos(double *lat, double *lon, bool get);
void MemoryBuoyID(char *id, bool get);
void Bootcnt(int *bootcnt, bool add);
void LastStatus(byte *data, double *tglat, double *tglon, bool get);
void CompassCallibrationFactorsFloat(float *MaxX, float *MaxY, float *MaxZ, float *MinX, float *MinY, float *MinZ, bool get);
void CompassCallibrationFactorsInt(int16_t *MaxX, int16_t *MaxY, int16_t *MaxZ, int16_t *MinX, int16_t *MinY, int16_t *MinZ, bool get);
void CompassOffsetCorrection(int *delta, bool get);
void computeParameters(int *minOfsetDist, int *maxOfsetDist, int *minSpeed, int *maxSpeed, bool get);
void pidParameters(double *p, double *i, double *d,bool get);

#endif /* DATASTORAGE_H_ */