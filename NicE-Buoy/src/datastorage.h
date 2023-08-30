#ifndef DATASTORAGE_H_
#define DATASTORAGE_H_

void InitMemory(void);
void MemoryAnchorPos(double *lat, double *lon, bool get);
void MemoryDockPos(double *lat, double *lon, bool get);
void MemoryBuoyID(char *id, bool get);
void Bootcnt(int *bootcnt, bool add);
void LastStatus(byte *data, double *tglat, double *tglon, bool add);
void CompassCallibrationFactors(int16_t *MaxX, int16_t *MaxY, int16_t *MaxZ, int16_t *MinX, int16_t *MinY, int16_t *MinZ, bool get);
void CompassOffsetCorrection(int *delta, bool get);

#endif /* DATASTORAGE_H_ */