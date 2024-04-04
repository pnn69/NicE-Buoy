#ifndef DATASTORAGE_H_
#define DATASTORAGE_H_

void InitMemory(void);
void GetMemoryAnchorPos(double *lat, double *lon);
void SetMemoryAnchorPos(double lat, double lon);
void SetMemoryDockPos(double lat, double lon);
void GetMemoryDockPos(double *lat, double *lon);
void MemoryDockPos(double *lat, double *lon, bool get);
void SetMemoryBuoyID(char id);
void GetMemoryBuoyID(char *id);
void Bootcnt(int *bootcnt, bool add);
void LastStatus(byte *data, bool add, double *tglat, double *tglon);
void CompassCallibrationFactors(int16_t *MaxX, int16_t *MaxY, int16_t *MaxZ, int16_t *MinX, int16_t *MinY, int16_t *MinZ, bool get);
void CompassOffsetCorrection(int *delta, bool get);

#endif /* DATASTORAGE_H_ */