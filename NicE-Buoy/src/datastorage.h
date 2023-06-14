#ifndef DATASTORAGE_H_
#define DATASTORAGE_H_

void InitMemory(void);
void GetMemoryAnchorPos(double *lat, double *lon);
void SetMemoryAnchorPos(double lat, double lon);
void SetMemoryDockPos(double lat, double lon);
void GetMemoryDockPos(double *lat, double *lon);
void SetMemoryBuoyID(char id);
void GetMemoryBuoyID(char *id);
void Bootcnt(int *bootcnt, bool add);
void LastStatus(byte *data, bool add, double *tglat, double *tglon);

#endif /* DATASTORAGE_H_ */