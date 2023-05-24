#ifndef DATASTORAGE_H_
#define DATASTORAGE_H_

void InitMemory(void);
void GetMemoryAnchorPos(double *lat, double *lon);
void SetMemoryAnchorPos(double lat, double lon);
void SetMemoryDockPos(double lat, double lon);
void GetMemoryDockPos(double *lat, double *lon);
void SetMemoryBuoyID(char id);
void GetMemoryBuoyID(char *id);

#endif /* DATASTORAGE_H_ */