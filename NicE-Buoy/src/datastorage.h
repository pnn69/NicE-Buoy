#ifndef DATASTORAGE_H_
#define DATASTORAGE_H_

void InitMemory(void);
void GetAnchorPosMemory(double *lat, double *lon);
void SetAnchorPosMemory(double lat, double lon);
void DockMemory(bool, double *, double *);
void BuoyID(bool, char *);

#endif /* DATASTORAGE_H_ */