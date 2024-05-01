#ifndef CALCULATE_H_
#define CALCULATE_H_

extern double errSum;

void initCalculate(void);
double averigeWindRose(double samples[], int n);
double deviationWindRose(double samples[], int n);
void addNewSampleInBuffer(double *input, int buflen, double nwdata);
void setparameters(int *minOfsetDist, int *maxOfsetDist, int *minSpeed, int *maxSpeed);
double CalcDocSpeed(double tgdistance);
void adjustPositionDirDist(int dir, double dist, double *tglatitude, double *tglongitude);
void CalcRemoteRudderBuoy(double magheading, float tgheading, int speed, int *bb, int *sb);
bool CalcRudderBuoy(double magheading, float tgheading, double tdistance, int speed, int *bb, int *sb);
void initRudderPid(void);
void initSpeedPid(void);
int hooverPid(double dist);

#endif /* CALCULATE_H_ */
