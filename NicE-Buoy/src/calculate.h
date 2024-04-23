#ifndef CALCULATE_H_
#define CALCULATE_H_

extern double errSum;

void initCalculate(void);
void setparameters(int *minOfsetDist, int *maxOfsetDist, int *minSpeed, int *maxSpeed);
int CalcDocSpeed(double tgdistance);
void adjustPositionDirDist(int dir, double dist, double *tglatitude, double *tglongitude);
void CalcRemoteRudderBuoy(double magheading, float tgheading, int speed, int *bb, int *sb);
bool CalcRudderBuoy(double magheading, float tgheading,double tdistance, int speed, int *bb, int *sb);
void initRudderPid(void);
void initSpeedPid(void);
int hooverPid(double dist);

#endif /* CALCULATE_H_ */
