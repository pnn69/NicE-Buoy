#ifndef CALCULATE_H_
#define CALCULATE_H_

extern double errSum;

void initCalculate(void);
void setparameters(int *minOfsetDist, int *maxOfsetDist, int *minSpeed, int *maxSpeed);
int CalcDocSpeed(double tgdistance);
void resetRudder(void);
void CalcSpeedRudderBuoy(double magheading, float tgheading, int speed, int *bb, int *sb);
void CalcEngingRudderBuoy(double magheading, float tgheading, int speed, int *bb, int *sb);
void initPid(void);
int hooverPid(double dist);

#endif /* CALCULATE_H_ */
