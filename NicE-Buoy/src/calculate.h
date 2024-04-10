#ifndef CALCULATE_H_
#define CALCULATE_H_

void initCalculate(void);
void setparameters(int *minOfsetDist, int *maxOfsetDist, int *minSpeed, int *maxSpeed);
int CalcEngingSpeedBuoy(double magheading, float tgheading, double tgdistance, int *bb, int *sb);
void CalcEngingSpeed(float magheading, int tgheading, int speed, int *bb, int *sb);
int CalcNewDirection(double heading1, double heading2);

#endif /* CALCULATE_H_ */
