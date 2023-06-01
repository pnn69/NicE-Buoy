#ifndef CALCULATE_H_
#define CALCULATE_H_

int CalcEngingSpeedBuoy(float magheading, unsigned long tgheading, unsigned long tgdistance, int *bb, int *sb);
void CalcEngingSpeed(float magheading, unsigned long tgheading, int speed, int *bb, int *sb);
int CalcNewDirection(double heading1, double heading2);

#endif /* CALCULATE_H_ */
