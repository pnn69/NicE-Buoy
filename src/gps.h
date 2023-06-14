#ifndef GPS_H_
#define GPS_H_

extern bool gpsvalid;
void displayGPSInfo(void);
int InitGps(void);
void RouteToPoint(double lat1, double lon1, double lat2, double lon2, unsigned long *distance, unsigned long *direction);
int GetNewGpsData(double *gpslat, double *gpslng);
void GpsTask(void *arg);

#endif /* GPS_H_ */
