#ifndef GPS_H_
#define GPS_H_

struct GpsDataType
{
    double lat = 0, dlat = 0, corrlat = 0;
    double lon = 0, dlon = 0, corrlon = 0;
    double speed = 0;
    double cource = 0;
    bool fix = false;
    byte fixtype = 0;
    int lastfix = 0;
};

extern GpsDataType gpsdata;

void displayGPSInfo(void);
int InitGps(void);
void RouteToPoint(double lat1, double lon1, double lat2, double lon2, unsigned long *distance, unsigned long *direction);
int GetNewGpsData(void);
void GpsTask(void *arg);

#endif /* GPS_H_ */
