#ifndef GPS_H_
#define GPS_H_

struct GpsDataType
{
    double lat = 0, dlat = 0, corrlat = 0; // corrlat = delta latitude given by ground station
    double lon = 0, dlon = 0, corrlon = 0; // dlat = corrected latitude with delta offset
    double speed = 0;                      // kmp/h
    double cource = 0;                     // degrees
    bool fix = false;
    byte fixtype = 0; // 0-3
    byte nrsats = 0;
    int lastfix = 0; // ms
};

extern GpsDataType gpsdata;

extern bool gpsvalid;
void displayGPSInfo(void);
int InitGps(void);
void RouteToPoint(double lat1, double lon1, double lat2, double lon2, double *distance, double *direction);
int GetNewGpsData(void);
void GpsTask(void *arg);

#endif /* GPS_H_ */
