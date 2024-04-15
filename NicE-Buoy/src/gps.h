#ifndef GPS_H_
#define GPS_H_

struct GpsDataType
{
    double lat = 0;  // corrlat = delta latitude given by ground station
    double lon = 0; // dlat = corrected latitude with delta offset
    double speed = 0;                      // kmp/h
    double cource = 0;                     // degrees
    bool fix = false;
    byte fixtype = 0; // 0-3
    unsigned int nrsats = 0;
    int lastfix = 0; // ms
};

extern GpsDataType gpsdata;

extern bool gpsvalid;
extern bool gpsactive;

void InitGps(void);
void displayGPSInfo(void);
void RouteToPoint(double lat1, double lon1, double lat2, double lon2, double *distance, double *direction);
int GetNewGpsData(void);

#endif /* GPS_H_ */
