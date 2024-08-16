#ifndef GPS_H_
#define GPS_H_

struct GpsDataType
{
    double lat = 0;   // corrlat = delta latitude given by ground station
    double lon = 0;   // dlat = corrected latitude with delta offset
    float speed = 0;  // kmp/h
    float cource = 0; // degrees
    bool fix = false; //
    char fixtype = 0; // 0-3
    int nrsats = 0;  //
    int lastfix = 0;  // ms
};

extern GpsDataType gpsdata;

void GpsTask(void *arg);

#endif /* GPS_H_ */