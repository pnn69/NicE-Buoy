#ifndef GPS_H_
#define GPS_H_
struct GpsNav
{
    double tgLat = 0;
    double tgLon = 0;
    double tgDist = 0;
    double tgDir = 0;
    int cmd = 0;
};

struct GpsDataType
{
    double lat = 0;       // corrlat = delta latitude given by ground station
    double lon = 0;       // dlat = corrected latitude with delta offset
    double speed = 0;     // kmp/h
    double cource = 0;    // degrees
    bool fix = false;     //
    char fixtype = 0;     // 0-3
    int nrsats = 0;       //
    uint32_t lastfix = 0; // ms
    bool firstfix = false;
};

// extern GpsDataType gpsdata;
extern QueueHandle_t gpsQue;
bool initgpsqueue(void);
void GpsTask(void *arg);

#endif /* GPS_H_ */