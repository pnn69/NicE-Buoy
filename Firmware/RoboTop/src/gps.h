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

// extern GpsDataType gpsdata;
void RouteToPoint(double lat1, double lon1, double lat2, double lon2, double *distance, double *direction);
extern QueueHandle_t gpsQue;
bool initgpsqueue(void);
void GpsTask(void *arg);

#endif /* GPS_H_ */