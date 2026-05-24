#ifndef MAG_CALIB_H_
#define MAG_CALIB_H_

#include <Arduino.h>
#include <vector>

struct MagPoint {
    float x, y, z;
};

class MagCalibrator {
public:
    MagCalibrator();
    void addPoint(float x, float y, float z);
    bool calculateCalibration(float hard_iron[3], float soft_iron[3][3]);
    void reset();
    int getNumPoints() const { return points.size(); }

private:
    std::vector<MagPoint> points;
    static constexpr int MAX_POINTS = 300;
    static constexpr float MIN_DIST_SQ = 25.0f; // Minimum squared distance (5uT) between points to accept a new one

    bool solve9x9(double A[9][9], double b[9], double x[9]);
    void invert3x3(double m[3][3], double inv[3][3]);
    void jacobiEigen3x3(double m[3][3], double v[3][3], double d[3]);
};

#endif // MAG_CALIB_H_
