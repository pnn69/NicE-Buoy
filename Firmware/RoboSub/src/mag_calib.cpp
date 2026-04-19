#include "mag_calib.h"
#include <math.h>
#include <string.h>

MagCalibrator::MagCalibrator() {
    points.reserve(MAX_POINTS);
}

void MagCalibrator::reset() {
    points.clear();
}

void MagCalibrator::addPoint(float x, float y, float z) {
    if (points.size() >= MAX_POINTS) return;
    
    // Check distance to existing points to ensure good spatial distribution
    for (const auto& p : points) {
        float dx = p.x - x;
        float dy = p.y - y;
        float dz = p.z - z;
        if (dx*dx + dy*dy + dz*dz < MIN_DIST_SQ) {
            return; // Too close to an existing point
        }
    }
    points.push_back({x, y, z});
}

bool MagCalibrator::solve9x9(double A[9][9], double b[9], double x[9]) {
    double M[9][10];
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) M[i][j] = A[i][j];
        M[i][9] = b[i];
    }

    for (int i = 0; i < 9; i++) {
        // Pivot
        int pivot = i;
        for (int j = i + 1; j < 9; j++) {
            if (fabs(M[j][i]) > fabs(M[pivot][i])) pivot = j;
        }
        if (fabs(M[pivot][i]) < 1e-10) return false; // Singular

        if (pivot != i) {
            for (int k = 0; k < 10; k++) {
                double temp = M[i][k];
                M[i][k] = M[pivot][k];
                M[pivot][k] = temp;
            }
        }

        double div = M[i][i];
        for (int k = i; k < 10; k++) M[i][k] /= div;

        for (int j = 0; j < 9; j++) {
            if (i != j) {
                double mult = M[j][i];
                for (int k = i; k < 10; k++) M[j][k] -= mult * M[i][k];
            }
        }
    }

    for (int i = 0; i < 9; i++) x[i] = M[i][9];
    return true;
}

void MagCalibrator::invert3x3(double m[3][3], double inv[3][3]) {
    double det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
                 m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                 m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    if (fabs(det) < 1e-10) {
        // Fallback to identity
        for(int i=0; i<3; i++) for(int j=0; j<3; j++) inv[i][j] = (i==j)?1.0:0.0;
        return;
    }

    double invdet = 1.0 / det;

    inv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
    inv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
    inv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
    inv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
    inv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;
}

void MagCalibrator::jacobiEigen3x3(double m[3][3], double v[3][3], double d[3]) {
    double a[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            a[i][j] = m[i][j];
            v[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    int iters = 0;
    while (iters < 50) {
        double max_off = 0.0;
        int p = 0, q = 1;
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 3; j++) {
                if (fabs(a[i][j]) > max_off) {
                    max_off = fabs(a[i][j]);
                    p = i;
                    q = j;
                }
            }
        }

        if (max_off < 1e-9) break;

        double theta = 0.5 * atan2(2.0 * a[p][q], a[p][p] - a[q][q]);
        double c = cos(theta);
        double s = sin(theta);

        for (int i = 0; i < 3; i++) {
            double temp_v = v[i][p];
            v[i][p] = c * temp_v - s * v[i][q];
            v[i][q] = s * temp_v + c * v[i][q];

            if (i != p && i != q) {
                double temp_a = a[i][p];
                a[i][p] = c * temp_a - s * a[i][q];
                a[p][i] = a[i][p];
                a[i][q] = s * temp_a + c * a[i][q];
                a[q][i] = a[i][q];
            }
        }
        double app = c * c * a[p][p] - 2.0 * s * c * a[p][q] + s * s * a[q][q];
        double aqq = s * s * a[p][p] + 2.0 * s * c * a[p][q] + c * c * a[q][q];
        a[p][p] = app;
        a[q][q] = aqq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;

        iters++;
    }

    for (int i = 0; i < 3; i++) d[i] = a[i][i];
}

bool MagCalibrator::calculateCalibration(float hard_iron[3], float soft_iron[3][3]) {
    if (points.size() < 10) return false;

    double M[9][9] = {0};
    double rhs[9] = {0};

    for (const auto& p : points) {
        double v[9] = {
            (double)p.x * p.x, (double)p.y * p.y, (double)p.z * p.z,
            2.0 * p.x * p.y, 2.0 * p.x * p.z, 2.0 * p.y * p.z,
            2.0 * p.x, 2.0 * p.y, 2.0 * p.z
        };

        for (int i = 0; i < 9; i++) {
            rhs[i] += v[i];
            for (int j = 0; j < 9; j++) {
                M[i][j] += v[i] * v[j];
            }
        }
    }

    double beta[9];
    if (!solve9x9(M, rhs, beta)) return false;

    double Q[3][3] = {
        {beta[0], beta[3], beta[4]},
        {beta[3], beta[1], beta[5]},
        {beta[4], beta[5], beta[2]}
    };
    double u[3] = {2.0 * beta[6], 2.0 * beta[7], 2.0 * beta[8]};

    double Q_inv[3][3];
    invert3x3(Q, Q_inv);

    double c[3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            c[i] += Q_inv[i][j] * u[j];
        }
        c[i] *= -0.5;
    }

    double R2 = 1.0;
    for (int i = 0; i < 3; i++) {
        double Q_c_i = 0;
        for (int j = 0; j < 3; j++) Q_c_i += Q[i][j] * c[j];
        R2 += c[i] * Q_c_i;
    }

    if (R2 <= 0) return false;

    double W_norm[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            W_norm[i][j] = Q[i][j] / R2;
        }
    }

    double eigval[3];
    double eigvec[3][3];
    jacobiEigen3x3(W_norm, eigvec, eigval);

    double W_soft[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                if (eigval[k] > 0) {
                    W_soft[i][j] += eigvec[i][k] * sqrt(eigval[k]) * eigvec[j][k];
                }
            }
        }
    }

    // Scale to a typical magnetic field value (e.g., 50.0 uT) to keep magnitudes standard
    double scale = 50.0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            soft_iron[i][j] = (float)(W_soft[i][j] * scale);
        }
        hard_iron[i] = (float)c[i];
    }

    return true;
}
