#pragma once

#include <pathfinder/geometry/vector2.hpp>

#include <stdexcept>

namespace pathfinder {

// 3x3 row-major double matrix. Intended for small fixed-size linear algebra
// (3x3 pose covariance for the EKF; eventually 6x6 for the full state).
// Not optimized for large matrices; if Pathfinder ever needs general dense
// linear algebra, swap this for a dedicated library.
struct Matrix3 {
    double m[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    static constexpr Matrix3 zero() { return Matrix3{}; }

    static constexpr Matrix3 identity() {
        Matrix3 r;
        r.m[0][0] = 1.0;
        r.m[1][1] = 1.0;
        r.m[2][2] = 1.0;
        return r;
    }

    Matrix3 operator+(const Matrix3& rhs) const {
        Matrix3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = m[i][j] + rhs.m[i][j];
        return r;
    }

    Matrix3 operator-(const Matrix3& rhs) const {
        Matrix3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = m[i][j] - rhs.m[i][j];
        return r;
    }

    Matrix3 operator*(const Matrix3& rhs) const {
        Matrix3 r;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double s = 0.0;
                for (int k = 0; k < 3; ++k) s += m[i][k] * rhs.m[k][j];
                r.m[i][j] = s;
            }
        }
        return r;
    }

    Matrix3 operator*(double s) const {
        Matrix3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = m[i][j] * s;
        return r;
    }

    Matrix3& operator+=(const Matrix3& rhs) { *this = *this + rhs; return *this; }
    Matrix3& operator-=(const Matrix3& rhs) { *this = *this - rhs; return *this; }
    Matrix3& operator*=(const Matrix3& rhs) { *this = *this * rhs; return *this; }
    Matrix3& operator*=(double s) { *this = *this * s; return *this; }

    Matrix3 transpose() const {
        Matrix3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = m[j][i];
        return r;
    }

    double determinant() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    // Cofactor-based inverse. Caller is responsible for invertibility.
    Matrix3 inverse() const {
        const double det = determinant();
        if (det == 0.0) throw std::domain_error("Matrix3::inverse: singular matrix");
        const double inv_det = 1.0 / det;
        Matrix3 r;
        r.m[0][0] =  (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
        r.m[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * inv_det;
        r.m[0][2] =  (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;
        r.m[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * inv_det;
        r.m[1][1] =  (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
        r.m[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * inv_det;
        r.m[2][0] =  (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
        r.m[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * inv_det;
        r.m[2][2] =  (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;
        return r;
    }
};

inline Matrix3 operator*(double s, const Matrix3& m) { return m * s; }

inline Vector3 operator*(const Matrix3& m, Vector3 v) {
    return {
        m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z,
        m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z,
        m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z,
    };
}

} // namespace pathfinder
