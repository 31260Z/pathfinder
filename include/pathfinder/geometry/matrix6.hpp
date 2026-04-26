#pragma once

#include <cstddef>
#include <stdexcept>

namespace pathfinder {

// 6x6 row-major double matrix and 6-element vector. Used by the Tier-2 EKF
// for the full 6-DOF state covariance ([x, y, θ, v_x, v_y, ω]).
//
// Hand-rolled rather than added to a dependency: the rest of the library
// uses bespoke `Vector2`/`Pose2`/`Matrix3` types (per spec §17 — no Eigen),
// and the EKF only needs basic arithmetic + a dense Gauss-Jordan inverse on
// a tiny (6x6) matrix. ~2k FP ops per cycle even with naive triple-nested
// loops; well within the 1.5 ms/cycle budget cited in spec §8.
struct Vector6 {
    double v[6] = {};

    constexpr double& operator[](std::size_t i) { return v[i]; }
    constexpr double  operator[](std::size_t i) const { return v[i]; }

    Vector6 operator+(const Vector6& rhs) const {
        Vector6 r;
        for (std::size_t i = 0; i < 6; ++i) r.v[i] = v[i] + rhs.v[i];
        return r;
    }
    Vector6 operator-(const Vector6& rhs) const {
        Vector6 r;
        for (std::size_t i = 0; i < 6; ++i) r.v[i] = v[i] - rhs.v[i];
        return r;
    }

    Vector6& operator+=(const Vector6& rhs) { *this = *this + rhs; return *this; }
    Vector6& operator-=(const Vector6& rhs) { *this = *this - rhs; return *this; }
};

struct Matrix6 {
    double m[6][6] = {};

    static Matrix6 zero() { return Matrix6{}; }

    static Matrix6 identity() {
        Matrix6 r;
        for (std::size_t i = 0; i < 6; ++i) r.m[i][i] = 1.0;
        return r;
    }

    double& at(std::size_t i, std::size_t j) { return m[i][j]; }
    double  at(std::size_t i, std::size_t j) const { return m[i][j]; }

    Matrix6 operator+(const Matrix6& rhs) const {
        Matrix6 r;
        for (std::size_t i = 0; i < 6; ++i)
            for (std::size_t j = 0; j < 6; ++j)
                r.m[i][j] = m[i][j] + rhs.m[i][j];
        return r;
    }

    Matrix6 operator-(const Matrix6& rhs) const {
        Matrix6 r;
        for (std::size_t i = 0; i < 6; ++i)
            for (std::size_t j = 0; j < 6; ++j)
                r.m[i][j] = m[i][j] - rhs.m[i][j];
        return r;
    }

    Matrix6 operator*(const Matrix6& rhs) const {
        Matrix6 r;
        for (std::size_t i = 0; i < 6; ++i) {
            for (std::size_t j = 0; j < 6; ++j) {
                double s = 0.0;
                for (std::size_t k = 0; k < 6; ++k) s += m[i][k] * rhs.m[k][j];
                r.m[i][j] = s;
            }
        }
        return r;
    }

    Matrix6 operator*(double scalar) const {
        Matrix6 r;
        for (std::size_t i = 0; i < 6; ++i)
            for (std::size_t j = 0; j < 6; ++j)
                r.m[i][j] = m[i][j] * scalar;
        return r;
    }

    Matrix6& operator+=(const Matrix6& rhs) { *this = *this + rhs; return *this; }
    Matrix6& operator-=(const Matrix6& rhs) { *this = *this - rhs; return *this; }
    Matrix6& operator*=(const Matrix6& rhs) { *this = *this * rhs; return *this; }
    Matrix6& operator*=(double scalar) { *this = *this * scalar; return *this; }

    Matrix6 transpose() const {
        Matrix6 r;
        for (std::size_t i = 0; i < 6; ++i)
            for (std::size_t j = 0; j < 6; ++j)
                r.m[i][j] = m[j][i];
        return r;
    }

    // Gauss-Jordan elimination with partial pivoting. Throws on singular
    // input. EKF callers add a tiny diagonal regularizer before inverting if
    // they expect the matrix to be near-singular (small sigma measurements).
    Matrix6 inverse() const {
        // Augmented matrix [A | I], 6x12 row-major.
        double a[6][12] = {};
        for (std::size_t i = 0; i < 6; ++i) {
            for (std::size_t j = 0; j < 6; ++j) a[i][j] = m[i][j];
            a[i][6 + i] = 1.0;
        }

        for (std::size_t i = 0; i < 6; ++i) {
            // Partial pivot: find max-magnitude row in column i, rows i..5.
            std::size_t pivot = i;
            double      best  = (a[i][i] < 0 ? -a[i][i] : a[i][i]);
            for (std::size_t r = i + 1; r < 6; ++r) {
                const double absv = (a[r][i] < 0 ? -a[r][i] : a[r][i]);
                if (absv > best) { best = absv; pivot = r; }
            }
            if (best == 0.0) {
                throw std::domain_error("Matrix6::inverse: singular matrix");
            }
            if (pivot != i) {
                for (std::size_t j = 0; j < 12; ++j) {
                    const double tmp = a[i][j];
                    a[i][j] = a[pivot][j];
                    a[pivot][j] = tmp;
                }
            }

            // Normalize pivot row.
            const double inv_pivot = 1.0 / a[i][i];
            for (std::size_t j = 0; j < 12; ++j) a[i][j] *= inv_pivot;

            // Eliminate other rows.
            for (std::size_t r = 0; r < 6; ++r) {
                if (r == i) continue;
                const double factor = a[r][i];
                if (factor == 0.0) continue;
                for (std::size_t j = 0; j < 12; ++j) {
                    a[r][j] -= factor * a[i][j];
                }
            }
        }

        Matrix6 inv;
        for (std::size_t i = 0; i < 6; ++i)
            for (std::size_t j = 0; j < 6; ++j)
                inv.m[i][j] = a[i][6 + j];
        return inv;
    }
};

inline Matrix6 operator*(double s, const Matrix6& m) { return m * s; }

inline Vector6 operator*(const Matrix6& m, const Vector6& v) {
    Vector6 r;
    for (std::size_t i = 0; i < 6; ++i) {
        double s = 0.0;
        for (std::size_t j = 0; j < 6; ++j) s += m.m[i][j] * v.v[j];
        r.v[i] = s;
    }
    return r;
}

} // namespace pathfinder
