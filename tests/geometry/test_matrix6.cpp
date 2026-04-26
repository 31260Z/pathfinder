#include <doctest/doctest.h>

#include <pathfinder/geometry/matrix6.hpp>

#include <cmath>
#include <cstddef>
#include <stdexcept>

using namespace pathfinder;

namespace {
bool matrix6_approx_equal(const Matrix6& a, const Matrix6& b, double eps = 1e-9) {
    for (std::size_t i = 0; i < 6; ++i)
        for (std::size_t j = 0; j < 6; ++j)
            if (std::abs(a.m[i][j] - b.m[i][j]) > eps) return false;
    return true;
}
} // namespace

TEST_CASE("Matrix6: zero and identity factories") {
    Matrix6 z = Matrix6::zero();
    Matrix6 I = Matrix6::identity();
    for (std::size_t i = 0; i < 6; ++i) {
        for (std::size_t j = 0; j < 6; ++j) {
            CHECK(z.m[i][j] == 0.0);
            CHECK(I.m[i][j] == (i == j ? 1.0 : 0.0));
        }
    }
}

TEST_CASE("Matrix6: at() accessor reads and writes") {
    Matrix6 m;
    m.at(2, 3) = 4.5;
    CHECK(m.at(2, 3) == doctest::Approx(4.5));
    const Matrix6& cm = m;
    CHECK(cm.at(2, 3) == doctest::Approx(4.5));
}

TEST_CASE("Matrix6: identity * m == m") {
    Matrix6 m;
    int v = 1;
    for (std::size_t i = 0; i < 6; ++i)
        for (std::size_t j = 0; j < 6; ++j)
            m.m[i][j] = static_cast<double>(v++);
    // Make it non-singular by adding to the diagonal.
    for (std::size_t i = 0; i < 6; ++i) m.m[i][i] += 100.0;

    CHECK(matrix6_approx_equal(Matrix6::identity() * m, m));
    CHECK(matrix6_approx_equal(m * Matrix6::identity(), m));
}

TEST_CASE("Matrix6: addition and subtraction") {
    Matrix6 a = Matrix6::identity();
    Matrix6 b = Matrix6::identity();
    Matrix6 sum = a + b;
    for (std::size_t i = 0; i < 6; ++i) CHECK(sum.m[i][i] == doctest::Approx(2.0));
    CHECK(matrix6_approx_equal(a - b, Matrix6::zero()));
}

TEST_CASE("Matrix6: scalar multiplication left and right") {
    Matrix6 I = Matrix6::identity();
    Matrix6 s = I * 2.5;
    Matrix6 t = 2.5 * I;
    for (std::size_t i = 0; i < 6; ++i) {
        CHECK(s.m[i][i] == doctest::Approx(2.5));
        CHECK(t.m[i][i] == doctest::Approx(2.5));
    }
}

TEST_CASE("Matrix6: transpose involution") {
    Matrix6 m;
    int v = 1;
    for (std::size_t i = 0; i < 6; ++i)
        for (std::size_t j = 0; j < 6; ++j)
            m.m[i][j] = static_cast<double>(v++);
    Matrix6 t = m.transpose();
    CHECK(t.m[0][1] == m.m[1][0]);
    CHECK(t.m[3][5] == m.m[5][3]);
    CHECK(matrix6_approx_equal(t.transpose(), m));
}

TEST_CASE("Matrix6: matrix * vector arithmetic") {
    Matrix6 m;
    for (std::size_t i = 0; i < 6; ++i) {
        for (std::size_t j = 0; j < 6; ++j) {
            m.m[i][j] = static_cast<double>(i * 6 + j + 1);
        }
    }
    Vector6 v;
    for (std::size_t i = 0; i < 6; ++i) v[i] = static_cast<double>(i + 1);

    Vector6 r = m * v;
    // Row 0: 1·1 + 2·2 + 3·3 + 4·4 + 5·5 + 6·6 = 1+4+9+16+25+36 = 91
    CHECK(r[0] == doctest::Approx(91.0));
    // Row 1: 7·1 + 8·2 + 9·3 + 10·4 + 11·5 + 12·6 = 7+16+27+40+55+72 = 217
    CHECK(r[1] == doctest::Approx(217.0));
}

TEST_CASE("Matrix6: identity inverse is identity") {
    CHECK(matrix6_approx_equal(Matrix6::identity().inverse(), Matrix6::identity()));
}

TEST_CASE("Matrix6: inverse round-trip on a non-trivial matrix") {
    Matrix6 m = Matrix6::identity() * 2.0;
    // Add a few off-diagonal couplings to make sure the inverse really inverts
    // a non-diagonal system.
    m.m[0][1] = 0.5;  m.m[1][0] = 0.3;
    m.m[2][3] = 0.7;  m.m[3][2] = 0.1;
    m.m[4][5] = 0.2;  m.m[5][4] = 0.4;
    m.m[0][5] = 0.05; m.m[5][0] = 0.06;

    Matrix6 inv = m.inverse();
    CHECK(matrix6_approx_equal(m * inv, Matrix6::identity(), 1e-9));
    CHECK(matrix6_approx_equal(inv * m, Matrix6::identity(), 1e-9));
}

TEST_CASE("Matrix6: inverse with partial pivoting (zero pivot in top-left)") {
    // Force the algorithm to swap rows: row 0 has a zero in column 0.
    Matrix6 m = Matrix6::identity();
    m.m[0][0] = 0.0;
    m.m[0][1] = 1.0;   // row 0 = [0, 1, 0, 0, 0, 0]
    m.m[1][0] = 1.0;
    m.m[1][1] = 0.0;   // row 1 = [1, 0, 0, 0, 0, 0]
    // Rows 2..5 stay identity. The matrix is a permutation; inverse is itself.

    Matrix6 inv = m.inverse();
    CHECK(matrix6_approx_equal(m * inv, Matrix6::identity()));
    CHECK(matrix6_approx_equal(inv * m, Matrix6::identity()));
}

TEST_CASE("Matrix6: singular matrix throws") {
    CHECK_THROWS_AS(Matrix6::zero().inverse(), std::domain_error);
}

TEST_CASE("Matrix6: compound assignment") {
    Matrix6 a = Matrix6::identity();
    a += Matrix6::identity();
    CHECK(a.m[0][0] == doctest::Approx(2.0));
    a -= Matrix6::identity();
    CHECK(a.m[0][0] == doctest::Approx(1.0));
    a *= 3.0;
    CHECK(a.m[0][0] == doctest::Approx(3.0));
    a *= Matrix6::identity();
    CHECK(a.m[0][0] == doctest::Approx(3.0));
}

TEST_CASE("Vector6: arithmetic and indexing") {
    Vector6 a;
    Vector6 b;
    for (std::size_t i = 0; i < 6; ++i) {
        a[i] = static_cast<double>(i);
        b[i] = static_cast<double>(2 * i);
    }
    Vector6 sum = a + b;
    Vector6 diff = b - a;
    for (std::size_t i = 0; i < 6; ++i) {
        CHECK(sum[i] == doctest::Approx(static_cast<double>(3 * i)));
        CHECK(diff[i] == doctest::Approx(static_cast<double>(i)));
    }
    a += b;
    for (std::size_t i = 0; i < 6; ++i) CHECK(a[i] == doctest::Approx(static_cast<double>(3 * i)));
    a -= b;
    for (std::size_t i = 0; i < 6; ++i) CHECK(a[i] == doctest::Approx(static_cast<double>(i)));
}
