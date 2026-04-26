#include <doctest/doctest.h>

#include <pathfinder/geometry/matrix3.hpp>
#include <pathfinder/geometry/vector2.hpp>

using namespace pathfinder;

namespace {
bool matrix_approx_equal(const Matrix3& a, const Matrix3& b, double eps = 1e-9) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (std::abs(a.m[i][j] - b.m[i][j]) > eps) return false;
    return true;
}
} // namespace

TEST_CASE("Matrix3: identity and zero factories") {
    Matrix3 z = Matrix3::zero();
    Matrix3 I = Matrix3::identity();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            CHECK(z.m[i][j] == 0.0);
            CHECK(I.m[i][j] == (i == j ? 1.0 : 0.0));
        }
    }
}

TEST_CASE("Matrix3: identity * v == v") {
    Vector3 v{1.0, 2.0, 3.0};
    Vector3 r = Matrix3::identity() * v;
    CHECK(r.x == doctest::Approx(1.0));
    CHECK(r.y == doctest::Approx(2.0));
    CHECK(r.z == doctest::Approx(3.0));
}

TEST_CASE("Matrix3: identity * m == m") {
    Matrix3 m;
    m.m[0][0] = 1; m.m[0][1] = 2; m.m[0][2] = 3;
    m.m[1][0] = 4; m.m[1][1] = 5; m.m[1][2] = 6;
    m.m[2][0] = 7; m.m[2][1] = 8; m.m[2][2] = 10;  // make it non-singular
    CHECK(matrix_approx_equal(Matrix3::identity() * m, m));
    CHECK(matrix_approx_equal(m * Matrix3::identity(), m));
}

TEST_CASE("Matrix3: addition and subtraction") {
    Matrix3 a = Matrix3::identity();
    Matrix3 b = Matrix3::identity();
    Matrix3 sum = a + b;
    CHECK(sum.m[0][0] == 2.0);
    CHECK(sum.m[1][1] == 2.0);
    CHECK(sum.m[2][2] == 2.0);
    Matrix3 diff = a - b;
    CHECK(matrix_approx_equal(diff, Matrix3::zero()));
}

TEST_CASE("Matrix3: scalar multiplication") {
    Matrix3 I = Matrix3::identity();
    Matrix3 s = I * 3.5;
    CHECK(s.m[0][0] == doctest::Approx(3.5));
    CHECK(s.m[1][1] == doctest::Approx(3.5));
    CHECK(s.m[2][2] == doctest::Approx(3.5));
    Matrix3 s2 = 2.0 * I;
    CHECK(s2.m[0][0] == doctest::Approx(2.0));
}

TEST_CASE("Matrix3: transpose involution") {
    Matrix3 m;
    m.m[0][0] = 1; m.m[0][1] = 2; m.m[0][2] = 3;
    m.m[1][0] = 4; m.m[1][1] = 5; m.m[1][2] = 6;
    m.m[2][0] = 7; m.m[2][1] = 8; m.m[2][2] = 9;

    Matrix3 t = m.transpose();
    CHECK(t.m[0][1] == 4);
    CHECK(t.m[1][0] == 2);
    CHECK(t.m[0][2] == 7);

    CHECK(matrix_approx_equal(t.transpose(), m));
}

TEST_CASE("Matrix3: determinant on known matrices") {
    CHECK(Matrix3::identity().determinant() == doctest::Approx(1.0));
    CHECK(Matrix3::zero().determinant() == doctest::Approx(0.0));

    Matrix3 m;
    m.m[0][0] = 2; m.m[0][1] = 0; m.m[0][2] = 0;
    m.m[1][0] = 0; m.m[1][1] = 3; m.m[1][2] = 0;
    m.m[2][0] = 0; m.m[2][1] = 0; m.m[2][2] = 4;
    CHECK(m.determinant() == doctest::Approx(24.0));

    Matrix3 g;
    g.m[0][0] = 1; g.m[0][1] = 2; g.m[0][2] = 3;
    g.m[1][0] = 0; g.m[1][1] = 1; g.m[1][2] = 4;
    g.m[2][0] = 5; g.m[2][1] = 6; g.m[2][2] = 0;
    CHECK(g.determinant() == doctest::Approx(1.0));
}

TEST_CASE("Matrix3: inverse * matrix == identity") {
    Matrix3 m;
    m.m[0][0] = 1; m.m[0][1] = 2; m.m[0][2] = 3;
    m.m[1][0] = 0; m.m[1][1] = 1; m.m[1][2] = 4;
    m.m[2][0] = 5; m.m[2][1] = 6; m.m[2][2] = 0;

    Matrix3 inv = m.inverse();
    CHECK(matrix_approx_equal(m * inv, Matrix3::identity()));
    CHECK(matrix_approx_equal(inv * m, Matrix3::identity()));
}

TEST_CASE("Matrix3: inverse of identity is identity") {
    CHECK(matrix_approx_equal(Matrix3::identity().inverse(), Matrix3::identity()));
}

TEST_CASE("Matrix3: inverse of singular throws") {
    CHECK_THROWS_AS(Matrix3::zero().inverse(), std::domain_error);
}

TEST_CASE("Matrix3: matrix * vector arithmetic") {
    Matrix3 m;
    m.m[0][0] = 1; m.m[0][1] = 2; m.m[0][2] = 3;
    m.m[1][0] = 4; m.m[1][1] = 5; m.m[1][2] = 6;
    m.m[2][0] = 7; m.m[2][1] = 8; m.m[2][2] = 9;

    Vector3 v{1, 2, 3};
    Vector3 r = m * v;
    CHECK(r.x == doctest::Approx(14.0));
    CHECK(r.y == doctest::Approx(32.0));
    CHECK(r.z == doctest::Approx(50.0));
}

TEST_CASE("Vector3: arithmetic") {
    Vector3 a{1, 2, 3};
    Vector3 b{4, 5, 6};
    CHECK((a + b) == Vector3{5, 7, 9});
    CHECK((b - a) == Vector3{3, 3, 3});
    CHECK((-a) == Vector3{-1, -2, -3});
    CHECK((a * 2.0) == Vector3{2, 4, 6});
    CHECK((2.0 * a) == Vector3{2, 4, 6});
}

TEST_CASE("Matrix3: compound assignment") {
    Matrix3 a = Matrix3::identity();
    a += Matrix3::identity();
    CHECK(a.m[0][0] == 2.0);
    a -= Matrix3::identity();
    CHECK(a.m[0][0] == 1.0);
    a *= 3.0;
    CHECK(a.m[0][0] == 3.0);
}
