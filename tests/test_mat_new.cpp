#include <cmath>
#include <cutil/mat.hpp>
#include <doctest.h>

using namespace Cutil;

// Helper: build a matrix from row-major initializer list (for readability in tests)
// Internal storage is column-major, so this is a convenience wrapper
template <unsigned int R, unsigned int C, typename T = float>
Mat<R, C, T> make_mat(std::initializer_list<T> row_major) {
  auto it = row_major.begin();
  Mat<R, C, T> m;
  for(unsigned int r = 0; r < R; r++)
    for(unsigned int c = 0; c < C; c++)
      m(r, c) = *it++;
  return m;
}

// Helper: check Mat<N,N> ~ identity within tolerance
template <unsigned int N, typename T>
bool is_identity(const Mat<N, N, T>& m, double tol = 1e-4) {
  for(unsigned int r = 0; r < N; r++)
    for(unsigned int c = 0; c < N; c++) {
      double expected = (r == c) ? 1.0 : 0.0;
      if(std::abs(static_cast<double>(m(r, c)) - expected) > tol) return false;
    }
  return true;
}

// Helper: check two matrices are element-wise close
template <unsigned int R, unsigned int C, typename T>
bool mat_approx_eq(const Mat<R, C, T>& a, const Mat<R, C, T>& b, double tol = 1e-4) {
  for(unsigned int i = 0; i < R * C; i++)
    if(std::abs(static_cast<double>(a[i]) - static_cast<double>(b[i])) > tol) return false;
  return true;
}

// ===== Constructors / factories =============================================

TEST_SUITE("Mat - Constructors") {
  TEST_CASE("Default constructor zeros all elements") {
    Mat3f m;
    for(unsigned int i = 0; i < 9; i++) CHECK(m[i] == 0.0f);
  }

  TEST_CASE("Identity factory: 3x3") {
    Mat3f m = Mat3f::identity();
    CHECK(m(0, 0) == 1.0f); CHECK(m(1, 1) == 1.0f); CHECK(m(2, 2) == 1.0f);
    CHECK(m(0, 1) == 0.0f); CHECK(m(0, 2) == 0.0f);
    CHECK(m(1, 0) == 0.0f); CHECK(m(2, 0) == 0.0f);
  }

  TEST_CASE("Identity factory: 4x4") {
    Mat4f m = Mat4f::identity();
    CHECK(is_identity(m));
  }

  TEST_CASE("Zeros factory") {
    Mat3f m = Mat3f::zeros();
    for(unsigned int i = 0; i < 9; i++) CHECK(m[i] == 0.0f);
  }

  TEST_CASE("Ones factory") {
    Mat3f m = Mat3f::ones();
    for(unsigned int i = 0; i < 9; i++) CHECK(m[i] == 1.0f);
  }

  TEST_CASE("Copy constructor") {
    Mat3f a = Mat3f::identity();
    a(0, 2)  = 7.0f;
    Mat3f b  = a;
    CHECK(b(0, 2) == 7.0f);
    b(0, 2) = 99.0f;
    CHECK(a(0, 2) == 7.0f); // independent copy
  }

  TEST_CASE("Assignment operator") {
    Mat3f a = Mat3f::identity();
    Mat3f b;
    b = a;
    CHECK(b(1, 1) == 1.0f);
    b(1, 1) = 0.0f;
    CHECK(a(1, 1) == 1.0f); // independent
  }
}

// ===== Element access =======================================================

TEST_SUITE("Mat - Element Access") {
  TEST_CASE("(row, col) round-trip for 3x3") {
    Mat3f m;
    for(unsigned int r = 0; r < 3; r++)
      for(unsigned int c = 0; c < 3; c++)
        m(r, c) = static_cast<float>(r * 3 + c + 1);
    for(unsigned int r = 0; r < 3; r++)
      for(unsigned int c = 0; c < 3; c++)
        CHECK(m(r, c) == static_cast<float>(r * 3 + c + 1));
  }

  TEST_CASE("Column-major layout") {
    Mat<2, 2, float> m;
    m(0, 0) = 1; m(1, 0) = 2; // col 0
    m(0, 1) = 3; m(1, 1) = 4; // col 1
    // column-major: data[col*Rows+row]  →  [1, 2, 3, 4]
    CHECK(m[0] == 1.0f);
    CHECK(m[1] == 2.0f);
    CHECK(m[2] == 3.0f);
    CHECK(m[3] == 4.0f);
  }

  TEST_CASE("col() and row() on identity") {
    Mat3f m = Mat3f::identity();
    CHECK(m.col(0) == Vec3f(1, 0, 0));
    CHECK(m.col(1) == Vec3f(0, 1, 0));
    CHECK(m.col(2) == Vec3f(0, 0, 1));
    CHECK(m.row(0) == Vec3f(1, 0, 0));
    CHECK(m.row(1) == Vec3f(0, 1, 0));
    CHECK(m.row(2) == Vec3f(0, 0, 1));
  }

  TEST_CASE("set_col / set_row round-trip") {
    Mat3f m = Mat3f::zeros();
    m.set_col(0, Vec3f(1, 2, 3));
    CHECK(m(0, 0) == 1); CHECK(m(1, 0) == 2); CHECK(m(2, 0) == 3);
    m.set_row(1, Vec3f(10, 20, 30));
    CHECK(m(1, 0) == 10); CHECK(m(1, 1) == 20); CHECK(m(1, 2) == 30);
  }
}

// ===== Arithmetic ===========================================================

TEST_SUITE("Mat - Arithmetic") {
  TEST_CASE("Add and subtract") {
    Mat3f a = Mat3f::identity();
    Mat3f b = Mat3f::identity();
    Mat3f c = a + b;
    CHECK(c(0, 0) == 2.0f);
    CHECK(c(0, 1) == 0.0f);
    Mat3f d = c - a;
    CHECK(mat_approx_eq(d, b));
  }

  TEST_CASE("Scalar multiply/divide") {
    Mat3f m = Mat3f::identity();
    Mat3f m2 = m * 5.0f;
    CHECK(m2(0, 0) == 5.0f);
    CHECK(m2(0, 1) == 0.0f);
    Mat3f m3 = 5.0f * m;
    CHECK(mat_approx_eq(m2, m3));
    Mat3f m4 = m2 / 5.0f;
    CHECK(mat_approx_eq(m4, m));
  }

  TEST_CASE("Negation") {
    Mat3f m;
    m(0, 1) = 3.0f;
    Mat3f n = -m;
    CHECK(n(0, 1) == -3.0f);
  }

  TEST_CASE("Compound += / -= / *=") {
    Mat3f a = Mat3f::identity();
    a += Mat3f::identity();
    CHECK(a(0, 0) == 2.0f);
    a -= Mat3f::identity();
    CHECK(a(0, 0) == 1.0f);
    a *= 4.0f;
    CHECK(a(0, 0) == 4.0f);
  }
}

// ===== Matrix Multiplication ================================================

TEST_SUITE("Mat - Matrix Multiplication") {
  TEST_CASE("A * Identity = A (3x3)") {
    Mat3f a = make_mat<3, 3, float>({1, 2, 3, 4, 5, 6, 7, 8, 9});
    Mat3f r = a * Mat3f::identity();
    CHECK(mat_approx_eq(r, a));
  }

  TEST_CASE("Identity * A = A (3x3)") {
    Mat3f a = make_mat<3, 3, float>({1, 2, 3, 4, 5, 6, 7, 8, 9});
    Mat3f r = Mat3f::identity() * a;
    CHECK(mat_approx_eq(r, a));
  }

  TEST_CASE("A * Zero = Zero") {
    Mat3f a = Mat3f::identity();
    Mat3f r = a * Mat3f::zeros();
    CHECK(mat_approx_eq(r, Mat3f::zeros()));
  }

  TEST_CASE("2x2 known product") {
    // [ 1 2 ] * [ 5 6 ] = [ 1*5+2*7  1*6+2*8 ] = [ 19 22 ]
    // [ 3 4 ]   [ 7 8 ]   [ 3*5+4*7  3*6+4*8 ]   [ 43 50 ]
    auto a = make_mat<2, 2, float>({1, 2, 3, 4});
    auto b = make_mat<2, 2, float>({5, 6, 7, 8});
    auto c = a * b;
    CHECK(doctest::Approx(c(0, 0)) == 19.0f);
    CHECK(doctest::Approx(c(0, 1)) == 22.0f);
    CHECK(doctest::Approx(c(1, 0)) == 43.0f);
    CHECK(doctest::Approx(c(1, 1)) == 50.0f);
  }

  TEST_CASE("3x3 known product") {
    // A = diag(1,2,3), B = all-twos → C(i,j) = 2*A(i,i)
    Mat3f a;
    a(0, 0) = 1; a(1, 1) = 2; a(2, 2) = 3;
    Mat3f b = Mat3f::ones();
    Mat3f c = a * b;
    // row 0 of result: 1*[1,1,1] = [1,1,1]
    CHECK(doctest::Approx(c(0, 0)) == 1.0f);
    CHECK(doctest::Approx(c(0, 1)) == 1.0f);
    CHECK(doctest::Approx(c(0, 2)) == 1.0f);
    // row 1: 2*[1,1,1]
    CHECK(doctest::Approx(c(1, 0)) == 2.0f);
    CHECK(doctest::Approx(c(1, 1)) == 2.0f);
    // row 2: 3*[1,1,1]
    CHECK(doctest::Approx(c(2, 0)) == 3.0f);
  }

  TEST_CASE("4x4 known product") {
    Mat4f a;
    a(0, 0) = 2; a(1, 1) = 3; a(2, 2) = 4; a(3, 3) = 5;
    Mat4f b;
    b(0, 0) = 10; b(1, 1) = 10; b(2, 2) = 10; b(3, 3) = 10;
    Mat4f c = a * b;
    CHECK(doctest::Approx(c(0, 0)) == 20.0f);
    CHECK(doctest::Approx(c(1, 1)) == 30.0f);
    CHECK(doctest::Approx(c(2, 2)) == 40.0f);
    CHECK(doctest::Approx(c(3, 3)) == 50.0f);
    CHECK(doctest::Approx(c(0, 1)) == 0.0f);
  }

  TEST_CASE("Associativity: (A*B)*C == A*(B*C) (3x3)") {
    auto a = make_mat<3, 3, float>({1, 2, 0, 0, 3, 1, 0, 0, 4});
    auto b = make_mat<3, 3, float>({2, 0, 1, 1, 3, 0, 0, 2, 1});
    auto c = make_mat<3, 3, float>({0, 1, 2, 3, 0, 1, 1, 2, 0});
    Mat3f lhs = (a * b) * c;
    Mat3f rhs = a * (b * c);
    CHECK(mat_approx_eq(lhs, rhs, 1e-4f));
  }

  TEST_CASE("Associativity: 4x4 (chained transforms)") {
    Mat4f tx = mat4_translation<float>(Vec3f(1, 2, 3));
    Mat4f sc = mat4_scale<float>(Vec3f(2, 2, 2));
    Mat4f rx = mat4_rotation_x<float>(0.5f);
    Mat4f lhs = (tx * sc) * rx;
    Mat4f rhs = tx * (sc * rx);
    CHECK(mat_approx_eq(lhs, rhs, 1e-4f));
  }

  TEST_CASE("Non-square multiplication 2x3 * 3x4 = 2x4") {
    // A (2x3): [1 2 3; 4 5 6]
    // B (3x4): [1 0 0 0; 0 1 0 0; 0 0 1 0]  = 3x4 pseudo-identity
    // A*B = A with an extra zero column
    auto a  = make_mat<2, 3, float>({1, 2, 3, 4, 5, 6});
    Mat<3, 4, float> b;
    b(0, 0) = 1; b(1, 1) = 1; b(2, 2) = 1;
    auto c = a * b;
    static_assert(std::is_same<decltype(c), Mat<2, 4, float>>::value, "dim check");
    CHECK(doctest::Approx(c(0, 0)) == 1.0f);
    CHECK(doctest::Approx(c(0, 1)) == 2.0f);
    CHECK(doctest::Approx(c(0, 2)) == 3.0f);
    CHECK(doctest::Approx(c(0, 3)) == 0.0f);
    CHECK(doctest::Approx(c(1, 0)) == 4.0f);
    CHECK(doctest::Approx(c(1, 1)) == 5.0f);
    CHECK(doctest::Approx(c(1, 2)) == 6.0f);
  }

  TEST_CASE("Chained 3 multiplications: translate, rotate, scale") {
    Mat4f t = mat4_translation<float>(Vec3f(1, 0, 0));
    Mat4f r = mat4_rotation_z<float>(0.0f);
    Mat4f s = mat4_scale<float>(Vec3f(3, 3, 3));
    Vec4f p(1.0f, 0.0f, 0.0f, 1.0f);

    // Scale first, then rotate (identity at 0), then translate
    Vec4f result = (t * r * s) * p;
    // s*p = (3,0,0,1), r*(3,0,0,1) = (3,0,0,1), t*(3,0,0,1) = (4,0,0,1)
    CHECK(doctest::Approx(result[0]) == 4.0f);
    CHECK(doctest::Approx(result[1]) == 0.0f);
    CHECK(doctest::Approx(result[2]) == 0.0f);
    CHECK(doctest::Approx(result[3]) == 1.0f);
  }
}

// ===== Matrix-Vector Multiplication =========================================

TEST_SUITE("Mat - Matrix-Vector Multiplication") {
  TEST_CASE("Identity * vec = vec (3)") {
    Vec3f v(1, 2, 3);
    CHECK(Mat3f::identity() * v == v);
  }

  TEST_CASE("Diagonal scale * vec") {
    Mat3f m;
    m(0, 0) = 2; m(1, 1) = 3; m(2, 2) = 4;
    Vec3f v(1, 1, 1);
    Vec3f r = m * v;
    CHECK(r[0] == 2.0f); CHECK(r[1] == 3.0f); CHECK(r[2] == 4.0f);
  }

  TEST_CASE("Translation via 4x4 * vec4") {
    Mat4f t = mat4_translation<float>(Vec3f(5, 10, 15));
    Vec4f p(1, 2, 3, 1); // w=1 → point
    Vec4f r = t * p;
    CHECK(doctest::Approx(r[0]) == 6.0f);
    CHECK(doctest::Approx(r[1]) == 12.0f);
    CHECK(doctest::Approx(r[2]) == 18.0f);
    CHECK(doctest::Approx(r[3]) == 1.0f);
  }

  TEST_CASE("Translation does NOT affect directions (w=0)") {
    Mat4f t = mat4_translation<float>(Vec3f(5, 10, 15));
    Vec4f d(1, 0, 0, 0); // w=0 → direction
    Vec4f r = t * d;
    CHECK(doctest::Approx(r[0]) == 1.0f);
    CHECK(doctest::Approx(r[1]) == 0.0f);
    CHECK(doctest::Approx(r[2]) == 0.0f);
    CHECK(doctest::Approx(r[3]) == 0.0f);
  }

  TEST_CASE("Scale via 4x4 * vec4") {
    Mat4f s = mat4_scale<float>(Vec3f(2, 3, 4));
    Vec4f p(1, 2, 3, 1);
    Vec4f r = s * p;
    CHECK(doctest::Approx(r[0]) == 2.0f);
    CHECK(doctest::Approx(r[1]) == 6.0f);
    CHECK(doctest::Approx(r[2]) == 12.0f);
  }

  TEST_CASE("RotX(pi/2) maps +Y to +Z") {
    Mat4f rx = mat4_rotation_x<float>(M_PI / 2.0);
    Vec4f y_axis(0, 1, 0, 0);
    Vec4f r = rx * y_axis;
    CHECK(doctest::Approx(r[0]).epsilon(1e-5) == 0.0f);
    CHECK(doctest::Approx(r[1]).epsilon(1e-5) == 0.0f);
    CHECK(doctest::Approx(r[2]).epsilon(1e-5) == 1.0f);
  }

  TEST_CASE("RotY(pi/2) maps +X to -Z") {
    Mat4f ry = mat4_rotation_y<float>(M_PI / 2.0);
    Vec4f x_axis(1, 0, 0, 0);
    Vec4f r = ry * x_axis;
    CHECK(doctest::Approx(r[0]).epsilon(1e-5) == 0.0f);
    CHECK(doctest::Approx(r[1]).epsilon(1e-5) == 0.0f);
    CHECK(doctest::Approx(r[2]).epsilon(1e-5) == -1.0f);
  }

  TEST_CASE("RotZ(pi/2) maps +X to +Y") {
    Mat4f rz = mat4_rotation_z<float>(M_PI / 2.0);
    Vec4f x_axis(1, 0, 0, 0);
    Vec4f r = rz * x_axis;
    CHECK(doctest::Approx(r[0]).epsilon(1e-5) == 0.0f);
    CHECK(doctest::Approx(r[1]).epsilon(1e-5) == 1.0f);
    CHECK(doctest::Approx(r[2]).epsilon(1e-5) == 0.0f);
  }
}

// ===== Trace ================================================================

TEST_SUITE("Mat - Trace") {
  TEST_CASE("Trace of 3x3 identity = 3") {
    CHECK(Mat3f::identity().trace() == 3.0f);
  }

  TEST_CASE("Trace of 4x4 identity = 4") {
    CHECK(Mat4f::identity().trace() == 4.0f);
  }

  TEST_CASE("Trace of zeros = 0") {
    CHECK(Mat4f::zeros().trace() == 0.0f);
  }

  TEST_CASE("Trace of known 3x3") {
    // [[1,2,3],[4,5,6],[7,8,9]] → trace = 1+5+9 = 15
    auto m = make_mat<3, 3, float>({1, 2, 3, 4, 5, 6, 7, 8, 9});
    CHECK(m.trace() == 15.0f);
  }

  TEST_CASE("Trace of 2x2") {
    auto m = make_mat<2, 2, float>({3, 7, 2, 5});
    CHECK(m.trace() == 8.0f); // 3 + 5
  }

  TEST_CASE("Linearity: trace(A + B) = trace(A) + trace(B)") {
    auto a = make_mat<3, 3, float>({1, 2, 3, 4, 5, 6, 7, 8, 9});
    auto b = make_mat<3, 3, float>({9, 8, 7, 6, 5, 4, 3, 2, 1});
    float ta = a.trace();
    float tb = b.trace();
    float tab = (a + b).trace();
    CHECK(doctest::Approx(tab) == ta + tb);
  }

  TEST_CASE("Cyclic property: trace(A*B) = trace(B*A) (3x3)") {
    auto a = make_mat<3, 3, float>({1, 2, 3, 4, 5, 6, 7, 8, 10});
    auto b = make_mat<3, 3, float>({2, 0, 1, 1, 3, 0, 0, 2, 1});
    float tab = (a * b).trace();
    float tba = (b * a).trace();
    CHECK(doctest::Approx(tab).epsilon(1e-3) == tba);
  }

  TEST_CASE("Trace of diagonal scale matrix") {
    Mat4f s = mat4_scale<float>(Vec3f(2, 3, 4));
    // Diagonal: [2, 3, 4, 1] → trace = 10
    CHECK(s.trace() == doctest::Approx(10.0f));
  }
}

// ===== Determinant ==========================================================

TEST_SUITE("Mat - Determinant") {
  TEST_CASE("det of 1x1") {
    Mat<1, 1, float> m;
    m(0, 0) = 7.0f;
    CHECK(doctest::Approx(m.det()) == 7.0f);
  }

  TEST_CASE("det of 2x2 = +7") {
    // [ 3 4 ]   det = 3*5 - 4*2 = 7
    // [ 2 5 ]
    auto m = make_mat<2, 2, float>({3, 4, 2, 5});
    CHECK(doctest::Approx(m.det()) == 7.0f);
  }

  TEST_CASE("det of 2x2 = -2") {
    // [ 1 2 ]   det = 1*4 - 2*3 = -2
    // [ 3 4 ]
    auto m = make_mat<2, 2, float>({1, 2, 3, 4});
    CHECK(doctest::Approx(m.det()) == -2.0f);
  }

  TEST_CASE("det of 2x2 singular = 0") {
    // [ 2 4 ]   rows proportional
    // [ 1 2 ]
    auto m = make_mat<2, 2, float>({2, 4, 1, 2});
    CHECK(doctest::Approx(m.det()).epsilon(1e-5) == 0.0f);
  }

  TEST_CASE("det of 2x2 zero matrix = 0") {
    CHECK(doctest::Approx(Mat<2, 2, float>::zeros().det()) == 0.0f);
  }

  TEST_CASE("det of 2x2 identity = 1") {
    CHECK(doctest::Approx(Mat<2, 2, float>::identity().det()) == 1.0f);
  }

  TEST_CASE("det of 3x3 = 22") {
    // [1 2 3; 0 4 5; 1 0 6]
    // = 1*(4*6-5*0) - 2*(0*6-5*1) + 3*(0*0-4*1)
    // = 24 + 10 - 12 = 22
    auto m = make_mat<3, 3, float>({1, 2, 3, 0, 4, 5, 1, 0, 6});
    CHECK(doctest::Approx(m.det()).epsilon(1e-4) == 22.0f);
  }

  TEST_CASE("det of 3x3 = -1") {
    // Permutation matrix (row swap of identity): det = -1
    // [0 1 0; 1 0 0; 0 0 1]
    auto m = make_mat<3, 3, float>({0, 1, 0, 1, 0, 0, 0, 0, 1});
    CHECK(doctest::Approx(m.det()) == -1.0f);
  }

  TEST_CASE("det of 3x3 singular (dependent rows) = 0") {
    // Row2 = Row0 + Row1
    auto m = make_mat<3, 3, float>({1, 2, 3, 4, 5, 6, 5, 7, 9});
    CHECK(doctest::Approx(m.det()).epsilon(1e-4) == 0.0f);
  }

  TEST_CASE("det of 3x3 upper-triangular = product of diagonal") {
    // [[2,3,4],[0,5,6],[0,0,7]] → det = 2*5*7 = 70
    auto m = make_mat<3, 3, float>({2, 3, 4, 0, 5, 6, 0, 0, 7});
    CHECK(doctest::Approx(m.det()).epsilon(1e-3) == 70.0f);
  }

  TEST_CASE("det of 3x3 lower-triangular = product of diagonal") {
    // [[3,0,0],[5,4,0],[6,7,2]] → det = 3*4*2 = 24
    auto m = make_mat<3, 3, float>({3, 0, 0, 5, 4, 0, 6, 7, 2});
    CHECK(doctest::Approx(m.det()).epsilon(1e-3) == 24.0f);
  }

  TEST_CASE("det of 4x4 identity = 1") {
    CHECK(doctest::Approx(Mat4f::identity().det()) == 1.0f);
  }

  TEST_CASE("det of 4x4 diagonal = product of diagonal") {
    Mat4f m;
    m(0, 0) = 2; m(1, 1) = 3; m(2, 2) = 4; m(3, 3) = 5;
    CHECK(doctest::Approx(m.det()).epsilon(1e-3) == 120.0f); // 2*3*4*5
  }

  TEST_CASE("det of 4x4 singular = 0") {
    auto m = make_mat<4, 4, float>({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16});
    CHECK(doctest::Approx(m.det()).epsilon(1e-1) == 0.0f);
  }

  TEST_CASE("det(scalar * I_n) = scalar^n") {
    // det(2*I_3) = 2^3 = 8
    Mat3f m = Mat3f::identity() * 2.0f;
    CHECK(doctest::Approx(m.det()).epsilon(1e-4) == 8.0f);
  }

  TEST_CASE("det(A * B) = det(A) * det(B)  (3x3)") {
    auto a = make_mat<3, 3, float>({1, 2, 3, 0, 4, 5, 1, 0, 6});
    auto b = make_mat<3, 3, float>({2, 0, 1, 1, 3, 0, 0, 2, 1});
    float dab = (a * b).det();
    float da  = a.det();
    float db  = b.det();
    CHECK(doctest::Approx(dab).epsilon(1e-2) == da * db);
  }

  TEST_CASE("det(A * B) = det(A) * det(B)  (4x4)") {
    auto a = make_mat<4, 4, float>({2, 0, 0, 0, 0, 3, 0, 0, 0, 0, 5, 0, 0, 0, 0, 7});
    auto b = make_mat<4, 4, float>({1, 2, 0, 0, 0, 1, 3, 0, 0, 0, 1, 4, 0, 0, 0, 1});
    float dab = (a * b).det();
    float da  = a.det();
    float db  = b.det();
    CHECK(doctest::Approx(dab).epsilon(1e-2) == da * db);
  }

  TEST_CASE("det(A^T) == det(A)  (3x3)") {
    auto a = make_mat<3, 3, float>({1, 2, 3, 0, 4, 5, 1, 0, 6});
    CHECK(doctest::Approx(a.transposed().det()).epsilon(1e-3) == a.det());
  }

  TEST_CASE("Rotation matrix: det = 1") {
    for(float angle : {0.0f, 0.5f, 1.0f, 2.0f, (float)M_PI}) {
      Mat4f rx = mat4_rotation_x<float>(angle);
      CHECK(doctest::Approx(rx.det()).epsilon(1e-4) == 1.0f);
    }
  }

  TEST_CASE("Scale matrix: det = product of scales") {
    Mat4f s = mat4_scale<float>(Vec3f(2, 3, 5));
    CHECK(doctest::Approx(s.det()).epsilon(1e-4) == 30.0f); // 2*3*5*1
  }

  TEST_CASE("Translation matrix: det = 1") {
    Mat4f t = mat4_translation<float>(Vec3f(100, 200, 300));
    CHECK(doctest::Approx(t.det()).epsilon(1e-4) == 1.0f);
  }
}

// ===== Transpose ============================================================

TEST_SUITE("Mat - Transpose") {
  TEST_CASE("3x3: (A^T)^T = A") {
    auto a  = make_mat<3, 3, float>({1, 2, 3, 4, 5, 6, 7, 8, 9});
    auto tt = a.transposed().transposed();
    CHECK(mat_approx_eq(tt, a));
  }

  TEST_CASE("3x3 known transpose") {
    // [1 2 3; 4 5 6; 7 8 9]^T = [1 4 7; 2 5 8; 3 6 9]
    auto a  = make_mat<3, 3, float>({1, 2, 3, 4, 5, 6, 7, 8, 9});
    auto at = a.transposed();
    CHECK(at(0, 0) == 1); CHECK(at(0, 1) == 4); CHECK(at(0, 2) == 7);
    CHECK(at(1, 0) == 2); CHECK(at(1, 1) == 5); CHECK(at(1, 2) == 8);
    CHECK(at(2, 0) == 3); CHECK(at(2, 1) == 6); CHECK(at(2, 2) == 9);
  }

  TEST_CASE("Non-square transpose 2x3 → 3x2") {
    auto a  = make_mat<2, 3, float>({1, 2, 3, 4, 5, 6});
    auto at = a.transposed();
    static_assert(std::is_same<decltype(at), Mat<3, 2, float>>::value, "dim check");
    CHECK(at(0, 0) == 1); CHECK(at(0, 1) == 4);
    CHECK(at(1, 0) == 2); CHECK(at(1, 1) == 5);
    CHECK(at(2, 0) == 3); CHECK(at(2, 1) == 6);
  }

  TEST_CASE("In-place transpose 3x3") {
    auto a   = make_mat<3, 3, float>({1, 2, 3, 4, 5, 6, 7, 8, 9});
    auto ref = a.transposed();
    a.transpose();
    CHECK(mat_approx_eq(a, ref));
  }

  TEST_CASE("In-place transpose identity remains identity") {
    Mat3f m = Mat3f::identity();
    m.transpose();
    CHECK(mat_approx_eq(m, Mat3f::identity()));
  }

  TEST_CASE("(A * B)^T = B^T * A^T") {
    auto a  = make_mat<3, 3, float>({1, 2, 3, 0, 4, 5, 1, 0, 6});
    auto b  = make_mat<3, 3, float>({2, 0, 1, 1, 3, 0, 0, 2, 1});
    auto lhs = (a * b).transposed();
    auto rhs = b.transposed() * a.transposed();
    CHECK(mat_approx_eq(lhs, rhs, 1e-4f));
  }
}

// ===== Inverse ==============================================================

TEST_SUITE("Mat - Inverse") {
  TEST_CASE("inv(I_3) = I_3") {
    Mat3f inv = Mat3f::identity().inverse();
    CHECK(is_identity(inv));
  }

  TEST_CASE("inv(I_4) = I_4") {
    Mat4f inv = Mat4f::identity().inverse();
    CHECK(is_identity(inv));
  }

  TEST_CASE("A * inv(A) = I  (2x2)") {
    auto a   = make_mat<2, 2, float>({3, 4, 2, 5});
    auto inv = a.inverse();
    CHECK(is_identity(a * inv, 1e-4));
  }

  TEST_CASE("inv(A) * A = I  (2x2)") {
    auto a   = make_mat<2, 2, float>({3, 4, 2, 5});
    auto inv = a.inverse();
    CHECK(is_identity(inv * a, 1e-4));
  }

  TEST_CASE("A * inv(A) = I  (3x3)") {
    auto a   = make_mat<3, 3, float>({1, 2, 3, 0, 4, 5, 1, 0, 6});
    auto inv = a.inverse();
    CHECK(is_identity(a * inv, 1e-4));
  }

  TEST_CASE("inv(A) * A = I  (3x3)") {
    auto a   = make_mat<3, 3, float>({1, 2, 3, 0, 4, 5, 1, 0, 6});
    auto inv = a.inverse();
    CHECK(is_identity(inv * a, 1e-4));
  }

  TEST_CASE("A * inv(A) = I  (4x4 diagonal)") {
    Mat4f a;
    a(0, 0) = 2; a(1, 1) = 3; a(2, 2) = 4; a(3, 3) = 5;
    CHECK(is_identity(a * a.inverse(), 1e-4));
  }

  TEST_CASE("A * inv(A) = I  (4x4 general)") {
    auto a = make_mat<4, 4, float>({2, 1, 0, 0, 0, 3, 1, 0, 0, 0, 4, 1, 0, 0, 0, 5});
    CHECK(is_identity(a * a.inverse(), 1e-4));
  }

  TEST_CASE("inv(inv(A)) ~ A  (3x3)") {
    auto a      = make_mat<3, 3, float>({1, 2, 3, 0, 4, 5, 1, 0, 6});
    auto inv_a  = a.inverse();
    auto inv_inv = inv_a.inverse();
    CHECK(mat_approx_eq(inv_inv, a, 1e-3f));
  }

  TEST_CASE("det(inv(A)) = 1/det(A)  (3x3)") {
    auto a   = make_mat<3, 3, float>({1, 2, 3, 0, 4, 5, 1, 0, 6});
    float da = a.det();
    float di = a.inverse().det();
    CHECK(doctest::Approx(di).epsilon(1e-3) == 1.0f / da);
  }

  TEST_CASE("inv(A * B) = inv(B) * inv(A)  (3x3)") {
    auto a    = make_mat<3, 3, float>({1, 2, 3, 0, 4, 5, 1, 0, 6});
    auto b    = make_mat<3, 3, float>({2, 0, 1, 1, 3, 0, 0, 2, 1});
    auto lhs  = (a * b).inverse();
    auto rhs  = b.inverse() * a.inverse();
    CHECK(mat_approx_eq(lhs, rhs, 1e-3f));
  }

  TEST_CASE("Inverse of diagonal matrix = reciprocal diagonal") {
    Mat3f m;
    m(0, 0) = 2; m(1, 1) = 4; m(2, 2) = 8;
    Mat3f inv = m.inverse();
    CHECK(doctest::Approx(inv(0, 0)).epsilon(1e-5) == 0.5f);
    CHECK(doctest::Approx(inv(1, 1)).epsilon(1e-5) == 0.25f);
    CHECK(doctest::Approx(inv(2, 2)).epsilon(1e-5) == 0.125f);
    CHECK(doctest::Approx(inv(0, 1)).epsilon(1e-5) == 0.0f);
  }

  TEST_CASE("Scale matrix inverse = reciprocal scale") {
    Mat4f s   = mat4_scale<float>(Vec3f(2, 4, 8));
    Mat4f inv = s.inverse();
    CHECK(doctest::Approx(inv(0, 0)) == 0.5f);
    CHECK(doctest::Approx(inv(1, 1)) == 0.25f);
    CHECK(doctest::Approx(inv(2, 2)) == 0.125f);
  }

  TEST_CASE("Rotation inverse = transpose") {
    for(float angle : {0.3f, 1.0f, 2.5f}) {
      Mat4f rx = mat4_rotation_x<float>(angle);
      Mat4f inv = rx.inverse();
      Mat4f transposed = rx.transposed();
      CHECK(mat_approx_eq(inv, transposed, 1e-4f));
    }
  }
}

// ===== Frobenius Norm ========================================================

TEST_SUITE("Mat - Frobenius Norm") {
  TEST_CASE("||I_3||_F = sqrt(3)") {
    CHECK(doctest::Approx(Mat3f::identity().frobenius_norm()) == std::sqrt(3.0));
  }

  TEST_CASE("||I_4||_F = sqrt(4) = 2") {
    CHECK(doctest::Approx(Mat4f::identity().frobenius_norm()) == 2.0);
  }

  TEST_CASE("||zeros||_F = 0") {
    CHECK(doctest::Approx(Mat3f::zeros().frobenius_norm()) == 0.0);
  }

  TEST_CASE("||scalar * A||_F = |scalar| * ||A||_F") {
    auto a  = make_mat<3, 3, float>({1, 2, 3, 4, 5, 6, 7, 8, 9});
    double na = a.frobenius_norm();
    double ns = (a * 3.0f).frobenius_norm();
    CHECK(doctest::Approx(ns).epsilon(1e-4) == 3.0 * na);
  }
}

// ===== Rotation Properties ==================================================

TEST_SUITE("Mat - Rotation Properties") {
  TEST_CASE("R * R^T = I for RotX") {
    for(float angle : {0.0f, 0.5f, 1.2f, (float)M_PI}) {
      Mat4f r  = mat4_rotation_x<float>(angle);
      Mat4f rt = r.transposed();
      CHECK(is_identity(r * rt, 1e-4));
    }
  }

  TEST_CASE("R * R^T = I for RotY") {
    for(float angle : {0.1f, 1.0f, 2.0f}) {
      Mat4f r = mat4_rotation_y<float>(angle);
      CHECK(is_identity(r * r.transposed(), 1e-4));
    }
  }

  TEST_CASE("R * R^T = I for RotZ") {
    for(float angle : {0.1f, 1.0f, 2.0f}) {
      Mat4f r = mat4_rotation_z<float>(angle);
      CHECK(is_identity(r * r.transposed(), 1e-4));
    }
  }

  TEST_CASE("det(R) = 1 for all rotation axes") {
    float angle = 1.234f;
    CHECK(doctest::Approx(mat4_rotation_x<float>(angle).det()).epsilon(1e-4) == 1.0f);
    CHECK(doctest::Approx(mat4_rotation_y<float>(angle).det()).epsilon(1e-4) == 1.0f);
    CHECK(doctest::Approx(mat4_rotation_z<float>(angle).det()).epsilon(1e-4) == 1.0f);
  }

  TEST_CASE("Rx(a) * Rx(-a) = I") {
    float angle = 0.7f;
    Mat4f rx_pos = mat4_rotation_x<float>(angle);
    Mat4f rx_neg = mat4_rotation_x<float>(-angle);
    CHECK(is_identity(rx_pos * rx_neg, 1e-4));
  }

  TEST_CASE("Rodrigues axis-angle rotation: det = 1") {
    Vec3f axis(1, 1, 1);
    float angle = 1.0f;
    Mat4f r = mat4_rotation<float>(axis, angle);
    CHECK(doctest::Approx(r.det()).epsilon(1e-4) == 1.0f);
  }

  TEST_CASE("Rodrigues: identity for angle=0") {
    Vec3f axis(0, 0, 1);
    Mat4f r = mat4_rotation<float>(axis, 0.0f);
    CHECK(is_identity(r, 1e-5));
  }

  TEST_CASE("Rodrigues: RotZ(pi/2) maps +X to +Y") {
    Vec3f axis(0, 0, 1);
    Mat4f r = mat4_rotation<float>(axis, (float)M_PI / 2.0f);
    Vec4f x_axis(1, 0, 0, 0);
    Vec4f result = r * x_axis;
    CHECK(doctest::Approx(result[0]).epsilon(1e-4) == 0.0f);
    CHECK(doctest::Approx(result[1]).epsilon(1e-4) == 1.0f);
    CHECK(doctest::Approx(result[2]).epsilon(1e-4) == 0.0f);
  }
}

// ===== SIMD correctness =====================================================

TEST_SUITE("Mat - SIMD 4x4 float correctness") {
  TEST_CASE("4x4 * 4x4 full matrix: known product") {
    // Use a simple case: A = 2*I, B = 3*I → A*B = 6*I
    Mat4f a = Mat4f::identity() * 2.0f;
    Mat4f b = Mat4f::identity() * 3.0f;
    Mat4f c = a * b;
    for(unsigned int i = 0; i < 4; i++) {
      for(unsigned int j = 0; j < 4; j++) {
        float expected = (i == j) ? 6.0f : 0.0f;
        CHECK(doctest::Approx(c(i, j)) == expected);
      }
    }
  }

  TEST_CASE("4x4 * 4x4 non-trivial product") {
    // Build matrices explicitly and verify one element
    Mat4f a;
    a(0, 0) = 1; a(0, 1) = 2; a(0, 2) = 3; a(0, 3) = 4;
    a(1, 0) = 0; a(1, 1) = 1; a(1, 2) = 0; a(1, 3) = 1;
    a(2, 0) = 1; a(2, 1) = 0; a(2, 2) = 1; a(2, 3) = 0;
    a(3, 0) = 0; a(3, 1) = 0; a(3, 2) = 0; a(3, 3) = 1;

    Mat4f b = Mat4f::identity();
    b(0, 3) = 5; b(1, 3) = 6; b(2, 3) = 7; b(3, 3) = 1;

    Mat4f c = a * b;
    // row0 of c: [1 2 3 (4+5*1+2*6+3*7+4*1)] = [1 2 3 ?]
    // c(0,0) = a_row0 · b_col0 = 1*1+2*0+3*0+4*0 = 1
    CHECK(doctest::Approx(c(0, 0)) == 1.0f);
    // c(0,3) = a_row0 · b_col3 = 1*5+2*6+3*7+4*1 = 5+12+21+4 = 42
    CHECK(doctest::Approx(c(0, 3)) == 42.0f);
    // c(1,3) = a_row1 · b_col3 = 0*5+1*6+0*7+1*1 = 7
    CHECK(doctest::Approx(c(1, 3)) == 7.0f);
  }

  TEST_CASE("SIMD: (A*B)*C == A*(B*C) (4x4 random-ish)") {
    auto a = make_mat<4, 4, float>({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 1, 2, 3, 14});
    auto b = make_mat<4, 4, float>({2, 0, 1, 0, 0, 3, 0, 1, 1, 0, 2, 0, 0, 1, 0, 3});
    auto c = make_mat<4, 4, float>({1, 0, 0, 1, 0, 2, 0, 0, 0, 0, 3, 0, 1, 0, 0, 4});
    CHECK(mat_approx_eq((a * b) * c, a * (b * c), 1e-3f));
  }

  TEST_CASE("4x4 * vec4: identity leaves vector unchanged") {
    Vec4f v(3, 1, 4, 1);
    Vec4f r = Mat4f::identity() * v;
    CHECK(r == v);
  }

  TEST_CASE("4x4 * vec4: known result") {
    Mat4f m;
    m(0, 0) = 1; m(0, 1) = 0; m(0, 2) = 0; m(0, 3) = 5;
    m(1, 0) = 0; m(1, 1) = 1; m(1, 2) = 0; m(1, 3) = 0;
    m(2, 0) = 0; m(2, 1) = 0; m(2, 2) = 1; m(2, 3) = 0;
    m(3, 0) = 0; m(3, 1) = 0; m(3, 2) = 0; m(3, 3) = 1;
    Vec4f v(1, 2, 3, 1);
    Vec4f r = m * v;
    CHECK(doctest::Approx(r[0]) == 6.0f); // 1 + 5*1
    CHECK(doctest::Approx(r[1]) == 2.0f);
    CHECK(doctest::Approx(r[2]) == 3.0f);
    CHECK(doctest::Approx(r[3]) == 1.0f);
  }
}

// ===== Perspective / Ortho ==================================================

TEST_SUITE("Mat - Projection Matrices") {
  TEST_CASE("Perspective: (0,0,-near,1) maps to NDC z=-1") {
    // Standard OpenGL: near plane maps to z=-1 in clip space
    double near = 1.0, far = 100.0;
    Mat4f p = mat4_perspective<float>(M_PI / 4.0, 1.0, near, far);
    Vec4f pt(0, 0, -(float)near, 1);
    Vec4f clip = p * pt;
    // z_clip/w_clip should be -1 for near
    float z_ndc = clip[2] / clip[3];
    CHECK(doctest::Approx(z_ndc).epsilon(1e-4) == -1.0f);
  }

  TEST_CASE("Perspective: (0,0,-far,1) maps to NDC z=+1") {
    double near = 1.0, far = 100.0;
    Mat4f p = mat4_perspective<float>(M_PI / 4.0, 1.0, near, far);
    Vec4f pt(0, 0, -(float)far, 1);
    Vec4f clip = p * pt;
    float z_ndc = clip[2] / clip[3];
    CHECK(doctest::Approx(z_ndc).epsilon(1e-4) == 1.0f);
  }

  TEST_CASE("Ortho: det != 0") {
    Mat4f o = mat4_ortho<float>(-5, 5, -5, 5, 0.1, 100);
    CHECK(std::abs(o.det()) > 1e-6f);
  }

  TEST_CASE("Ortho: (0,0,-near,1) maps to NDC z=-1") {
    float near = 0.1f, far = 100.0f;
    Mat4f o = mat4_ortho<float>(-1, 1, -1, 1, near, far);
    Vec4f pt(0, 0, -near, 1);
    Vec4f clip = o * pt;
    // Orthographic: no perspective divide needed (w stays 1)
    CHECK(doctest::Approx(clip[2]).epsilon(1e-4) == -1.0f);
  }

  TEST_CASE("Look-at: forward direction is in -Z column") {
    Vec3f eye(0, 0, 5), center(0, 0, 0), up(0, 1, 0);
    Mat4f m = mat4_look_at<float>(eye, center, up);
    // The camera looks along -Z in view space; the third column (row 2) of
    // the rotation part should encode the -forward direction
    Vec4f forward(0, 0, -1, 0); // -Z in world
    Vec4f in_view = m * forward;
    // In view space, -Z world should map to +Z view (OpenGL convention z=-fwd)
    CHECK(doctest::Approx(std::abs(in_view[2])).epsilon(1e-4) == 1.0f);
  }
}

// ===== Comparison ===========================================================

TEST_SUITE("Mat - Comparison") {
  TEST_CASE("== same content") {
    Mat3f a = Mat3f::identity();
    Mat3f b = Mat3f::identity();
    CHECK(a == b);
  }

  TEST_CASE("!= different content") {
    Mat3f a = Mat3f::identity();
    Mat3f b = Mat3f::zeros();
    CHECK(a != b);
  }
}

// ===== Static info ==========================================================

TEST_SUITE("Mat - Static info") {
  TEST_CASE("rows() / cols() / size()") {
    CHECK(Mat3f::rows() == 3);
    CHECK(Mat3f::cols() == 3);
    CHECK(Mat3f::size() == 9);
    CHECK(Mat4f::rows() == 4);
    CHECK(Mat4f::cols() == 4);
    CHECK(Mat4f::size() == 16);
    CHECK((Mat<2, 5, float>::rows()) == 2);
    CHECK((Mat<2, 5, float>::cols()) == 5);
    CHECK((Mat<2, 5, float>::size()) == 10);
  }
}
