#include <cutil/mat.hpp>
#include <doctest.h>
#include <cmath>

using namespace Cutil;

TEST_SUITE("Mat - Constructors") {
  TEST_CASE("Default constructor") {
    Mat3f m;
    for(int i = 0; i < 9; i++) {
      CHECK(m[i] == 0.0f);
    }
  }

  TEST_CASE("Copy constructor") {
    Mat3f m1;
    m1(0, 0) = 1.0f;
    Mat3f m2 = m1;
    CHECK(m2(0, 0) == 1.0f);
  }

  TEST_CASE("Assignment operator") {
    Mat3f m1;
    m1(0, 0) = 5.0f;
    Mat3f m2;
    m2 = m1;
    CHECK(m2(0, 0) == 5.0f);
  }

  TEST_CASE("Identity matrix") {
    Mat3f m = Mat3f::identity();
    CHECK(m(0, 0) == 1.0f);
    CHECK(m(1, 1) == 1.0f);
    CHECK(m(2, 2) == 1.0f);
    CHECK(m(0, 1) == 0.0f);
    CHECK(m(1, 0) == 0.0f);
  }

  TEST_CASE("Zeros matrix") {
    Mat3f m = Mat3f::zeros();
    for(int i = 0; i < 9; i++) {
      CHECK(m[i] == 0.0f);
    }
  }

  TEST_CASE("Ones matrix") {
    Mat3f m = Mat3f::ones();
    for(int i = 0; i < 9; i++) {
      CHECK(m[i] == 1.0f);
    }
  }
}

TEST_SUITE("Mat - Element Access") {
  TEST_CASE("2D element access (row, col)") {
    Mat3f m;
    m(0, 0) = 1.0f;
    m(1, 2) = 2.0f;
    CHECK(m(0, 0) == 1.0f);
    CHECK(m(1, 2) == 2.0f);
  }

  TEST_CASE("Flat array access") {
    Mat3f m;
    m[0] = 1.0f;
    m[8] = 9.0f;
    CHECK(m[0] == 1.0f);
    CHECK(m[8] == 9.0f);
  }

  TEST_CASE("Column access") {
    Mat3f m = Mat3f::identity();
    Vec3f col0 = m.col(0);
    CHECK(col0[0] == 1.0f);
    CHECK(col0[1] == 0.0f);
    CHECK(col0[2] == 0.0f);
  }

  TEST_CASE("Row access") {
    Mat3f m = Mat3f::identity();
    Vec3f row0 = m.row(0);
    CHECK(row0[0] == 1.0f);
    CHECK(row0[1] == 0.0f);
    CHECK(row0[2] == 0.0f);
  }

  TEST_CASE("Set column") {
    Mat3f m = Mat3f::zeros();
    m.set_col(0, Vec3f(1.0f, 2.0f, 3.0f));
    CHECK(m(0, 0) == 1.0f);
    CHECK(m(1, 0) == 2.0f);
    CHECK(m(2, 0) == 3.0f);
  }

  TEST_CASE("Set row") {
    Mat3f m = Mat3f::zeros();
    m.set_row(0, Vec3f(1.0f, 2.0f, 3.0f));
    CHECK(m(0, 0) == 1.0f);
    CHECK(m(0, 1) == 2.0f);
    CHECK(m(0, 2) == 3.0f);
  }
}

TEST_SUITE("Mat - Arithmetic Operations") {
  TEST_CASE("Matrix addition") {
    Mat3f m1 = Mat3f::identity();
    Mat3f m2 = Mat3f::identity();
    Mat3f m3 = m1 + m2;
    CHECK(m3(0, 0) == 2.0f);
    CHECK(m3(1, 1) == 2.0f);
    CHECK(m3(0, 1) == 0.0f);
  }

  TEST_CASE("Matrix subtraction") {
    Mat3f m1;
    m1(0, 0) = 5.0f;
    Mat3f m2;
    m2(0, 0) = 3.0f;
    Mat3f m3 = m1 - m2;
    CHECK(m3(0, 0) == 2.0f);
  }

  TEST_CASE("Scalar multiplication") {
    Mat3f m;
    m(0, 0) = 2.0f;
    Mat3f m2 = m * 3.0f;
    CHECK(m2(0, 0) == 6.0f);
  }

  TEST_CASE("Scalar multiplication (scalar * matrix)") {
    Mat3f m;
    m(0, 0) = 2.0f;
    Mat3f m2 = 3.0f * m;
    CHECK(m2(0, 0) == 6.0f);
  }

  TEST_CASE("Scalar division") {
    Mat3f m;
    m(0, 0) = 6.0f;
    Mat3f m2 = m / 3.0f;
    CHECK(doctest::Approx(m2(0, 0)) == 2.0f);
  }

  TEST_CASE("Negation") {
    Mat3f m;
    m(0, 0) = 5.0f;
    Mat3f m2 = -m;
    CHECK(m2(0, 0) == -5.0f);
  }

  TEST_CASE("Compound assignment +=") {
    Mat3f m1 = Mat3f::identity();
    Mat3f m2 = Mat3f::identity();
    m1 += m2;
    CHECK(m1(0, 0) == 2.0f);
    CHECK(m1(1, 1) == 2.0f);
  }

  TEST_CASE("Compound assignment -=") {
    Mat3f m1;
    m1(0, 0) = 5.0f;
    Mat3f m2;
    m2(0, 0) = 3.0f;
    m1 -= m2;
    CHECK(m1(0, 0) == 2.0f);
  }

  TEST_CASE("Compound assignment *=") {
    Mat3f m;
    m(0, 0) = 2.0f;
    m *= 3.0f;
    CHECK(m(0, 0) == 6.0f);
  }
}

TEST_SUITE("Mat - Matrix Multiplication") {
  TEST_CASE("3x3 * 3x3 matrix multiplication") {
    Mat3f m1;
    m1(0, 0) = 1.0f;
    m1(0, 1) = 2.0f;
    m1(1, 0) = 3.0f;
    m1(1, 1) = 4.0f;

    Mat3f m2;
    m2(0, 0) = 5.0f;
    m2(0, 1) = 6.0f;
    m2(1, 0) = 7.0f;
    m2(1, 1) = 8.0f;

    Mat3f result = m1 * m2;
    // result(0, 0) = m1(0, 0)*m2(0, 0) + m1(0, 1)*m2(1, 0) = 1*5 + 2*7 = 19
    CHECK(doctest::Approx(result(0, 0)) == 19.0f);
  }

  TEST_CASE("Identity matrix multiplication") {
    Mat3f m = Mat3f::identity();
    Mat3f result = m * m;
    CHECK(result == m);
  }

  TEST_CASE("4x4 * 4x4 matrix multiplication") {
    Mat4f m1 = Mat4f::identity();
    m1(0, 3) = 5.0f;  // Translation in x
    Mat4f m2 = Mat4f::identity();
    Mat4f result = m1 * m2;
    CHECK(doctest::Approx(result(0, 3)) == 5.0f);
  }
}

TEST_SUITE("Mat - Matrix-Vector Multiplication") {
  TEST_CASE("3x3 matrix * 3D vector") {
    Mat3f m = Mat3f::identity();
    Vec3f v(1.0f, 2.0f, 3.0f);
    Vec3f result = m * v;
    CHECK(result[0] == 1.0f);
    CHECK(result[1] == 2.0f);
    CHECK(result[2] == 3.0f);
  }

  TEST_CASE("Scaled matrix * vector") {
    Mat3f m = Mat3f::identity();
    m(0, 0) = 2.0f;
    m(1, 1) = 3.0f;
    m(2, 2) = 4.0f;
    Vec3f v(1.0f, 1.0f, 1.0f);
    Vec3f result = m * v;
    CHECK(result[0] == 2.0f);
    CHECK(result[1] == 3.0f);
    CHECK(result[2] == 4.0f);
  }

  TEST_CASE("4x4 matrix * 4D vector") {
    Mat4f m = Mat4f::identity();
    Vec4f v(1.0f, 2.0f, 3.0f, 1.0f);
    Vec4f result = m * v;
    CHECK(result[0] == 1.0f);
    CHECK(result[1] == 2.0f);
    CHECK(result[2] == 3.0f);
    CHECK(result[3] == 1.0f);
  }
}

TEST_SUITE("Mat - Transpose") {
  TEST_CASE("Matrix transpose 3x3") {
    Mat3f m;
    m(0, 0) = 1.0f;
    m(0, 1) = 2.0f;
    m(1, 0) = 3.0f;
    Mat3f mt = m.transposed();
    CHECK(mt(0, 0) == 1.0f);
    CHECK(mt(0, 1) == 3.0f);
    CHECK(mt(1, 0) == 2.0f);
  }

  TEST_CASE("Transpose of transpose is identity") {
    Mat3f m;
    m(0, 1) = 5.0f;
    Mat3f mt = m.transposed().transposed();
    CHECK(mt == m);
  }

  TEST_CASE("In-place transpose (square matrix)") {
    Mat3f m;
    m(0, 1) = 5.0f;
    m(1, 0) = 3.0f;
    m.transpose();
    CHECK(m(0, 1) == 3.0f);
    CHECK(m(1, 0) == 5.0f);
  }
}

TEST_SUITE("Mat - Determinant") {
  TEST_CASE("Identity matrix determinant") {
    Mat3f m = Mat3f::identity();
    float det = m.det();
    CHECK(doctest::Approx(det) == 1.0f);
  }

  TEST_CASE("Singular matrix determinant") {
    Mat3f m;
    m(0, 0) = 1.0f;
    m(0, 1) = 2.0f;
    m(1, 0) = 2.0f;
    m(1, 1) = 4.0f;
    float det = m.det();
    CHECK(doctest::Approx(det) == 0.0f);
  }

  TEST_CASE("2x2 matrix determinant") {
    Mat<2, 2, float> m;
    m(0, 0) = 1.0f;
    m(0, 1) = 2.0f;
    m(1, 0) = 3.0f;
    m(1, 1) = 4.0f;
    float det = m.det();
    // det = 1*4 - 2*3 = 4 - 6 = -2
    CHECK(doctest::Approx(det) == -2.0f);
  }
}

TEST_SUITE("Mat - Inverse") {
  TEST_CASE("Identity matrix inverse") {
    Mat3f m = Mat3f::identity();
    Mat3f m_inv = m.inverse();
    CHECK(m_inv == m);
  }

  TEST_CASE("Matrix * Inverse = Identity") {
    Mat3f m;
    m(0, 0) = 2.0f;
    m(1, 1) = 3.0f;
    m(2, 2) = 4.0f;
    Mat3f m_inv = m.inverse();
    Mat3f result = m * m_inv;
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        float expected = (i == j) ? 1.0f : 0.0f;
        CHECK(doctest::Approx(result(i, j)) == expected);
      }
    }
  }
}

TEST_SUITE("Mat - Square Matrix Functions") {
  TEST_CASE("Trace of identity") {
    Mat3f m = Mat3f::identity();
    float tr = m.trace();
    CHECK(tr == 3.0f);
  }

  TEST_CASE("Frobenius norm") {
    Mat3f m = Mat3f::identity();
    double norm = m.frobenius_norm();
    // Norm of identity = sqrt(1 + 1 + 1) = sqrt(3)
    CHECK(norm == doctest::Approx(std::sqrt(3.0)));
  }
}

TEST_SUITE("Mat - Godot-style 4x4 Transform Functions") {
  TEST_CASE("Translation matrix") {
    Mat4f m = mat4_translation(Vec3f(5.0f, 10.0f, 15.0f));
    CHECK(m(0, 3) == 5.0f);
    CHECK(m(1, 3) == 10.0f);
    CHECK(m(2, 3) == 15.0f);
  }

  TEST_CASE("Scale matrix") {
    Mat4f m = mat4_scale(Vec3f(2.0f, 3.0f, 4.0f));
    CHECK(m(0, 0) == 2.0f);
    CHECK(m(1, 1) == 3.0f);
    CHECK(m(2, 2) == 4.0f);
  }

  TEST_CASE("Rotation X") {
    Mat4f m = mat4_rotation_x<float>(0.0f);
    CHECK(doctest::Approx(m(1, 1)) == 1.0f);
    CHECK(doctest::Approx(m(2, 2)) == 1.0f);
  }

  TEST_CASE("Rotation Y") {
    Mat4f m = mat4_rotation_y<float>(0.0f);
    CHECK(doctest::Approx(m(0, 0)) == 1.0f);
    CHECK(doctest::Approx(m(2, 2)) == 1.0f);
  }

  TEST_CASE("Rotation Z") {
    Mat4f m = mat4_rotation_z<float>(0.0f);
    CHECK(doctest::Approx(m(0, 0)) == 1.0f);
    CHECK(doctest::Approx(m(1, 1)) == 1.0f);
  }

  TEST_CASE("Look-at matrix creates orthogonal basis") {
    Mat4f m = mat4_look_at<float>(Vec3f(0.0f, 0.0f, 5.0f), Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 1.0f, 0.0f));
    // Basis vectors should be unit vectors
    Vec3f col0(m(0, 0), m(1, 0), m(2, 0));
    Vec3f col1(m(0, 1), m(1, 1), m(2, 1));
    double len0 = col0.length();
    double len1 = col1.length();
    CHECK(len0 == doctest::Approx(1.0));
    CHECK(len1 == doctest::Approx(1.0));
  }

  TEST_CASE("Perspective projection") {
    Mat4f m = mat4_perspective<float>(M_PI / 4.0, 16.0 / 9.0, 0.1, 100.0);
    // Check that it's not all zeros
    CHECK(m(1, 1) != 0.0f);
    CHECK(m(2, 2) != 0.0f);
  }

  TEST_CASE("Orthographic projection") {
    Mat4f m = mat4_ortho<float>(-1.0, 1.0, -1.0, 1.0, 0.1, 100.0);
    // Check diagonal elements
    CHECK(m(0, 0) != 0.0f);
    CHECK(m(1, 1) != 0.0f);
    CHECK(m(2, 2) != 0.0f);
    CHECK(m(3, 3) == 1.0f);
  }
}

TEST_SUITE("Mat - Comparison") {
  TEST_CASE("Equality") {
    Mat3f m1 = Mat3f::identity();
    Mat3f m2 = Mat3f::identity();
    CHECK(m1 == m2);
  }

  TEST_CASE("Inequality") {
    Mat3f m1 = Mat3f::identity();
    Mat3f m2 = Mat3f::zeros();
    CHECK(m1 != m2);
  }
}

TEST_SUITE("Mat - SIMD Optimization (4x4 float)") {
  TEST_CASE("4x4 float matrix multiplication (SSE optimized)") {
    Mat4f m1;
    m1(0, 0) = 1.0f;
    m1(1, 1) = 2.0f;
    m1(2, 2) = 3.0f;
    m1(3, 3) = 4.0f;

    Mat4f m2;
    m2(0, 0) = 5.0f;
    m2(1, 1) = 6.0f;
    m2(2, 2) = 7.0f;
    m2(3, 3) = 8.0f;

    Mat4f result = m1 * m2;
    CHECK(doctest::Approx(result(0, 0)) == 5.0f);
    CHECK(doctest::Approx(result(1, 1)) == 12.0f);
    CHECK(doctest::Approx(result(2, 2)) == 21.0f);
    CHECK(doctest::Approx(result(3, 3)) == 32.0f);
  }

  TEST_CASE("4x4 float matrix * vector (SSE optimized)") {
    Mat4f m = Mat4f::identity();
    m(0, 0) = 2.0f;
    m(1, 1) = 3.0f;
    m(2, 2) = 4.0f;
    m(3, 3) = 5.0f;

    Vec4f v(1.0f, 1.0f, 1.0f, 1.0f);
    Vec4f result = m * v;

    CHECK(doctest::Approx(result[0]) == 2.0f);
    CHECK(doctest::Approx(result[1]) == 3.0f);
    CHECK(doctest::Approx(result[2]) == 4.0f);
    CHECK(doctest::Approx(result[3]) == 5.0f);
  }
}

TEST_SUITE("Mat - Static info") {
  TEST_CASE("rows() and cols()") {
    CHECK(Mat3f::rows() == 3);
    CHECK(Mat3f::cols() == 3);
    CHECK(Mat4f::rows() == 4);
    CHECK(Mat4f::cols() == 4);
  }

  TEST_CASE("size()") {
    CHECK(Mat3f::size() == 9);
    CHECK(Mat4f::size() == 16);
  }
}
