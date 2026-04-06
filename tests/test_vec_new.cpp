#include <cmath>
#include <cutil/vec.hpp>
#include <doctest.h>

using namespace Cutil;

TEST_SUITE("NVec - Constructors") {
  TEST_CASE("Default constructor") {
    Vec3f v;
    CHECK(v[0] == 0.0f);
    CHECK(v[1] == 0.0f);
    CHECK(v[2] == 0.0f);
  }

  TEST_CASE("Scalar fill constructor") {
    Vec3f v(5.0f);
    CHECK(v[0] == 5.0f);
    CHECK(v[1] == 5.0f);
    CHECK(v[2] == 5.0f);
  }

  TEST_CASE("2D constructor") {
    Vec2f v(1.0f, 2.0f);
    CHECK(v[0] == 1.0f);
    CHECK(v[1] == 2.0f);
  }

  TEST_CASE("3D constructor") {
    Vec3f v(1.0f, 2.0f, 3.0f);
    CHECK(v[0] == 1.0f);
    CHECK(v[1] == 2.0f);
    CHECK(v[2] == 3.0f);
  }

  TEST_CASE("4D constructor") {
    Vec4f v(1.0f, 2.0f, 3.0f, 4.0f);
    CHECK(v[0] == 1.0f);
    CHECK(v[1] == 2.0f);
    CHECK(v[2] == 3.0f);
    CHECK(v[3] == 4.0f);
  }

  TEST_CASE("Copy constructor") {
    Vec3f v1(1.0f, 2.0f, 3.0f);
    Vec3f v2 = v1;
    CHECK(v2[0] == 1.0f);
    CHECK(v2[1] == 2.0f);
    CHECK(v2[2] == 3.0f);
  }

  TEST_CASE("Type conversion constructor") {
    Vec3d vd(1.0, 2.0, 3.0);
    Vec3f vf(vd);
    CHECK(doctest::Approx(vf[0]) == 1.0f);
    CHECK(doctest::Approx(vf[1]) == 2.0f);
    CHECK(doctest::Approx(vf[2]) == 3.0f);
  }

  TEST_CASE("Extend constructor") {
    Vec2f v2(1.0f, 2.0f);
    Vec3f v3(v2, 3.0f);
    CHECK(v3[0] == 1.0f);
    CHECK(v3[1] == 2.0f);
    CHECK(v3[2] == 3.0f);
  }
}

TEST_SUITE("NVec - Named Accessors") {
  TEST_CASE("x, y, z accessors") {
    Vec3f v(1.0f, 2.0f, 3.0f);
    CHECK(v.x() == 1.0f);
    CHECK(v.y() == 2.0f);
    CHECK(v.z() == 3.0f);
  }

  TEST_CASE("w accessor") {
    Vec4f v(1.0f, 2.0f, 3.0f, 4.0f);
    CHECK(v.w() == 4.0f);
  }

  TEST_CASE("xy() swizzle") {
    Vec3f v(1.0f, 2.0f, 3.0f);
    Vec2f v2 = v.xy();
    CHECK(v2[0] == 1.0f);
    CHECK(v2[1] == 2.0f);
  }

  TEST_CASE("xyz() swizzle") {
    Vec4f v(1.0f, 2.0f, 3.0f, 4.0f);
    Vec3f v3 = v.xyz();
    CHECK(v3[0] == 1.0f);
    CHECK(v3[1] == 2.0f);
    CHECK(v3[2] == 3.0f);
  }
}

TEST_SUITE("NVec - Arithmetic Operations") {
  TEST_CASE("Vector addition") {
    Vec3f v1(1.0f, 2.0f, 3.0f);
    Vec3f v2(4.0f, 5.0f, 6.0f);
    Vec3f v3 = v1 + v2;
    CHECK(v3[0] == 5.0f);
    CHECK(v3[1] == 7.0f);
    CHECK(v3[2] == 9.0f);
  }

  TEST_CASE("Vector subtraction") {
    Vec3f v1(5.0f, 7.0f, 9.0f);
    Vec3f v2(1.0f, 2.0f, 3.0f);
    Vec3f v3 = v1 - v2;
    CHECK(v3[0] == 4.0f);
    CHECK(v3[1] == 5.0f);
    CHECK(v3[2] == 6.0f);
  }

  TEST_CASE("Component-wise multiplication") {
    Vec3f v1(2.0f, 3.0f, 4.0f);
    Vec3f v2(5.0f, 6.0f, 7.0f);
    Vec3f v3 = v1 * v2;
    CHECK(v3[0] == 10.0f);
    CHECK(v3[1] == 18.0f);
    CHECK(v3[2] == 28.0f);
  }

  TEST_CASE("Component-wise division") {
    Vec3f v1(10.0f, 20.0f, 30.0f);
    Vec3f v2(2.0f, 5.0f, 10.0f);
    Vec3f v3 = v1 / v2;
    CHECK(doctest::Approx(v3[0]) == 5.0f);
    CHECK(doctest::Approx(v3[1]) == 4.0f);
    CHECK(doctest::Approx(v3[2]) == 3.0f);
  }

  TEST_CASE("Scalar multiplication") {
    Vec3f v(2.0f, 3.0f, 4.0f);
    Vec3f v2 = v * 3.0f;
    CHECK(v2[0] == 6.0f);
    CHECK(v2[1] == 9.0f);
    CHECK(v2[2] == 12.0f);
  }

  TEST_CASE("Scalar multiplication (scalar * vector)") {
    Vec3f v(2.0f, 3.0f, 4.0f);
    Vec3f v2 = 3.0f * v;
    CHECK(v2[0] == 6.0f);
    CHECK(v2[1] == 9.0f);
    CHECK(v2[2] == 12.0f);
  }

  TEST_CASE("Scalar division") {
    Vec3f v(6.0f, 9.0f, 12.0f);
    Vec3f v2 = v / 3.0f;
    CHECK(doctest::Approx(v2[0]) == 2.0f);
    CHECK(doctest::Approx(v2[1]) == 3.0f);
    CHECK(doctest::Approx(v2[2]) == 4.0f);
  }

  TEST_CASE("Scalar addition") {
    Vec3f v(1.0f, 2.0f, 3.0f);
    Vec3f v2 = v + 5.0f;
    CHECK(v2[0] == 6.0f);
    CHECK(v2[1] == 7.0f);
    CHECK(v2[2] == 8.0f);
  }

  TEST_CASE("Negation") {
    Vec3f v(1.0f, -2.0f, 3.0f);
    Vec3f v2 = -v;
    CHECK(v2[0] == -1.0f);
    CHECK(v2[1] == 2.0f);
    CHECK(v2[2] == -3.0f);
  }

  TEST_CASE("Compound assignment +=") {
    Vec3f v1(1.0f, 2.0f, 3.0f);
    Vec3f v2(4.0f, 5.0f, 6.0f);
    v1 += v2;
    CHECK(v1[0] == 5.0f);
    CHECK(v1[1] == 7.0f);
    CHECK(v1[2] == 9.0f);
  }

  TEST_CASE("Compound assignment -=") {
    Vec3f v1(5.0f, 7.0f, 9.0f);
    Vec3f v2(1.0f, 2.0f, 3.0f);
    v1 -= v2;
    CHECK(v1[0] == 4.0f);
    CHECK(v1[1] == 5.0f);
    CHECK(v1[2] == 6.0f);
  }

  TEST_CASE("Compound assignment *=") {
    Vec3f v(2.0f, 3.0f, 4.0f);
    v *= 3.0f;
    CHECK(v[0] == 6.0f);
    CHECK(v[1] == 9.0f);
    CHECK(v[2] == 12.0f);
  }

  TEST_CASE("Compound assignment /=") {
    Vec3f v(6.0f, 9.0f, 12.0f);
    v /= 3.0f;
    CHECK(doctest::Approx(v[0]) == 2.0f);
    CHECK(doctest::Approx(v[1]) == 3.0f);
    CHECK(doctest::Approx(v[2]) == 4.0f);
  }
}

TEST_SUITE("NVec - Dot Product & Length") {
  TEST_CASE("Dot product") {
    Vec3f v1(1.0f, 2.0f, 3.0f);
    Vec3f v2(4.0f, 5.0f, 6.0f);
    float d = v1.dot(v2);
    CHECK(d == doctest::Approx(32.0f)); // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
  }

  TEST_CASE("Length squared") {
    Vec3f v(3.0f, 4.0f, 0.0f);
    float len_sq = v.length_squared();
    CHECK(len_sq == doctest::Approx(25.0f));
  }

  TEST_CASE("Length (magnitude)") {
    Vec3f v(3.0f, 4.0f, 0.0f);
    double len = v.length();
    CHECK(len == doctest::Approx(5.0));
  }

  TEST_CASE("Normalization") {
    Vec3f v(3.0f, 4.0f, 0.0f);
    Vec3f vn = v.normalized();
    CHECK(vn[0] == doctest::Approx(0.6f));
    CHECK(vn[1] == doctest::Approx(0.8f));
    CHECK(vn[2] == doctest::Approx(0.0f));
    CHECK(vn.length() == doctest::Approx(1.0));
  }

  TEST_CASE("Free function: dot") {
    Vec3f v1(1.0f, 2.0f, 3.0f);
    Vec3f v2(4.0f, 5.0f, 6.0f);
    float d = dot(v1, v2);
    CHECK(d == doctest::Approx(32.0f));
  }

  TEST_CASE("Free function: length") {
    Vec3f v(3.0f, 4.0f, 0.0f);
    double len = length(v);
    CHECK(len == doctest::Approx(5.0));
  }

  TEST_CASE("Free function: normalize") {
    Vec3f v(3.0f, 4.0f, 0.0f);
    Vec3f vn = normalize(v);
    CHECK(vn[0] == doctest::Approx(0.6f));
    CHECK(vn[1] == doctest::Approx(0.8f));
  }
}

TEST_SUITE("NVec - Cross Product") {
  TEST_CASE("Cross product") {
    Vec3f v1(1.0f, 0.0f, 0.0f);
    Vec3f v2(0.0f, 1.0f, 0.0f);
    Vec3f v3 = v1.cross(v2);
    CHECK(v3[0] == doctest::Approx(0.0f));
    CHECK(v3[1] == doctest::Approx(0.0f));
    CHECK(v3[2] == doctest::Approx(1.0f));
  }

  TEST_CASE("Cross product anti-commutativity") {
    Vec3f v1(1.0f, 0.0f, 0.0f);
    Vec3f v2(0.0f, 1.0f, 0.0f);
    Vec3f c1 = v1.cross(v2);
    Vec3f c2 = v2.cross(v1);
    CHECK(c1[2] == doctest::Approx(-c2[2]));
  }

  TEST_CASE("Free function: cross") {
    Vec3f v1(1.0f, 0.0f, 0.0f);
    Vec3f v2(0.0f, 1.0f, 0.0f);
    Vec3f v3 = cross(v1, v2);
    CHECK(v3[2] == doctest::Approx(1.0f));
  }
}

TEST_SUITE("NVec - Godot-style Functions") {
  TEST_CASE("angle_to") {
    Vec3f v1(1.0f, 0.0f, 0.0f);
    Vec3f v2(0.0f, 1.0f, 0.0f);
    double angle = v1.angle_to(v2);
    CHECK(angle == doctest::Approx(M_PI / 2.0));
  }

  TEST_CASE("distance_to") {
    Vec3f v1(1.0f, 2.0f, 3.0f);
    Vec3f v2(4.0f, 6.0f, 8.0f);
    double dist = v1.distance_to(v2);
    CHECK(dist == doctest::Approx(std::sqrt(9.0 + 16.0 + 25.0)));
  }

  TEST_CASE("lerp (linear interpolation)") {
    Vec3f v1(0.0f, 0.0f, 0.0f);
    Vec3f v2(10.0f, 20.0f, 30.0f);
    Vec3f v = v1.lerp(v2, 0.5);
    CHECK(v[0] == doctest::Approx(5.0f));
    CHECK(v[1] == doctest::Approx(10.0f));
    CHECK(v[2] == doctest::Approx(15.0f));
  }

  TEST_CASE("reflect") {
    Vec3f incident(1.0f, -1.0f, 0.0f);
    Vec3f normal(0.0f, 1.0f, 0.0f); // normal pointing up
    Vec3f reflected = incident.reflect(normal);
    CHECK(reflected[0] == doctest::Approx(1.0f));
    CHECK(reflected[1] == doctest::Approx(1.0f));
    CHECK(reflected[2] == doctest::Approx(0.0f));
  }

  TEST_CASE("slide") {
    Vec3f v(1.0f, 1.0f, 0.0f);
    Vec3f normal(0.0f, 1.0f, 0.0f);
    Vec3f result = v.slide(normal);
    CHECK(result[0] == doctest::Approx(1.0f));
    CHECK(result[1] == doctest::Approx(0.0f));
    CHECK(result[2] == doctest::Approx(0.0f));
  }

  TEST_CASE("project") {
    Vec3f v(1.0f, 1.0f, 0.0f);
    Vec3f b(1.0f, 0.0f, 0.0f);
    Vec3f result = v.project(b);
    CHECK(result[0] == doctest::Approx(1.0f));
    CHECK(result[1] == doctest::Approx(0.0f));
    CHECK(result[2] == doctest::Approx(0.0f));
  }
}

TEST_SUITE("NVec - Per-element Operations") {
  TEST_CASE("abs") {
    Vec3f v(-1.0f, 2.0f, -3.0f);
    Vec3f v_abs = v.abs();
    CHECK(v_abs[0] == 1.0f);
    CHECK(v_abs[1] == 2.0f);
    CHECK(v_abs[2] == 3.0f);
  }

  TEST_CASE("clamp") {
    Vec3f v(0.5f, 1.5f, 2.5f);
    Vec3f v_clamped = v.clamp(1.0f, 2.0f);
    CHECK(v_clamped[0] == 1.0f);
    CHECK(v_clamped[1] == doctest::Approx(1.5f));
    CHECK(v_clamped[2] == 2.0f);
  }

  TEST_CASE("min_with") {
    Vec3f v1(1.0f, 5.0f, 3.0f);
    Vec3f v2(2.0f, 3.0f, 4.0f);
    Vec3f v = v1.min_with(v2);
    CHECK(v[0] == 1.0f);
    CHECK(v[1] == 3.0f);
    CHECK(v[2] == 3.0f);
  }

  TEST_CASE("max_with") {
    Vec3f v1(1.0f, 5.0f, 3.0f);
    Vec3f v2(2.0f, 3.0f, 4.0f);
    Vec3f v = v1.max_with(v2);
    CHECK(v[0] == 2.0f);
    CHECK(v[1] == 5.0f);
    CHECK(v[2] == 4.0f);
  }

  TEST_CASE("sum") {
    Vec3f v(1.0f, 2.0f, 3.0f);
    float s = v.sum();
    CHECK(s == 6.0f);
  }

  TEST_CASE("max_component") {
    Vec3f v(1.0f, 5.0f, 3.0f);
    float m = v.max_component();
    CHECK(m == 5.0f);
  }

  TEST_CASE("min_component") {
    Vec3f v(1.0f, 5.0f, 3.0f);
    float m = v.min_component();
    CHECK(m == 1.0f);
  }
}

TEST_SUITE("NVec - Utility Functions") {
  TEST_CASE("zero()") {
    Vec3f v = Vec3f::zero();
    CHECK(v[0] == 0.0f);
    CHECK(v[1] == 0.0f);
    CHECK(v[2] == 0.0f);
  }

  TEST_CASE("one()") {
    Vec3f v = Vec3f::one();
    CHECK(v[0] == 1.0f);
    CHECK(v[1] == 1.0f);
    CHECK(v[2] == 1.0f);
  }

  TEST_CASE("axis()") {
    Vec3f x = Vec3f::axis(0);
    Vec3f y = Vec3f::axis(1);
    Vec3f z = Vec3f::axis(2);
    CHECK(x[0] == 1.0f);
    CHECK(x[1] == 0.0f);
    CHECK(y[1] == 1.0f);
    CHECK(z[2] == 1.0f);
  }

  TEST_CASE("is_zero_approx") {
    Vec3f v(1e-7f, 1e-7f, 1e-7f);
    CHECK(v.is_zero_approx(1e-5f));
  }

  TEST_CASE("size()") {
    Vec3f v;
    CHECK(v.size() == 3);
    Vec4f v4;
    CHECK(v4.size() == 4);
  }
}

TEST_SUITE("NVec - Comparison") {
  TEST_CASE("Equality") {
    Vec3f v1(1.0f, 2.0f, 3.0f);
    Vec3f v2(1.0f, 2.0f, 3.0f);
    CHECK(v1 == v2);
  }

  TEST_CASE("Inequality") {
    Vec3f v1(1.0f, 2.0f, 3.0f);
    Vec3f v2(1.0f, 2.0f, 4.0f);
    CHECK(v1 != v2);
  }
}
