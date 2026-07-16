#include <cmath>
#include <cutil/quaternion.hpp>
#include <doctest.h>

using namespace cutil;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

TEST_SUITE("Quat - Constructors") {
  TEST_CASE("Default constructor is identity") {
    Quatd q;
    CHECK(q.x == 0.0);
    CHECK(q.y == 0.0);
    CHECK(q.z == 0.0);
    CHECK(q.w == 1.0);
  }

  TEST_CASE("Component constructor") {
    Quatd q(1.0, 2.0, 3.0, 4.0);
    CHECK(q.x == 1.0);
    CHECK(q.y == 2.0);
    CHECK(q.z == 3.0);
    CHECK(q.w == 4.0);
  }

  TEST_CASE("identity() factory") {
    Quatd q = Quatd::identity();
    CHECK(q == Quatd(0.0, 0.0, 0.0, 1.0));
  }
}

TEST_SUITE("Quat - Norm") {
  TEST_CASE("length and normalized") {
    Quatd q(0.0, 3.0, 4.0, 0.0);
    CHECK(doctest::Approx(q.length()) == 5.0);
    Quatd n = q.normalized();
    CHECK(doctest::Approx(n.length()) == 1.0);
    CHECK(n.is_normalized());
  }
}

TEST_SUITE("Quat - Conjugate and inverse") {
  TEST_CASE("conjugate matches inverse for a unit quaternion") {
    Quatd q = Quatd::from_axis_angle(Vec3d(0.0, 0.0, 1.0), 1.2);
    Quatd c = q.conjugate();
    Quatd i = q.inverse();
    CHECK(doctest::Approx(c.x) == i.x);
    CHECK(doctest::Approx(c.y) == i.y);
    CHECK(doctest::Approx(c.z) == i.z);
    CHECK(doctest::Approx(c.w) == i.w);
  }

  TEST_CASE("q * q.inverse() is identity") {
    Quatd q      = Quatd::from_axis_angle(Vec3d(1.0, 1.0, 0.0), 0.7);
    Quatd result = q * q.inverse();
    CHECK(doctest::Approx(result.w) == 1.0);
    CHECK(doctest::Approx(result.x) == 0.0);
    CHECK(doctest::Approx(result.y) == 0.0);
    CHECK(doctest::Approx(result.z) == 0.0);
  }
}

TEST_SUITE("Quat - Rotation") {
  TEST_CASE("rotating a vector 90 degrees around Z") {
    Quatd q      = Quatd::from_axis_angle(Vec3d(0.0, 0.0, 1.0), kPi / 2.0);
    Vec3d result = q * Vec3d(1.0, 0.0, 0.0);
    CHECK(doctest::Approx(result[0]) == 0.0);
    CHECK(doctest::Approx(result[1]) == 1.0);
    CHECK(doctest::Approx(result[2]) == 0.0);
  }

  TEST_CASE("composed rotations apply right-to-left") {
    Quatd rz = Quatd::from_axis_angle(Vec3d(0.0, 0.0, 1.0), kPi / 2.0);
    Quatd rx = Quatd::from_axis_angle(Vec3d(1.0, 0.0, 0.0), kPi / 2.0);
    Vec3d v(0.0, 1.0, 0.0);
    Vec3d combined = (rx * rz) * v;
    Vec3d stepwise = rx * (rz * v);
    CHECK(doctest::Approx(combined[0]) == stepwise[0]);
    CHECK(doctest::Approx(combined[1]) == stepwise[1]);
    CHECK(doctest::Approx(combined[2]) == stepwise[2]);
  }

  TEST_CASE("axis-angle round trip") {
    Vec3d axis(0.0, 1.0, 0.0);
    double angle = 1.0;
    Quatd q      = Quatd::from_axis_angle(axis, angle);
    CHECK(doctest::Approx(q.get_angle()) == angle);
    Vec3d a = q.get_axis();
    CHECK(doctest::Approx(a[0]) == axis[0]);
    CHECK(doctest::Approx(a[1]) == axis[1]);
    CHECK(doctest::Approx(a[2]) == axis[2]);
  }
}

TEST_SUITE("Quat - Matrix conversion") {
  TEST_CASE("to_mat3 / from_mat3 round trip") {
    Quatd q  = Quatd::from_axis_angle(Vec3d(0.2, 0.4, 0.8), 0.9);
    Mat3d m  = q.to_mat3();
    Quatd q2 = Quat<double>::from_mat3(m);
    Vec3d v(1.0, 2.0, 3.0);
    Vec3d r1 = q * v;
    Vec3d r2 = q2 * v;
    CHECK(doctest::Approx(r1[0]) == r2[0]);
    CHECK(doctest::Approx(r1[1]) == r2[1]);
    CHECK(doctest::Approx(r1[2]) == r2[2]);
  }

  TEST_CASE("to_mat4 embeds the rotation and leaves the last row/column as identity") {
    Quatd q = Quatd::from_axis_angle(Vec3d(0.0, 1.0, 0.0), 0.5);
    Mat4d m = q.to_mat4();
    CHECK(doctest::Approx(m(3, 3)) == 1.0);
    CHECK(doctest::Approx(m(0, 3)) == 0.0);
    CHECK(doctest::Approx(m(3, 0)) == 0.0);
  }
}

TEST_SUITE("Quat - Euler") {
  TEST_CASE("euler round trip") {
    Quatd q  = Quatd::from_euler(Vec3d(0.3, 0.2, 0.5));
    Vec3d e  = q.get_euler();
    Quatd q2 = Quatd::from_euler(e);
    CHECK(doctest::Approx(std::abs(q.dot(q2))) == 1.0);
  }
}

TEST_SUITE("Quat - Slerp") {
  TEST_CASE("slerp at t=0 and t=1 returns the endpoints") {
    Quatd a = Quatd::from_axis_angle(Vec3d(0.0, 0.0, 1.0), 0.0);
    Quatd b = Quatd::from_axis_angle(Vec3d(0.0, 0.0, 1.0), 1.5);
    Quatd s0 = a.slerp(b, 0.0);
    Quatd s1 = a.slerp(b, 1.0);
    CHECK(doctest::Approx(s0.w) == a.w);
    CHECK(doctest::Approx(s1.w) == b.w);
  }

  TEST_CASE("slerp halfway interpolates the rotation angle") {
    Quatd a   = Quatd::from_axis_angle(Vec3d(0.0, 0.0, 1.0), 0.0);
    Quatd b   = Quatd::from_axis_angle(Vec3d(0.0, 0.0, 1.0), 1.5);
    Quatd mid = slerp(a, b, 0.5);
    CHECK(doctest::Approx(mid.get_angle()) == 0.75);
  }
}
