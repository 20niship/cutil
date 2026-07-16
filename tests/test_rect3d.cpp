#include "doctest.h"
#include <cutil/rect3d.hpp>

using cutil::Range;
using cutil::Rect3D;
using cutil::Vec3f;

TEST_SUITE("Rect3D - Basic Operations") {
  TEST_CASE("Construct from ranges") {
    Rect3D r(Range(0, 1), Range(0, 2), Range(0, 3));
    CHECK(r.x.min == 0);
    CHECK(r.x.max == 1);
    CHECK(r.y.max == 2);
    CHECK(r.z.max == 3);
  }

  TEST_CASE("Construct from pos/size") {
    Rect3D r(Vec3f(1, 2, 3), Vec3f(4, 5, 6));
    CHECK(r.x.min == 1);
    CHECK(r.x.max == 5);
    CHECK(r.y.min == 2);
    CHECK(r.y.max == 7);
    CHECK(r.z.min == 3);
    CHECK(r.z.max == 9);
  }

  TEST_CASE("center/size/pos") {
    Rect3D r(Vec3f(0, 0, 0), Vec3f(2, 4, 6));
    Vec3f c = r.center();
    CHECK(c.data[0] == doctest::Approx(1.0f));
    CHECK(c.data[1] == doctest::Approx(2.0f));
    CHECK(c.data[2] == doctest::Approx(3.0f));
    Vec3f s = r.size();
    CHECK(s.data[0] == doctest::Approx(2.0f));
    CHECK(s.data[1] == doctest::Approx(4.0f));
    CHECK(s.data[2] == doctest::Approx(6.0f));
  }

  TEST_CASE("contains point and rect") {
    Rect3D r(Vec3f(0, 0, 0), Vec3f(10, 10, 10));
    CHECK(r.contains(Vec3f(5, 5, 5)));
    CHECK(!r.contains(Vec3f(15, 5, 5)));
    Rect3D inner(Vec3f(1, 1, 1), Vec3f(2, 2, 2));
    CHECK(r.contains(inner));
  }

  TEST_CASE("intersects") {
    Rect3D a(Vec3f(0, 0, 0), Vec3f(5, 5, 5));
    Rect3D b(Vec3f(3, 3, 3), Vec3f(5, 5, 5));
    Rect3D c(Vec3f(10, 10, 10), Vec3f(5, 5, 5));
    CHECK(a.intersects(b));
    CHECK(!a.intersects(c));
  }

  TEST_CASE("volume") {
    Rect3D r(Vec3f(0, 0, 0), Vec3f(2, 3, 4));
    CHECK(r.volume() == doctest::Approx(24.0f));
  }

  TEST_CASE("merge and operator|") {
    Rect3D a(Vec3f(0, 0, 0), Vec3f(1, 1, 1));
    Rect3D b(Vec3f(5, 5, 5), Vec3f(1, 1, 1));
    Rect3D merged = a | b;
    CHECK(merged.x.min == 0);
    CHECK(merged.x.max == 6);
  }

  TEST_CASE("operator& intersection") {
    Rect3D a(Vec3f(0, 0, 0), Vec3f(5, 5, 5));
    Rect3D b(Vec3f(3, 3, 3), Vec3f(5, 5, 5));
    Rect3D isect = a & b;
    CHECK(isect.x.min == 3);
    CHECK(isect.x.max == 5);
  }

  TEST_CASE("valid()") {
    Rect3D r;
    CHECK(!r.valid());
    Rect3D r2(Vec3f(0, 0, 0), Vec3f(1, 1, 1));
    CHECK(r2.valid());
  }

  TEST_CASE("equality") {
    Rect3D a(Vec3f(0, 0, 0), Vec3f(1, 1, 1));
    Rect3D b(Vec3f(0, 0, 0), Vec3f(1, 1, 1));
    Rect3D c(Vec3f(0, 0, 0), Vec3f(2, 1, 1));
    CHECK(a == b);
    CHECK(!(a == c));
  }

  TEST_CASE("PosSize / CenterSize static factories") {
    Rect3D r1 = Rect3D::PosSize(Vec3f(0, 0, 0), Vec3f(2, 2, 2));
    CHECK(r1.x.max == 2);
    Rect3D r2 = Rect3D::CenterSize(Vec3f(0, 0, 0), Vec3f(2, 2, 2));
    CHECK(r2.x.min == doctest::Approx(-1.0f));
    CHECK(r2.x.max == doctest::Approx(1.0f));
  }
}
