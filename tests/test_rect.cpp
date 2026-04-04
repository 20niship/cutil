#include "doctest.h"
#include <cutil/rect.hpp>

TEST_SUITE("Range") {
  TEST_CASE("Range construction") {
    Cutil::Range r(0.0f, 10.0f);
    CHECK(r.min == 0.0f);
    CHECK(r.max == 10.0f);
  }

  TEST_CASE("Range length and center") {
    Cutil::Range r(0.0f, 10.0f);
    CHECK(r.length() == 10.0f);
    CHECK(r.center() == 5.0f);
  }

  TEST_CASE("Range contains") {
    Cutil::Range r(0.0f, 10.0f);
    CHECK(r.contains(5.0f));
    CHECK(r.contains(0.0f));
    CHECK(r.contains(10.0f));
    CHECK_FALSE(r.contains(-1.0f));
    CHECK_FALSE(r.contains(11.0f));
  }

  TEST_CASE("Range intersects") {
    Cutil::Range r1(0.0f, 10.0f);
    Cutil::Range r2(5.0f, 15.0f);
    CHECK(r1.intersects(r2));

    Cutil::Range r3(20.0f, 30.0f);
    CHECK_FALSE(r1.intersects(r3));
  }

  TEST_CASE("Range merge") {
    Cutil::Range r1(0.0f, 10.0f);
    Cutil::Range r2(5.0f, 15.0f);
    r1.merge(r2);
    CHECK(r1.min == 0.0f);
    CHECK(r1.max == 15.0f);
  }

  TEST_CASE("Range expand") {
    Cutil::Range r(5.0f, 10.0f);
    r.expand(2.0f);
    CHECK(r.min == 2.0f);
    r.expand(15.0f);
    CHECK(r.max == 15.0f);
  }

  TEST_CASE("Range valid") {
    Cutil::Range r(0.0f, 10.0f);
    CHECK(r.valid());

    Cutil::Range r_invalid(10.0f, 0.0f);
    CHECK_FALSE(r_invalid.valid());
  }

  TEST_CASE("Range equality") {
    Cutil::Range r1(0.0f, 10.0f);
    Cutil::Range r2(0.0f, 10.0f);
    Cutil::Range r3(0.0f, 20.0f);
    CHECK(r1 == r2);
    CHECK_FALSE(r1 == r3);
  }
}

TEST_SUITE("Rect") {
  TEST_CASE("Rect construction") {
    Cutil::Rect rect(0.0f, 10.0f, 0.0f, 20.0f);
    CHECK(rect.left() == 0.0f);
    CHECK(rect.right() == 10.0f);
    CHECK(rect.top() == 0.0f);
    CHECK(rect.bottom() == 20.0f);
  }

  TEST_CASE("Rect size and center") {
    Cutil::Rect rect(0.0f, 10.0f, 0.0f, 20.0f);
    CHECK(rect.w() == 10.0f);
    CHECK(rect.h() == 20.0f);
    auto c = rect.center();
    CHECK(c[0] == 5.0f);
    CHECK(c[1] == 10.0f);
  }

  TEST_CASE("Rect area") {
    Cutil::Rect rect(0.0f, 10.0f, 0.0f, 20.0f);
    CHECK(rect.area() == 200.0f);
  }

  TEST_CASE("Rect contains point") {
    Cutil::Rect rect(0.0f, 10.0f, 0.0f, 20.0f);
    CHECK(rect.contains(5.0f, 10.0f));
    CHECK_FALSE(rect.contains(-1.0f, 10.0f));
    CHECK_FALSE(rect.contains(5.0f, -1.0f));
  }

  TEST_CASE("Rect PosSize") {
    auto rect = Cutil::Rect::PosSize(Cutil::Vector2f(0.0f, 0.0f), Cutil::Vector2f(10.0f, 20.0f));
    CHECK(rect.left() == 0.0f);
    CHECK(rect.right() == 10.0f);
    CHECK(rect.top() == 0.0f);
    CHECK(rect.bottom() == 20.0f);
  }

  TEST_CASE("Rect CenterSize") {
    auto rect = Cutil::Rect::CenterSize(Cutil::Vector2f(5.0f, 10.0f), Cutil::Vector2f(10.0f, 20.0f));
    CHECK(rect.center()[0] == 5.0f);
    CHECK(rect.center()[1] == 10.0f);
    CHECK(rect.w() == 10.0f);
    CHECK(rect.h() == 20.0f);
  }
}
