#include "doctest.h"
#include <cutil/vector.hpp>

TEST_SUITE("Vector2") {
  TEST_CASE("Vector2f construction") {
    Cutil::Vector2f v(3.0f, 4.0f);
    CHECK(v[0] == 3.0f);
    CHECK(v[1] == 4.0f);
  }

  TEST_CASE("Vector2f default construction") {
    Cutil::Vector2f v;
    CHECK(v[0] == 0.0f);
    CHECK(v[1] == 0.0f);
  }

  TEST_CASE("Vector2f magnitude") {
    Cutil::Vector2f v(3.0f, 4.0f);
    CHECK(doctest::Approx(v.norm()) == 5.0f);
  }

  TEST_CASE("Vector2f addition") {
    Cutil::Vector2f v1(1.0f, 2.0f);
    Cutil::Vector2f v2(3.0f, 4.0f);
    auto v3 = v1 + v2;
    CHECK(v3[0] == 4.0f);
    CHECK(v3[1] == 6.0f);
  }

  TEST_CASE("Vector2f subtraction") {
    Cutil::Vector2f v1(5.0f, 7.0f);
    Cutil::Vector2f v2(2.0f, 3.0f);
    auto v3 = v1 - v2;
    CHECK(v3[0] == 3.0f);
    CHECK(v3[1] == 4.0f);
  }

  TEST_CASE("Vector2f scalar multiplication") {
    Cutil::Vector2f v(2.0f, 3.0f);
    auto v2 = v * 2.0f;
    CHECK(v2[0] == 4.0f);
    CHECK(v2[1] == 6.0f);
  }

  TEST_CASE("Vector2f dot product") {
    Cutil::Vector2f v1(1.0f, 2.0f);
    Cutil::Vector2f v2(3.0f, 4.0f);
    auto dot = v1.dot(v2);
    CHECK(dot == 11.0f);
  }

  TEST_CASE("Vector2f normalization") {
    Cutil::Vector2f v(3.0f, 4.0f);
    auto v_norm = v.normalize();
    CHECK(doctest::Approx(v_norm[0]) == 0.6f);
    CHECK(doctest::Approx(v_norm[1]) == 0.8f);
  }
}

TEST_SUITE("Vector3") {
  TEST_CASE("Vector3f construction") {
    Cutil::Vector3f v(1.0f, 2.0f, 3.0f);
    CHECK(v[0] == 1.0f);
    CHECK(v[1] == 2.0f);
    CHECK(v[2] == 3.0f);
  }

  TEST_CASE("Vector3f addition") {
    Cutil::Vector3f v1(1.0f, 2.0f, 3.0f);
    Cutil::Vector3f v2(4.0f, 5.0f, 6.0f);
    auto v3 = v1 + v2;
    CHECK(v3[0] == 5.0f);
    CHECK(v3[1] == 7.0f);
    CHECK(v3[2] == 9.0f);
  }

  TEST_CASE("Vector3f magnitude") {
    Cutil::Vector3f v(1.0f, 2.0f, 2.0f);
    CHECK(doctest::Approx(v.norm()) == 3.0f);
  }

  TEST_CASE("Vector3f cross product") {
    Cutil::Vector3f v1(1.0f, 0.0f, 0.0f);
    Cutil::Vector3f v2(0.0f, 1.0f, 0.0f);
    auto v3 = v1.cross(v2);
    CHECK(v3[0] == 0.0f);
    CHECK(v3[1] == 0.0f);
    CHECK(v3[2] == 1.0f);
  }
}
