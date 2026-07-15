#include "doctest.h"
#include <cutil/color.hpp>

TEST_SUITE("Color Conversions") {
  TEST_CASE("RGB2H basic test") {
    cutil::Vector3b col(255, 0, 0); // Red
    double hue = cutil::RGB2H(col);
    CHECK(doctest::Approx(hue).epsilon(1.0) == 0.0);
  }

  TEST_CASE("RGB2H green test") {
    cutil::Vector3b col(0, 255, 0); // Green
    double hue = cutil::RGB2H(col);
    CHECK(doctest::Approx(hue).epsilon(1.0) == 120.0);
  }

  TEST_CASE("RGB2H blue test") {
    cutil::Vector3b col(0, 0, 255); // Blue
    double hue = cutil::RGB2H(col);
    CHECK(doctest::Approx(hue).epsilon(1.0) == 240.0);
  }

  TEST_CASE("RGB2H gray test (no hue)") {
    cutil::Vector3b col(128, 128, 128); // Gray
    double hue = cutil::RGB2H(col);
    CHECK(hue == 0.0);
  }

  TEST_CASE("RGB2S red test") {
    cutil::Vector3b col(255, 0, 0); // Pure red
    double sat = cutil::RGB2S(col);
    CHECK(doctest::Approx(sat).epsilon(1.0) == 100.0);
  }

  TEST_CASE("RGB2S gray test (no saturation)") {
    cutil::Vector3b col(128, 128, 128); // Gray
    double sat = cutil::RGB2S(col);
    CHECK(sat == 0.0);
  }

  TEST_CASE("RGB2V red test") {
    cutil::Vector3b col(255, 0, 0); // Full value
    double val = cutil::RGB2V(col);
    CHECK(doctest::Approx(val).epsilon(1.0) == 100.0);
  }

  TEST_CASE("RGB2V black test") {
    cutil::Vector3b col(0, 0, 0); // No value
    double val = cutil::RGB2V(col);
    CHECK(val == 0.0);
  }
}
