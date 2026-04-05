#include "doctest.h"
#include <cutil/variant.hpp>
#include <string>

using cutil::variant;

TEST_SUITE("variant - Basic Operations") {
  TEST_CASE("Default constructor creates first type") {
    variant<int, double, std::string> v;
    CHECK(v.index() == 0);
    CHECK(v.get<int>() == 0);
  }

  TEST_CASE("Constructor from int") {
    variant<int, double, std::string> v(42);
    CHECK(v.index() == 0);
    CHECK(v.get<int>() == 42);
  }

  TEST_CASE("Constructor from double") {
    variant<int, double, std::string> v(3.14);
    CHECK(v.index() == 1);
    CHECK(v.get<double>() == doctest::Approx(3.14));
  }

  TEST_CASE("Constructor from string") {
    variant<int, double, std::string> v(std::string("hello"));
    CHECK(v.index() == 2);
    CHECK(v.get<std::string>() == "hello");
  }

  TEST_CASE("Copy constructor") {
    variant<int, double, std::string> v1(100);
    variant<int, double, std::string> v2(v1);
    CHECK(v2.index() == 0);
    CHECK(v2.get<int>() == 100);
  }

  TEST_CASE("Move constructor") {
    variant<int, double, std::string> v1(std::string("move test"));
    variant<int, double, std::string> v2(std::move(v1));
    CHECK(v2.index() == 2);
    CHECK(v2.get<std::string>() == "move test");
  }
}

TEST_SUITE("variant - Assignment") {
  TEST_CASE("Copy assignment") {
    variant<int, double> v1(42);
    variant<int, double> v2(3.14);
    v2 = v1;
    CHECK(v2.index() == 0);
    CHECK(v2.get<int>() == 42);
  }

  TEST_CASE("Move assignment") {
    variant<int, double, std::string> v1(std::string("test"));
    variant<int, double, std::string> v2(100);
    v2 = std::move(v1);
    CHECK(v2.index() == 2);
    CHECK(v2.get<std::string>() == "test");
  }

  TEST_CASE("Assignment from value") {
    variant<int, double, std::string> v(100);
    v = std::string("new value");
    CHECK(v.index() == 2);
    CHECK(v.get<std::string>() == "new value");
  }
}

TEST_SUITE("variant - Get Operations") {
  TEST_CASE("get<T> returns correct type") {
    variant<int, double, std::string> v(42);
    CHECK(v.get<int>() == 42);
  }

  TEST_CASE("get<T> throws on wrong type") {
    variant<int, double, std::string> v(42);
    CHECK_THROWS(v.get<double>());
    CHECK_THROWS(v.get<std::string>());
  }

  TEST_CASE("try_get returns pointer on match") {
    variant<int, double, std::string> v(42);
    int* ptr = v.try_get<int>();
    CHECK(ptr != nullptr);
    CHECK(*ptr == 42);
  }

  TEST_CASE("try_get returns nullptr on mismatch") {
    variant<int, double, std::string> v(42);
    double* ptr = v.try_get<double>();
    CHECK(ptr == nullptr);
    std::string* sptr = v.try_get<std::string>();
    CHECK(sptr == nullptr);
  }
}

TEST_SUITE("variant - Comparison") {
  TEST_CASE("Equality for same values") {
    variant<int, double> v1(42);
    variant<int, double> v2(42);
    CHECK(v1 == v2);
  }

  TEST_CASE("Inequality for different values") {
    variant<int, double> v1(42);
    variant<int, double> v2(43);
    CHECK(v1 != v2);
  }

  TEST_CASE("Inequality for different types") {
    variant<int, double> v1(42);
    variant<int, double> v2(42.0);
    CHECK(v1 != v2);
  }

  TEST_CASE("String comparison") {
    variant<int, std::string> v1(std::string("hello"));
    variant<int, std::string> v2(std::string("hello"));
    CHECK(v1 == v2);
  }
}

TEST_SUITE("variant - Complex Types") {
  TEST_CASE("Stores complex types correctly") {
    struct Point {
      int x, y;
      Point(int x = 0, int y = 0) : x(x), y(y) {}
      bool operator==(const Point& other) const { return x == other.x && y == other.y; }
    };

    variant<int, Point> v(Point(10, 20));
    CHECK(v.index() == 1);
    Point& p = v.get<Point>();
    CHECK(p.x == 10);
    CHECK(p.y == 20);
  }
}
