#include "doctest.h"
#include <cutil/path.hpp>

using cutil::Path;

TEST_SUITE("Path - Basic Operations") {
  TEST_CASE("Construct from c-string and str()") {
    Path p("foo/bar.txt");
    CHECK(p.str() == "foo/bar.txt");
    CHECK(!p.empty());
  }

  TEST_CASE("Default construct is empty") {
    Path p;
    CHECK(p.empty());
  }

  TEST_CASE("is_absolute") {
    Path rel("foo/bar.txt");
    CHECK(!rel.is_absolute());
#if defined(_WIN32)
    Path abs("C:/foo/bar.txt");
#else
    Path abs("/foo/bar.txt");
#endif
    CHECK(abs.is_absolute());
  }

  TEST_CASE("absolute() returns an absolute path") {
    Path rel("foo/bar.txt");
    Path abs = rel.absolute();
    CHECK(abs.is_absolute());
  }

  TEST_CASE("parent()") {
    Path p("foo/bar/baz.txt");
    CHECK(p.parent().str() == "foo/bar");
  }

  TEST_CASE("filename()") {
    Path p("foo/bar/baz.txt");
    CHECK(p.filename() == "baz.txt");
  }

  TEST_CASE("extension()") {
    Path p("foo/bar/baz.txt");
    CHECK(p.extension() == ".txt");
  }

  TEST_CASE("operator/ joins paths") {
    Path p("foo/bar");
    Path joined = p / cutil::Str("baz.txt");
    CHECK(joined.str() == "foo/bar/baz.txt");
  }

  TEST_CASE("relative_to()") {
#if defined(_WIN32)
    Path p("C:/foo/bar/baz.txt");
    Path base("C:/foo");
#else
    Path p("/foo/bar/baz.txt");
    Path base("/foo");
#endif
    Path rel = p.relative_to(base);
    CHECK(rel.str() == "bar/baz.txt");
  }

  TEST_CASE("equality operator") {
    Path a("foo/bar.txt");
    Path b("foo/bar.txt");
    Path c("foo/baz.txt");
    CHECK(a == b);
    CHECK(a != c);
  }
}
