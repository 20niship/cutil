#include "doctest.h"
#include <cutil/json.hpp>

using cutil::json::Value;

TEST_SUITE("json - Basic Operations") {
  TEST_CASE("parse and dump int") {
    bool ok;
    Value v = Value::parse("42", &ok);
    CHECK(ok);
    CHECK(v.as_int() == 42);
  }

  TEST_CASE("parse and dump float") {
    bool ok;
    Value v = Value::parse("3.5", &ok);
    CHECK(ok);
    CHECK(v.as_double() == doctest::Approx(3.5));
  }

  TEST_CASE("parse and dump string") {
    bool ok;
    Value v = Value::parse("\"hello\"", &ok);
    CHECK(ok);
    CHECK(v.as_string() == "hello");
  }

  TEST_CASE("parse invalid json fails gracefully") {
    bool ok;
    Value v = Value::parse("{not valid json", &ok);
    CHECK(!ok);
  }

  TEST_CASE("make_* factories and dump") {
    Value b = Value::make_bool(true);
    CHECK(b.as_bool() == true);
    Value i = Value::make_int(7);
    CHECK(i.as_int() == 7);
    Value d = Value::make_double(1.25);
    CHECK(d.as_double() == doctest::Approx(1.25));
    Value s = Value::make_string("abc");
    CHECK(s.as_string() == "abc");
  }

  TEST_CASE("nested object parse and get") {
    bool ok;
    Value v = Value::parse(R"({"a": 1, "b": {"c": 2}})", &ok);
    CHECK(ok);
    CHECK(v.is_object());
    CHECK(v.contains("a"));
    CHECK(v.contains("b"));
    CHECK(!v.contains("z"));
    CHECK(v.get("a").as_int() == 1);
    Value b = v.get("b");
    CHECK(b.is_object());
    CHECK(b.get("c").as_int() == 2);
  }

  TEST_CASE("build object with make_object/set") {
    Value obj = Value::make_object();
    obj.set("x", Value::make_int(10));
    obj.set("y", Value::make_string("hello"));
    CHECK(obj.is_object());
    CHECK(obj.contains("x"));
    CHECK(obj.get("x").as_int() == 10);
    CHECK(obj.get("y").as_string() == "hello");
  }

  TEST_CASE("array parse and size/get") {
    bool ok;
    Value v = Value::parse("[1, 2, 3]", &ok);
    CHECK(ok);
    CHECK(v.is_array());
    CHECK(v.size() == 3);
    CHECK(v.get(size_t(0)).as_int() == 1);
    CHECK(v.get(size_t(2)).as_int() == 3);
  }

  TEST_CASE("build array with make_array/push_back") {
    Value arr = Value::make_array();
    arr.push_back(Value::make_int(1));
    arr.push_back(Value::make_int(2));
    CHECK(arr.is_array());
    CHECK(arr.size() == 2);
    CHECK(arr.get(size_t(1)).as_int() == 2);
  }

  TEST_CASE("null value") {
    bool ok;
    Value v = Value::parse("null", &ok);
    CHECK(ok);
    CHECK(v.is_null());
  }

  TEST_CASE("round-trip dump then parse") {
    bool ok;
    Value v1 = Value::parse(R"({"x": 10, "y": [1,2,3]})", &ok);
    CHECK(ok);
    std::string dumped = v1.dump();
    Value v2            = Value::parse(dumped, &ok);
    CHECK(ok);
    CHECK(v2.is_object());
    CHECK(v2.get("x").as_int() == 10);
  }
}
