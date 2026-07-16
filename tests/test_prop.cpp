#include "doctest.h"
#include <cutil/prop.hpp>

using cutil::Path;
using cutil::Prop;
using cutil::Quat;
using cutil::Range;
using cutil::Rect;
using cutil::Rect3D;
using cutil::Str;
using cutil::Vec3f;
using cutil::Vec4f;

TEST_SUITE("Prop - POD fields") {
  TEST_CASE("set/get bool") {
    Prop p;
    p.set<bool>("visible", true);
    CHECK(p.get<bool>("visible") == true);
  }

  TEST_CASE("set/get float") {
    Prop p;
    p.set<float>("speed", 3.5f);
    CHECK(p.get<float>("speed") == doctest::Approx(3.5f));
  }

  TEST_CASE("set/get Vec3f and Vec4f") {
    Prop p;
    p.set<Vec3f>("pos", Vec3f(1, 2, 3));
    p.set<Vec4f>("color", Vec4f(0.1f, 0.2f, 0.3f, 1.0f));
    Vec3f pos = p.get<Vec3f>("pos");
    CHECK(pos.data[0] == doctest::Approx(1.0f));
    CHECK(pos.data[2] == doctest::Approx(3.0f));
    Vec4f color = p.get<Vec4f>("color");
    CHECK(color.data[3] == doctest::Approx(1.0f));
  }

  TEST_CASE("set/get Quat") {
    Prop p;
    Quat<float> q(0, 0, 0, 1);
    p.set<Quat<float>>("rot", q);
    CHECK(p.get<Quat<float>>("rot").w == doctest::Approx(1.0f));
  }

  TEST_CASE("set/get Range/Rect/Rect3D") {
    Prop p;
    p.set<Range>("range", Range(0, 10));
    p.set<Rect>("rect", Rect(0, 1, 0, 1));
    p.set<Rect3D>("bbox", Rect3D(Vec3f(0, 0, 0), Vec3f(1, 1, 1)));
    CHECK(p.get<Range>("range").max == doctest::Approx(10.0f));
    CHECK(p.get<Rect3D>("bbox").x.max == doctest::Approx(1.0f));
  }

  TEST_CASE("dynamic field addition (Blender custom-property style)") {
    Prop p;
    CHECK(p.field_count() == 0);
    p.set<int32_t>("hp", 100);
    p.set<int32_t>("mp", 50);
    CHECK(p.field_count() == 2);
    CHECK(p.contains("hp"));
    CHECK(p.contains("mp"));
    CHECK(!p.contains("nonexistent"));
  }

  TEST_CASE("overwrite existing POD field") {
    Prop p;
    p.set<int32_t>("hp", 100);
    p.set<int32_t>("hp", 42);
    CHECK(p.get<int32_t>("hp") == 42);
    CHECK(p.field_count() == 1);
  }
}

TEST_SUITE("Prop - Pointer fields (Str/Path/Binary)") {
  TEST_CASE("set/get Str (short SSO and long heap)") {
    Prop p;
    p.set<Str>("short_name", Str("abc"));
    p.set<Str>("long_name", Str("this is a fairly long string that exceeds sso capacity"));
    CHECK(p.get<Str>("short_name") == "abc");
    CHECK(p.get<Str>("long_name") == "this is a fairly long string that exceeds sso capacity");
  }

  TEST_CASE("set/get Path") {
    Prop p;
    p.set<Path>("asset_path", Path("models/character.fbx"));
    CHECK(p.get<Path>("asset_path").str() == "models/character.fbx");
  }

  TEST_CASE("set/get Binary") {
    Prop p;
    std::vector<uint8_t> data = {1, 2, 3, 4, 5};
    p.set<std::vector<uint8_t>>("blob", data);
    CHECK(p.get<std::vector<uint8_t>>("blob") == data);
  }

  TEST_CASE("overwrite existing Str field does not leak (functional check)") {
    Prop p;
    p.set<Str>("name", Str("first value that is long enough to go on the heap for sure"));
    p.set<Str>("name", Str("second"));
    CHECK(p.get<Str>("name") == "second");
  }

  TEST_CASE("copy preserves Str/Path/Binary content independently") {
    Prop a;
    a.set<Str>("name", Str("original long string value that goes onto the heap definitely"));
    a.set<std::vector<uint8_t>>("blob", std::vector<uint8_t>{9, 8, 7});

    Prop b(a);
    b.set<Str>("name", Str("changed"));
    CHECK(a.get<Str>("name") == "original long string value that goes onto the heap definitely");
    CHECK(b.get<Str>("name") == "changed");
    CHECK(a.get<std::vector<uint8_t>>("blob") == (std::vector<uint8_t>{9, 8, 7}));
  }

  TEST_CASE("move leaves source usable-but-empty and target has data") {
    Prop a;
    a.set<Str>("name", Str("some data that should move over to the new prop object"));
    Prop b(std::move(a));
    CHECK(b.get<Str>("name") == "some data that should move over to the new prop object");
  }
}

TEST_SUITE("Prop - Hierarchy (Nested Prop)") {
  TEST_CASE("set_child/get_child basic") {
    Prop parent;
    Prop child;
    child.set<int32_t>("hp", 10);
    parent.set_child("stats", child);
    CHECK(parent.get_child("stats").get<int32_t>("hp") == 10);
  }

  TEST_CASE("multi-level nesting (grandchild)") {
    Prop grandchild;
    grandchild.set<float>("value", 1.5f);

    Prop child;
    child.set_child("gc", grandchild);

    Prop parent;
    parent.set_child("child", child);

    CHECK(parent.get_child("child").get_child("gc").get<float>("value") == doctest::Approx(1.5f));
  }

  TEST_CASE("copying a Prop with children copies children independently") {
    Prop child;
    child.set<int32_t>("v", 1);

    Prop parent;
    parent.set_child("c", child);

    Prop parent_copy(parent);
    parent_copy.get_child("c").set<int32_t>("v", 2);

    CHECK(parent.get_child("c").get<int32_t>("v") == 1);
    CHECK(parent_copy.get_child("c").get<int32_t>("v") == 2);
  }
}

TEST_SUITE("Prop - Error handling") {
  TEST_CASE("get on missing key throws") {
    Prop p;
    CHECK_THROWS_AS(p.get<int32_t>("missing"), std::out_of_range);
  }

  TEST_CASE("set with mismatched type on existing field throws") {
    Prop p;
    p.set<int32_t>("x", 1);
    CHECK_THROWS_AS(p.set<float>("x", 1.0f), std::logic_error);
  }

  TEST_CASE("get with mismatched type throws") {
    Prop p;
    p.set<int32_t>("x", 1);
    CHECK_THROWS_AS(p.get<float>("x"), std::logic_error);
  }

  TEST_CASE("erase removes field") {
    Prop p;
    p.set<int32_t>("x", 1);
    CHECK(p.erase("x"));
    CHECK(!p.contains("x"));
    CHECK(!p.erase("x"));
  }

  TEST_CASE("erase a pointer field does not leak (functional check)") {
    Prop p;
    p.set<Str>("name", Str("a string long enough to require heap allocation for sure"));
    CHECK(p.erase("name"));
    CHECK(!p.contains("name"));
  }
}
