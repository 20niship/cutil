#include "doctest.h"
#include <cutil/prop.hpp>
#include <cutil/prop_io.hpp>

using cutil::Prop;
using cutil::PropInfo;
using cutil::Quat;
using cutil::Range;
using cutil::Rect;
using cutil::Rect3D;
using cutil::Str;
using cutil::Vec3f;
using cutil::Vec4f;

namespace {

struct DummyPoint {
  int x = 0;
  int y = 0;
};

std::string dummy_point_to_json(const void* obj) {
  const auto* p = reinterpret_cast<const DummyPoint*>(obj);
  return "{\"x\":" + std::to_string(p->x) + ",\"y\":" + std::to_string(p->y) + "}";
}

bool dummy_point_from_json(void* obj, const std::string& json) {
  int x = 0, y = 0;
  if(std::sscanf(json.c_str(), "{\"x\":%d,\"y\":%d}", &x, &y) != 2) return false;
  new(obj) DummyPoint{x, y};
  return true;
}

struct RegistryFixture {
  RegistryFixture() { cutil::register_dynamic_type<DummyPoint>("DummyPoint", dummy_point_to_json, dummy_point_from_json); }
};

} // namespace

TEST_SUITE("prop_dump_json / prop_load_json") {
  TEST_CASE("round-trip preserves POD fields") {
    Prop a;
    a.set<bool>("visible", true);
    a.set<int32_t>("hp", 42);
    a.set<float>("speed", 3.5f);
    a.set<Vec3f>("pos", Vec3f(1, 2, 3));
    a.set<Vec4f>("color", Vec4f(0.1f, 0.2f, 0.3f, 1.0f));
    a.set<Quat<float>>("rot", Quat<float>(0, 0, 0, 1));
    a.set<Range>("range", Range(0, 10));
    a.set<Rect>("rect", Rect(0, 1, 0, 1));
    a.set<Rect3D>("bbox", Rect3D(Vec3f(0, 0, 0), Vec3f(1, 1, 1)));

    std::string json;
    CHECK(cutil::prop_dump_json(a, json));

    Prop b;
    CHECK(cutil::prop_load_json(b, json));

    CHECK(b.get<bool>("visible") == true);
    CHECK(b.get<int32_t>("hp") == 42);
    CHECK(b.get<float>("speed") == doctest::Approx(3.5f));
    CHECK(b.get<Vec3f>("pos").data[2] == doctest::Approx(3.0f));
    CHECK(b.get<Vec4f>("color").data[3] == doctest::Approx(1.0f));
    CHECK(b.get<Quat<float>>("rot").w == doctest::Approx(1.0f));
    CHECK(b.get<Range>("range").max == doctest::Approx(10.0f));
    CHECK(b.get<Rect3D>("bbox").x.max == doctest::Approx(1.0f));
  }

  TEST_CASE("round-trip preserves Str/Path/Binary") {
    Prop a;
    a.set<Str>("name", Str("a fairly long string value that goes onto the heap for sure"));
    a.set<cutil::Path>("path", cutil::Path("assets/model.fbx"));
    a.set<std::vector<uint8_t>>("blob", std::vector<uint8_t>{1, 2, 3, 4});

    std::string json;
    cutil::prop_dump_json(a, json);

    Prop b;
    CHECK(cutil::prop_load_json(b, json));
    CHECK(b.get<Str>("name") == "a fairly long string value that goes onto the heap for sure");
    CHECK(b.get<cutil::Path>("path").str() == "assets/model.fbx");
    CHECK(b.get<std::vector<uint8_t>>("blob") == (std::vector<uint8_t>{1, 2, 3, 4}));
  }

  TEST_CASE("round-trip preserves nested Prop hierarchy") {
    Prop child;
    child.set<int32_t>("hp", 10);

    Prop parent;
    parent.set_child("stats", child);

    std::string json;
    cutil::prop_dump_json(parent, json);

    Prop restored;
    CHECK(cutil::prop_load_json(restored, json));
    CHECK(restored.get_child("stats").get<int32_t>("hp") == 10);
  }

  TEST_CASE("round-trip preserves Custom field via to_json/from_json") {
    RegistryFixture fixture;
    Prop a;
    a.set_custom("origin", "DummyPoint", DummyPoint{3, 4});

    std::string json;
    cutil::prop_dump_json(a, json);

    Prop b;
    CHECK(cutil::prop_load_json(b, json));
    CHECK(b.get_custom<DummyPoint>("origin").x == 3);
    CHECK(b.get_custom<DummyPoint>("origin").y == 4);
  }

  TEST_CASE("invalid json fails gracefully") {
    Prop p;
    CHECK(!cutil::prop_load_json(p, "{not valid json"));
  }
}

TEST_SUITE("prop_load_binary - JSON fallback on corrupted files") {
  TEST_CASE("automatic embedded-json fallback fires when magic is corrupted and no explicit fallback given") {
    // JsonBlock常時併載は廃止されたため、fallback未指定時は復元不能でfalseを返す(Dynamic型のBlobBlock内jsonのみ個別に残る)。
    Prop a;
    a.set<int32_t>("x", 100);

    std::vector<uint8_t> bytes;
    CHECK(cutil::prop_dump_binary(a, bytes));
    bytes[0] = 'X'; // magicを破壊

    Prop b;
    CHECK(!cutil::prop_load_binary(b, bytes));
  }

  TEST_CASE("explicit fallback is used for corrupted files") {
    Prop a;
    a.set<int32_t>("x", 100);
    std::vector<uint8_t> bytes;
    cutil::prop_dump_binary(a, bytes);
    bytes[0] = 'X';

    bool custom_fallback_called = false;
    Prop b;
    cutil::prop_load_binary(b, bytes, [&](Prop&, const std::vector<uint8_t>&) {
      custom_fallback_called = true;
      return true;
    });
    CHECK(custom_fallback_called);
  }
}
