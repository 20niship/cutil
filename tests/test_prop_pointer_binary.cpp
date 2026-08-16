#include "doctest.h"
#include <cstdio>
#include <cutil/prop.hpp>
#include <cutil/prop_io.hpp>
#include <cutil/prop_registry.hpp>
#include <new>
#include <string>

using cutil::CustomSlot;
using cutil::CustomTypeOps;
using cutil::CustomTypeRegistry;
using cutil::Path;
using cutil::Prop;
using cutil::Str;

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
  RegistryFixture() {
    CustomTypeOps ops;
    ops.size      = sizeof(DummyPoint);
    ops.align     = alignof(DummyPoint);
    ops.copy_ctor = [](void* dst, const void* src) { new(dst) DummyPoint(*reinterpret_cast<const DummyPoint*>(src)); };
    ops.dtor      = [](void* obj) { reinterpret_cast<DummyPoint*>(obj)->~DummyPoint(); };
    ops.to_json   = dummy_point_to_json;
    ops.from_json = dummy_point_from_json;
    CustomTypeRegistry::instance().register_type("DummyPoint", ops);
  }
};

} // namespace

TEST_SUITE("prop_dump_binary / prop_load_binary - pointer types (Phase 7)") {
  TEST_CASE("Str round-trip: SSO (short) and heap (long) both work") {
    Prop a;
    a.set<Str>("short_name", Str("abc"));
    a.set<Str>("long_name", Str("this is a fairly long string that will exceed sso capacity for sure"));

    std::vector<uint8_t> bytes;
    CHECK(cutil::prop_dump_binary(a, bytes));

    Prop b;
    CHECK(cutil::prop_load_binary(b, bytes));
    CHECK(b.get<Str>("short_name") == "abc");
    CHECK(b.get<Str>("long_name") == "this is a fairly long string that will exceed sso capacity for sure");
  }

  TEST_CASE("Binary round-trip: empty and large") {
    Prop a;
    a.set<std::vector<uint8_t>>("empty_blob", std::vector<uint8_t>{});
    std::vector<uint8_t> big(10000);
    for(size_t i = 0; i < big.size(); i++) big[i] = static_cast<uint8_t>(i % 256);
    a.set<std::vector<uint8_t>>("big_blob", big);

    std::vector<uint8_t> bytes;
    CHECK(cutil::prop_dump_binary(a, bytes));

    Prop b;
    CHECK(cutil::prop_load_binary(b, bytes));
    CHECK(b.get<std::vector<uint8_t>>("empty_blob").empty());
    CHECK(b.get<std::vector<uint8_t>>("big_blob") == big);
  }

  TEST_CASE("Path round-trip") {
    Prop a;
    a.set<Path>("asset", Path("models/character.fbx"));

    std::vector<uint8_t> bytes;
    CHECK(cutil::prop_dump_binary(a, bytes));

    Prop b;
    CHECK(cutil::prop_load_binary(b, bytes));
    CHECK(b.get<Path>("asset").str() == "models/character.fbx");
  }

  TEST_CASE("Custom type round-trip via to_json/from_json") {
    RegistryFixture fixture;
    Prop a;
    a.set_custom("origin", "DummyPoint", DummyPoint{7, 8});

    std::vector<uint8_t> bytes;
    CHECK(cutil::prop_dump_binary(a, bytes));

    Prop b;
    CHECK(cutil::prop_load_binary(b, bytes));
    CHECK(b.get_custom<DummyPoint>("origin").x == 7);
    CHECK(b.get_custom<DummyPoint>("origin").y == 8);
  }

  TEST_CASE("multi-level Nested Prop hierarchy round-trip") {
    Prop grandchild;
    grandchild.set<float>("value", 1.5f);

    Prop child;
    child.set<Str>("name", Str("child level"));
    child.set_child("gc", grandchild);

    Prop parent;
    parent.set<int32_t>("id", 42);
    parent.set_child("child", child);

    std::vector<uint8_t> bytes;
    CHECK(cutil::prop_dump_binary(parent, bytes));

    Prop restored;
    CHECK(cutil::prop_load_binary(restored, bytes));
    CHECK(restored.get<int32_t>("id") == 42);
    CHECK(restored.get_child("child").get<Str>("name") == "child level");
    CHECK(restored.get_child("child").get_child("gc").get<float>("value") == doctest::Approx(1.5f));
  }

  TEST_CASE("mixed POD and pointer fields preserve correct offsets") {
    Prop a;
    a.set<int32_t>("a", 1);
    a.set<Str>("s1", Str("hello world, this one is long enough to be heap allocated for real"));
    a.set<float>("b", 2.5f);
    a.set<std::vector<uint8_t>>("blob", std::vector<uint8_t>{10, 20, 30});
    a.set<Str>("s2", Str("short"));

    std::vector<uint8_t> bytes;
    CHECK(cutil::prop_dump_binary(a, bytes));

    Prop b;
    CHECK(cutil::prop_load_binary(b, bytes));
    CHECK(b.get<int32_t>("a") == 1);
    CHECK(b.get<Str>("s1") == "hello world, this one is long enough to be heap allocated for real");
    CHECK(b.get<float>("b") == doctest::Approx(2.5f));
    CHECK(b.get<std::vector<uint8_t>>("blob") == (std::vector<uint8_t>{10, 20, 30}));
    CHECK(b.get<Str>("s2") == "short");
  }

  TEST_CASE("out-of-range blob_offset/blob_size fails safely instead of crashing") {
    Prop a;
    a.set<Str>("name", Str("a string long enough to require heap allocation for the blob path"));

    std::vector<uint8_t> bytes;
    cutil::prop_dump_binary(a, bytes);

    // EntryTable内の該当エントリのdata_offsetから PropPointerDesc を読み取り、
    // blob_sizeを巨大な値に書き換えて破損データを模擬する。
    size_t entry_offset = sizeof(cutil::PropFileHeader);
    cutil::PropEntryHeader eh;
    std::memcpy(&eh, bytes.data() + entry_offset, sizeof(eh));
    REQUIRE((eh.flags & 0x1u) != 0);

    size_t data_block_offset = entry_offset + sizeof(cutil::PropEntryHeader);
    size_t pd_offset          = data_block_offset + eh.data_offset;
    cutil::PropPointerDesc pd;
    std::memcpy(&pd, bytes.data() + pd_offset, sizeof(pd));
    pd.blob_size = 0xFFFFFFFFu;
    std::memcpy(bytes.data() + pd_offset, &pd, sizeof(pd));

    Prop b;
    bool ok = cutil::prop_load_binary(b, bytes);
    CHECK(!ok); // クラッシュせず安全に失敗すること
  }
}
