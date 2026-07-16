#include "doctest.h"
#include <cutil/prop.hpp>
#include <cutil/prop_io.hpp>

using cutil::Prop;
using cutil::PropType;
using cutil::Quat;
using cutil::Range;
using cutil::Rect;
using cutil::Rect3D;
using cutil::Vec3f;
using cutil::Vec4f;

TEST_SUITE("prop_dump_binary / prop_load_binary - POD only") {
  TEST_CASE("round-trip preserves POD field values (bit-exact)") {
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

    std::vector<uint8_t> bytes;
    CHECK(cutil::prop_dump_binary(a, bytes));
    CHECK(!bytes.empty());

    Prop b;
    CHECK(cutil::prop_load_binary(b, bytes));

    CHECK(b.get<bool>("visible") == true);
    CHECK(b.get<int32_t>("hp") == 42);
    CHECK(b.get<float>("speed") == doctest::Approx(3.5f));
    CHECK(b.get<Vec3f>("pos").data[2] == doctest::Approx(3.0f));
    CHECK(b.get<Vec4f>("color").data[3] == doctest::Approx(1.0f));
    CHECK(b.get<Quat<float>>("rot").w == doctest::Approx(1.0f));
    CHECK(b.get<Range>("range").max == doctest::Approx(10.0f));
    CHECK(b.get<Rect3D>("bbox").x.max == doctest::Approx(1.0f));
  }

  TEST_CASE("pointer-type fields (Str) round-trip via the pointer blob path") {
    Prop a;
    a.set<int32_t>("hp", 1);
    a.set<cutil::Str>("name", cutil::Str("hello"));

    std::vector<uint8_t> bytes;
    CHECK(cutil::prop_dump_binary(a, bytes));

    Prop b;
    CHECK(cutil::prop_load_binary(b, bytes));
    CHECK(b.get<int32_t>("hp") == 1);
    CHECK(b.get<cutil::Str>("name") == "hello");
  }

  TEST_CASE("load with corrupted magic falls back") {
    Prop a;
    a.set<int32_t>("x", 1);
    std::vector<uint8_t> bytes;
    cutil::prop_dump_binary(a, bytes);
    bytes[0] = 'X'; // magicを破壊

    Prop b;
    bool fallback_called = false;
    bool ok = cutil::prop_load_binary(b, bytes, [&](Prop&, const std::vector<uint8_t>&) {
      fallback_called = true;
      return false;
    });
    CHECK(fallback_called);
    CHECK(!ok);
  }

  TEST_CASE("load without fallback returns false on corrupted data") {
    std::vector<uint8_t> bytes = {1, 2, 3}; // too small / invalid
    Prop b;
    CHECK(!cutil::prop_load_binary(b, bytes));
  }

  TEST_CASE("version mismatch triggers fallback") {
    Prop a;
    a.set<int32_t>("x", 1);
    std::vector<uint8_t> bytes;
    cutil::prop_dump_binary(a, bytes);

    // 既存のフィールドと異なるversionを事前にセットしたPropに対してloadすると
    // フォールバックが発火する。
    Prop b;
    b.set<int32_t>("x", 999);
    // bのフィールドのversionを直接書き換えて不一致を作る
    const_cast<cutil::PropInfoList&>(b.infos())[0].version = 2;

    bool fallback_called = false;
    cutil::prop_load_binary(b, bytes, [&](Prop&, const std::vector<uint8_t>&) {
      fallback_called = true;
      return true;
    });
    CHECK(fallback_called);
  }

  TEST_CASE("out-of-range data_offset/size fails safely instead of crashing") {
    Prop a;
    a.set<int32_t>("x", 1);
    std::vector<uint8_t> bytes;
    cutil::prop_dump_binary(a, bytes);
    // EntryTable内のdata_sizeを巨大な値に書き換えて破損データを模擬する
    size_t entry_offset  = sizeof(cutil::PropFileHeader);
    cutil::PropEntryHeader eh;
    std::memcpy(&eh, bytes.data() + entry_offset, sizeof(eh));
    eh.data_size = 0xFFFFFFFFu;
    std::memcpy(bytes.data() + entry_offset, &eh, sizeof(eh));

    Prop b;
    bool ok = cutil::prop_load_binary(b, bytes);
    CHECK(!ok); // クラッシュせず安全に失敗すること
  }
}
