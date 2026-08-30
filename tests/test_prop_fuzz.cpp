#include "doctest.h"
#include <cstdint>
#include <cstring>
#include <cutil/prop.hpp>
#include <cutil/prop_io.hpp>
#include <random>
#include <string>
#include <vector>

using cutil::Prop;
using cutil::PropInfo;
using cutil::Quat;
using cutil::Rect3D;
using cutil::Str;
using cutil::Vec3f;
using cutil::Vec4f;

namespace {

Prop make_sample_prop() {
  Prop p;
  p.set<bool>("visible", true);
  p.set<int32_t>("hp", 42);
  p.set<float>("speed", 3.5f);
  p.set<Vec3f>("pos", Vec3f(1, 2, 3));
  p.set<Vec4f>("color", Vec4f(0.1f, 0.2f, 0.3f, 1.0f));
  p.set<Quat<float>>("rot", Quat<float>(0, 0, 0, 1));
  p.set<Rect3D>("bbox", Rect3D(Vec3f(0, 0, 0), Vec3f(1, 1, 1)));
  p.set<Str>("name", Str("a fairly long string that lives on the heap for sure"));
  return p;
}

} // namespace

TEST_SUITE("Prop fuzz - malformed binary must not crash") {
  TEST_CASE("random byte-flip mutation of a valid binary blob") {
    Prop a = make_sample_prop();
    std::vector<uint8_t> golden;
    REQUIRE(cutil::prop_dump_binary(a, golden));
    REQUIRE(!golden.empty());

    std::mt19937 rng(1234);
    for(int iter = 0; iter < 2000; iter++) {
      std::vector<uint8_t> bytes = golden;
      std::uniform_int_distribution<size_t> pos_dist(0, bytes.size() - 1);
      std::uniform_int_distribution<int> flips_dist(1, 8);
      int flips = flips_dist(rng);
      for(int i = 0; i < flips; i++) {
        bytes[pos_dist(rng)] = static_cast<uint8_t>(rng());
      }

      Prop b;
      // クラッシュせず戻ってくること自体がテスト対象。戻り値のtrue/falseは問わない。
      bool ok = cutil::prop_load_binary(b, bytes);
      (void)ok;
    }
  }

  TEST_CASE("random length truncation/extension of a valid binary blob") {
    Prop a = make_sample_prop();
    std::vector<uint8_t> golden;
    REQUIRE(cutil::prop_dump_binary(a, golden));

    std::mt19937 rng(5678);
    for(size_t len = 0; len <= golden.size() + 16; len++) {
      std::vector<uint8_t> bytes(golden.begin(), golden.begin() + std::min(len, golden.size()));
      bytes.resize(len, 0);
      Prop b;
      bool ok = cutil::prop_load_binary(b, bytes);
      (void)ok;
    }
  }

  TEST_CASE("fully random garbage bytes of varying sizes") {
    std::mt19937 rng(999);
    for(size_t size : {0u, 1u, 4u, 16u, 64u, 256u, 4096u}) {
      for(int iter = 0; iter < 50; iter++) {
        std::vector<uint8_t> bytes(size);
        for(auto& b : bytes) b = static_cast<uint8_t>(rng());
        Prop p;
        bool ok = cutil::prop_load_binary(p, bytes);
        (void)ok;
      }
    }
  }
}

TEST_SUITE("Prop fuzz - malformed JSON must not crash") {
  TEST_CASE("random mutation of valid JSON text") {
    Prop a = make_sample_prop();
    std::string golden;
    REQUIRE(cutil::prop_dump_json(a, golden));
    REQUIRE(!golden.empty());

    std::mt19937 rng(42);
    static const char charset[] = "{}[]\":,0123456789abcdefXYZ.-truefalsenull \n\t";
    for(int iter = 0; iter < 2000; iter++) {
      std::string text = golden;
      std::uniform_int_distribution<size_t> pos_dist(0, text.size() - 1);
      std::uniform_int_distribution<size_t> char_dist(0, sizeof(charset) - 2);
      std::uniform_int_distribution<int> edits_dist(1, 6);
      int edits = edits_dist(rng);
      for(int i = 0; i < edits; i++) {
        text[pos_dist(rng)] = charset[char_dist(rng)];
      }

      Prop b;
      bool ok = cutil::prop_load_json(b, text);
      (void)ok;
    }
  }

  TEST_CASE("random truncation of valid JSON text") {
    Prop a = make_sample_prop();
    std::string golden;
    REQUIRE(cutil::prop_dump_json(a, golden));

    for(size_t len = 0; len <= golden.size(); len++) {
      Prop b;
      bool ok = cutil::prop_load_json(b, golden.substr(0, len));
      (void)ok;
    }
  }

  TEST_CASE("random garbage text") {
    std::mt19937 rng(7);
    static const char charset[] = "{}[]\":,0123456789abcdefg.-\n\t ";
    for(int iter = 0; iter < 500; iter++) {
      std::uniform_int_distribution<size_t> len_dist(0, 200);
      std::uniform_int_distribution<size_t> char_dist(0, sizeof(charset) - 2);
      size_t len = len_dist(rng);
      std::string text(len, ' ');
      for(auto& c : text) c = charset[char_dist(rng)];

      Prop b;
      bool ok = cutil::prop_load_json(b, text);
      (void)ok;
    }
  }
}

TEST_SUITE("Prop fuzz - API misuse must fail safely, not corrupt memory") {
  TEST_CASE("get<T> with mismatched type throws instead of reading garbage") {
    Prop p;
    p.set<int32_t>("hp", 42);
    CHECK_THROWS_AS(p.get<float>("hp"), std::logic_error);
    CHECK_THROWS_AS(p.get<Str>("hp"), std::logic_error);
  }

  TEST_CASE("get<T> with missing field throws instead of reading out of bounds") {
    Prop p;
    CHECK_THROWS_AS(p.get<int32_t>("does_not_exist"), std::out_of_range);
  }

  TEST_CASE("set<T> re-declaring an existing field with a different type throws") {
    Prop p;
    p.set<int32_t>("hp", 1);
    CHECK_THROWS_AS(p.set<float>("hp", 1.0f), std::logic_error);
  }

  TEST_CASE("load_to with a rule field type mismatching the stored field reports incomplete, not a crash") {
    Prop p;
    p.set<int32_t>("hp", 42);

    struct Target {
      float hp = 0.0f; // Prop側はint32_tなのでload_toはこのフィールドをスキップするはず
    };
    static const PropInfo rule = {{"hp", offsetof(Target, hp), cutil::prop_info_of<float>()}};

    Target t;
    bool complete = p.load_to(&t, &rule);
    CHECK(!complete);
    CHECK(t.hp == doctest::Approx(0.0f)); // 型不一致フィールドは書き換えられていないこと
  }

  TEST_CASE("load_to with a rule referencing a field absent from the Prop reports incomplete") {
    Prop p; // 空
    struct Target {
      int32_t hp = -1;
    };
    static const PropInfo rule = {{"hp", offsetof(Target, hp), cutil::prop_info_of<int32_t>()}};

    Target t;
    bool complete = p.load_to(&t, &rule);
    CHECK(!complete);
    CHECK(t.hp == -1);
  }
}
