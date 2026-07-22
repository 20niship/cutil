#include "doctest.h"
#include <cstddef>
#include <cutil/prop.hpp>
#include <cutil/prop_registry.hpp>
#include <string>

using cutil::CustomSlot;
using cutil::CustomTypeOps;
using cutil::CustomTypeRegistry;
using cutil::Prop;
using cutil::register_custom_type;

namespace {

// issueに出てくるVideo型のようなカスタム型のダミー実装。
struct DummyVideoClip {
  std::string path;
  int fps = 0;
  int copy_count = 0; // copy_ctorが実際に呼ばれたことを確認するためのカウンタ

  DummyVideoClip() = default;
  DummyVideoClip(std::string p, int f) : path(std::move(p)), fps(f) {}
  DummyVideoClip(const DummyVideoClip& other) : path(other.path), fps(other.fps), copy_count(other.copy_count + 1) {}
};

struct RegistryFixture {
  RegistryFixture() { register_custom_type<DummyVideoClip>("DummyVideoClip"); }
};

} // namespace

TEST_SUITE("CustomTypeRegistry") {
  TEST_CASE("register and find a type") {
    RegistryFixture fixture;
    const CustomTypeOps* ops = CustomTypeRegistry::instance().find("DummyVideoClip");
    REQUIRE(ops != nullptr);
    CHECK(ops->size == sizeof(DummyVideoClip));
    CHECK(ops->align == alignof(DummyVideoClip));
    CHECK(ops->copy_ctor != nullptr);
    CHECK(ops->dtor != nullptr);
  }

  TEST_CASE("find unregistered type returns nullptr") {
    CHECK(CustomTypeRegistry::instance().find("NonexistentType12345") == nullptr);
  }
}

TEST_SUITE("CustomSlot") {
  TEST_CASE("make/get round-trip") {
    RegistryFixture fixture;
    DummyVideoClip clip("movie.mp4", 30);
    CustomSlot slot = CustomSlot::make<DummyVideoClip>("DummyVideoClip", clip);
    CHECK(slot.get<DummyVideoClip>().path == "movie.mp4");
    CHECK(slot.get<DummyVideoClip>().fps == 30);
  }

  TEST_CASE("copy constructor invokes the registered copy_ctor") {
    RegistryFixture fixture;
    DummyVideoClip clip("a.mp4", 24);
    CustomSlot slot1 = CustomSlot::make<DummyVideoClip>("DummyVideoClip", clip);
    CustomSlot slot2(slot1);
    CHECK(slot2.get<DummyVideoClip>().path == "a.mp4");
    CHECK(slot2.get<DummyVideoClip>().copy_count >= 1);
  }

  TEST_CASE("make with unregistered type throws") {
    struct Unregistered {};
    Unregistered u;
    CHECK_THROWS_AS(CustomSlot::make<Unregistered>("NeverRegistered", u), std::logic_error);
  }
}

TEST_SUITE("Prop - Custom fields via set_custom/get_custom") {
  TEST_CASE("set_custom/get_custom basic round-trip") {
    RegistryFixture fixture;
    Prop p;
    DummyVideoClip clip("intro.mp4", 60);
    p.set_custom("video", "DummyVideoClip", clip);
    CHECK(p.get_custom<DummyVideoClip>("video").path == "intro.mp4");
    CHECK(p.get_custom<DummyVideoClip>("video").fps == 60);
  }

  TEST_CASE("copying a Prop with a Custom field copies it independently") {
    RegistryFixture fixture;
    Prop a;
    a.set_custom("video", "DummyVideoClip", DummyVideoClip("first.mp4", 30));

    Prop b(a);
    b.get_custom<DummyVideoClip>("video").path = "changed.mp4";

    CHECK(a.get_custom<DummyVideoClip>("video").path == "first.mp4");
    CHECK(b.get_custom<DummyVideoClip>("video").path == "changed.mp4");
  }

  TEST_CASE("overwriting an existing Custom field does not leak (functional check)") {
    RegistryFixture fixture;
    Prop p;
    p.set_custom("video", "DummyVideoClip", DummyVideoClip("first.mp4", 30));
    p.set_custom("video", "DummyVideoClip", DummyVideoClip("second.mp4", 60));
    CHECK(p.get_custom<DummyVideoClip>("video").path == "second.mp4");
  }

  TEST_CASE("erase removes a Custom field without leaking (functional check)") {
    RegistryFixture fixture;
    Prop p;
    p.set_custom("video", "DummyVideoClip", DummyVideoClip("clip.mp4", 30));
    CHECK(p.erase("video"));
    CHECK(!p.contains("video"));
  }

  TEST_CASE("dump/load_to with a struct embedding a CustomSlot field") {
    RegistryFixture fixture;

    struct Entity {
      CustomSlot video;
    };

    static const cutil::PropInfo rule = {
        {"video", cutil::PropType::Custom, offsetof(Entity, video), sizeof(Entity::video), true},
    };

    Entity a;
    a.video = CustomSlot::make<DummyVideoClip>("DummyVideoClip", DummyVideoClip("dump.mp4", 25));

    Prop p;
    CHECK(p.dump(&a, &rule));
    CHECK(p.get_custom<DummyVideoClip>("video").path == "dump.mp4");

    Entity b;
    CHECK(p.load_to(&b, &rule));
    CHECK(b.video.get<DummyVideoClip>().path == "dump.mp4");
    CHECK(b.video.get<DummyVideoClip>().fps == 25);
  }
}
