#include "doctest.h"
#include <cstddef>
#include <cutil/json.hpp>
#include <cutil/prop_entt.hpp>
#include <cutil/vector.hpp>

using namespace cutil;

namespace {

struct EnttModel3D {
  Vec3f pos;
  Str name;
  bool visible = false;
};

const PropInfo* EnttModel3DInfo() {
  static const PropInfo rule = {
      {"pos", PropType::Vec3, offsetof(EnttModel3D, pos), sizeof(EnttModel3D::pos), false},
      {"name", PropType::Str, offsetof(EnttModel3D, name), sizeof(EnttModel3D::name), true},
      {"visible", PropType::Bool, offsetof(EnttModel3D, visible), sizeof(EnttModel3D::visible), false},
  };
  return &rule;
}

void register_int_list_custom_type() {
  CustomTypeOps ops;
  ops.size      = sizeof(uiVector<int32_t>);
  ops.align     = alignof(uiVector<int32_t>);
  ops.copy_ctor = [](void* dst, const void* src) { new(dst) uiVector<int32_t>(*reinterpret_cast<const uiVector<int32_t>*>(src)); };
  ops.dtor      = [](void* obj) { reinterpret_cast<uiVector<int32_t>*>(obj)->~uiVector(); };
  ops.to_json   = [](const void* obj) -> std::string {
    const auto* v  = reinterpret_cast<const uiVector<int32_t>*>(obj);
    json::Value arr = json::Value::make_array();
    for(int i = 0; i < v->size(); i++) arr.push_back(json::Value::make_int((*v)[i]));
    return arr.dump();
  };
  ops.from_json = [](void* obj, const std::string& text) -> bool {
    bool ok               = false;
    json::Value arr       = json::Value::parse(text, &ok);
    if(!ok) return false;
    auto* v = new(obj) uiVector<int32_t>();
    for(size_t i = 0; i < arr.size(); i++) v->push_back(static_cast<int32_t>(arr.get(i).as_int()));
    return true;
  };
  CustomTypeRegistry::instance().register_type("IntList", ops);
}

struct EnttPropFixture {
  EnttPropFixture() {
    register_component_type<EnttModel3D>("Model3D", &EnttModel3DInfo);
    register_component_type<Str>("Str");
    register_int_list_custom_type();
    register_component_type_custom<uiVector<int32_t>>("IntList", "IntList");
  }
};

} // namespace

TEST_SUITE("EnttManager - Prop dump/load bridge") {
  TEST_CASE("struct component (offsetof rule) round-trips via entt_save_binary/entt_load_binary") {
    EnttPropFixture fixture;

    EnttManager mgr;
    mgr.add(EnttModel3D{Vec3f(1, 2, 3), Str("a"), true});
    mgr.add(EnttModel3D{Vec3f(4, 5, 6), Str("b"), false});

    std::vector<uint8_t> bytes;
    CHECK(entt_save_binary(mgr, bytes));

    EnttManager loaded;
    CHECK(entt_load_binary(loaded, bytes));

    auto models = loaded.get<EnttModel3D>();
    REQUIRE(models.size() == 2);
    CHECK(models[0]->pos.data[2] == doctest::Approx(3.0f));
    CHECK(models[0]->name == "a");
    CHECK(models[0]->visible == true);
    CHECK(models[1]->name == "b");
    CHECK(models[1]->visible == false);
  }

  TEST_CASE("leaf component (single_field_propinfo default) round-trips") {
    EnttPropFixture fixture;

    EnttManager mgr;
    mgr.add(Str("hello"));
    mgr.add(Str("world"));

    std::vector<uint8_t> bytes;
    CHECK(entt_save_binary(mgr, bytes));

    EnttManager loaded;
    CHECK(entt_load_binary(loaded, bytes));

    auto strs = loaded.get<Str>();
    REQUIRE(strs.size() == 2);
    CHECK(*strs[0] == "hello");
    CHECK(*strs[1] == "world");
  }

  TEST_CASE("container component (uiVector<int32_t> via CustomTypeRegistry) round-trips") {
    EnttPropFixture fixture;

    EnttManager mgr;
    uiVector<int32_t> a;
    a.push_back(1);
    a.push_back(2);
    a.push_back(3);
    mgr.add(a);

    std::vector<uint8_t> bytes;
    CHECK(entt_save_binary(mgr, bytes));

    EnttManager loaded;
    CHECK(entt_load_binary(loaded, bytes));

    auto lists = loaded.get<uiVector<int32_t>>();
    REQUIRE(lists.size() == 1);
    REQUIRE(lists[0]->size() == 3);
    CHECK((*lists[0])[0] == 1);
    CHECK((*lists[0])[2] == 3);
  }

  TEST_CASE("entt_load_binary forwards a caller-supplied fallback on corrupted data") {
    EnttPropFixture fixture;

    EnttManager mgr;
    mgr.add(EnttModel3D{Vec3f(1, 1, 1), Str("x"), true});

    std::vector<uint8_t> bytes;
    CHECK(entt_save_binary(mgr, bytes));
    bytes[0] = 'X'; // magicを破壊

    EnttManager loaded;
    bool fallback_called = false;
    bool ok               = entt_load_binary(loaded, bytes, [&](Prop&, const std::vector<uint8_t>&) {
      fallback_called = true;
      return false;
    });
    CHECK(fallback_called);
    CHECK(!ok);
  }
}
