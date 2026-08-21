#include "doctest.h"
#include <cstddef>
#include <cutil/prop.hpp>

using cutil::Path;
using cutil::Prop;
using cutil::PropInfo;
using cutil::Quat;
using cutil::Rect3D;
using cutil::Str;
using cutil::Vec3f;
using cutil::Vec4f;

namespace {

// issueで例示されているModel3D構造体。
struct Model3D {
  Vec3f pos;
  Vec3f scale;
  Quat<float> quat;
  Path path;
  Str name;
  bool visible = false;
  bool animate = false;
  Rect3D bbox;
  Vec4f color;
};

// 型ごとに一度だけ静的に生成される「ルール」。
const PropInfo& Model3DInfo() {
  static const PropInfo rule = {
      {"pos", offsetof(Model3D, pos), cutil::prop_info_of<Vec3f>()},
      {"scale", offsetof(Model3D, scale), cutil::prop_info_of<Vec3f>()},
      {"quat", offsetof(Model3D, quat), cutil::prop_info_of<Quat<float>>()},
      {"path", offsetof(Model3D, path), cutil::prop_info_of<Path>()},
      {"name", offsetof(Model3D, name), cutil::prop_info_of<Str>()},
      {"visible", offsetof(Model3D, visible), cutil::prop_info_of<bool>()},
      {"animate", offsetof(Model3D, animate), cutil::prop_info_of<bool>()},
      {"bbox", offsetof(Model3D, bbox), cutil::prop_info_of<Rect3D>()},
      {"color", offsetof(Model3D, color), cutil::prop_info_of<Vec4f>()},
  };
  return rule;
}

} // namespace

TEST_SUITE("Prop - dump/load_to with external struct") {
  TEST_CASE("round-trip preserves all POD fields") {
    Model3D a;
    a.pos     = Vec3f(1, 2, 3);
    a.scale   = Vec3f(4, 5, 6);
    a.quat    = Quat<float>(0, 0, 0, 1);
    a.visible = true;
    a.animate = false;
    a.bbox    = Rect3D(Vec3f(0, 0, 0), Vec3f(1, 1, 1));
    a.color   = Vec4f(0.1f, 0.2f, 0.3f, 1.0f);

    Prop p;
    CHECK(p.dump(&a, &Model3DInfo()));

    Model3D b;
    CHECK(p.load_to(&b, &Model3DInfo()));

    CHECK(b.pos.data[0] == doctest::Approx(1.0f));
    CHECK(b.pos.data[2] == doctest::Approx(3.0f));
    CHECK(b.scale.data[1] == doctest::Approx(5.0f));
    CHECK(b.quat.w == doctest::Approx(1.0f));
    CHECK(b.visible == true);
    CHECK(b.animate == false);
    CHECK(b.bbox.x.max == doctest::Approx(1.0f));
    CHECK(b.color.data[3] == doctest::Approx(1.0f));
  }

  TEST_CASE("round-trip preserves Path and Str (pointer fields)") {
    Model3D a;
    a.path = Path("models/character.fbx");
    a.name = Str("a fairly long model name that will exceed sso capacity for sure");

    Prop p;
    CHECK(p.dump(&a, &Model3DInfo()));

    Model3D b;
    CHECK(p.load_to(&b, &Model3DInfo()));

    CHECK(b.path.str() == "models/character.fbx");
    CHECK(b.name == "a fairly long model name that will exceed sso capacity for sure");
  }

  TEST_CASE("dump then modify Prop does not affect original struct") {
    Model3D a;
    a.name = Str("original");

    Prop p;
    p.dump(&a, &Model3DInfo());
    p.get<Str>("name") = Str("modified in prop");

    CHECK(a.name == "original");
  }

  TEST_CASE("load_to into an already-populated struct overwrites pointer fields correctly") {
    Model3D a;
    a.name = Str("first value that is definitely long enough to be heap allocated");
    Prop p;
    p.dump(&a, &Model3DInfo());

    Model3D b;
    b.name = Str("pre-existing value in b that should be replaced without leaking");
    CHECK(p.load_to(&b, &Model3DInfo()));
    CHECK(b.name == "first value that is definitely long enough to be heap allocated");
  }

  TEST_CASE("re-dump same struct into same Prop (already has fields) updates values") {
    Model3D a;
    a.name = Str("v1");
    Prop p;
    p.dump(&a, &Model3DInfo());
    CHECK(p.field_count() == Model3DInfo().fields.size());

    a.name = Str("v2, now long enough to require heap allocation for this string");
    p.dump(&a, &Model3DInfo());
    CHECK(p.field_count() == Model3DInfo().fields.size()); // フィールド数は増えない(上書き)

    Model3D b;
    CHECK(p.load_to(&b, &Model3DInfo()));
    CHECK(b.name == "v2, now long enough to require heap allocation for this string");
  }

  TEST_CASE("load_to reports incomplete when rule references a missing Prop field") {
    Prop p; // 何もdumpしていない空のProp
    Model3D b;
    bool complete = p.load_to(&b, &Model3DInfo());
    CHECK(!complete);
  }
}
