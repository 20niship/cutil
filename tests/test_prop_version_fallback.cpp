#include "doctest.h"
#include <cstddef>
#include <cutil/prop.hpp>
#include <cutil/prop_io.hpp>

using cutil::Prop;
using cutil::PropInfo;
using cutil::Quat;
using cutil::Str;
using cutil::Vec3f;

namespace {

// v1: 旧バージョンのソフトウェアが保存していたスキーマ。
struct Model3D_v1 {
  Vec3f pos;
  Vec3f scale;
  Quat<float> quat;
  Str name;
  bool visible = false;
};

// v2: フィールドを追加した新バージョンのスキーマ(animate/opacityが増えている)。
struct Model3D_v2 {
  Vec3f pos;
  Vec3f scale;
  Quat<float> quat;
  Str name;
  bool visible  = false;
  bool animate  = false;
  float opacity = 1.0f;
};

} // namespace

namespace cutil {
template <> struct PropInfoOf<Model3D_v1> {
  static const PropInfo* get() {
    return register_struct_type<Model3D_v1>("Model3D", {
                                                         {"pos", offsetof(Model3D_v1, pos), prop_info_of<Vec3f>()},
                                                         {"scale", offsetof(Model3D_v1, scale), prop_info_of<Vec3f>()},
                                                         {"quat", offsetof(Model3D_v1, quat), prop_info_of<Quat<float>>()},
                                                         {"name", offsetof(Model3D_v1, name), prop_info_of<Str>()},
                                                         {"visible", offsetof(Model3D_v1, visible), prop_info_of<bool>()},
                                                       });
  }
};
template <> struct PropInfoOf<Model3D_v2> {
  static const PropInfo* get() {
    return register_struct_type<Model3D_v2>("Model3D", {
                                                         {"pos", offsetof(Model3D_v2, pos), prop_info_of<Vec3f>()},
                                                         {"scale", offsetof(Model3D_v2, scale), prop_info_of<Vec3f>()},
                                                         {"quat", offsetof(Model3D_v2, quat), prop_info_of<Quat<float>>()},
                                                         {"name", offsetof(Model3D_v2, name), prop_info_of<Str>()},
                                                         {"visible", offsetof(Model3D_v2, visible), prop_info_of<bool>()},
                                                         {"animate", offsetof(Model3D_v2, animate), prop_info_of<bool>()},
                                                         {"opacity", offsetof(Model3D_v2, opacity), prop_info_of<float>()},
                                                       });
  }
};
} // namespace cutil

namespace {

// PropInfoRegistryは"Model3D"に対し最後にprop_info_of<T>()を呼んだ型を生きたスキーマとして保持するため、テスト後はv1へ戻す。
struct RestoreModel3DRegistry {
  ~RestoreModel3DRegistry() { cutil::register_prop_type<Model3D_v1>("Model3D"); }
};

} // namespace

TEST_SUITE("Prop - schema version upgrade fallback (Model3D v1 -> v2)") {
  TEST_CASE("old-version binary loads into a newer schema with additional fields") {
    RestoreModel3DRegistry restore_guard;

    // 1. 旧バージョン(v1)としてダンプする。
    const PropInfo* v1_info = cutil::prop_info_of<Model3D_v1>();
    uint32_t v1_version      = v1_info->version;

    Model3D_v1 a;
    a.pos     = Vec3f(1, 2, 3);
    a.scale   = Vec3f(4, 5, 6);
    a.quat    = Quat<float>(0, 0, 0, 1);
    a.name    = Str("legacy model");
    a.visible = true;

    Prop dumped;
    dumped.set<Model3D_v1>("model", a);
    std::vector<uint8_t> bytes;
    REQUIRE(cutil::prop_dump_binary(dumped, bytes));

    // 2. v2を現行スキーマとして登録し、フィールド追加に伴いversionをインクリメントする(実運用の模擬)。
    const PropInfo* v2_info = cutil::prop_info_of<Model3D_v2>();
    auto* mutable_v2_info    = const_cast<PropInfo*>(v2_info);
    mutable_v2_info->version = v1_version + 1;

    // 3. 新バージョンのソフトウェアで旧バージョンのバイナリを読み込む。
    Prop loaded;
    CHECK(cutil::prop_load_binary(loaded, bytes));
    REQUIRE(loaded.contains("model"));

    const Model3D_v2& b = loaded.get<Model3D_v2>("model");

    // 型情報が変わらないTrivialフィールドはversion不一致でもフィールド名一致で復元される。
    CHECK(b.pos.data[0] == doctest::Approx(1.0f));
    CHECK(b.pos.data[2] == doctest::Approx(3.0f));
    CHECK(b.scale.data[1] == doctest::Approx(5.0f));
    CHECK(b.quat.w == doctest::Approx(1.0f));
    CHECK(b.visible == true);

    // 新フィールドはPropInfo::default_ctorの全体memsetで初期化されるため、C++のメンバ初期化子(1.0f)ではなく0になる。
    CHECK(b.animate == false);
    CHECK(b.opacity == doctest::Approx(0.0f));

    mutable_v2_info->version = v1_version; // 後片付け: 他テストで再度prop_info_of<Model3D_v2>()を呼んでも影響しないように
  }

  TEST_CASE("old-version binary with matching version fast-paths and preserves everything except size-changed layout") {
    RestoreModel3DRegistry restore_guard;

    const PropInfo* v1_info = cutil::prop_info_of<Model3D_v1>();
    Model3D_v1 a;
    a.pos  = Vec3f(9, 8, 7);
    a.name = Str("same version round-trip");

    Prop dumped;
    dumped.set<Model3D_v1>("model", a);
    std::vector<uint8_t> bytes;
    REQUIRE(cutil::prop_dump_binary(dumped, bytes));

    // versionを上げずにそのまま同じv1型で読み戻す(通常のバージョン一致ラウンドトリップ)。
    Prop loaded;
    CHECK(cutil::prop_load_binary(loaded, bytes));
    REQUIRE(loaded.contains("model"));
    const Model3D_v1& b = loaded.get<Model3D_v1>("model");
    CHECK(b.pos.data[0] == doctest::Approx(9.0f));
    CHECK(b.name == "same version round-trip");

    (void)v1_info;
  }
}
