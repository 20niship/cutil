#include "doctest.h"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <new>
#include <string>
#include <type_traits>
#include <typeindex>
#include <unordered_map>
#include <vector>

#include <cutil/dictionary.hpp>
#include <cutil/json.hpp>
#include <cutil/prop.hpp>
#include <cutil/prop_io.hpp>
#include <cutil/vector.hpp>

// EnttManager / ComponentRegistryはコア公開APIではなくテスト専用実装としてこのファイルに閉じ込める。

namespace cutil_test_entt {

using namespace cutil;

// --- EnttManager: 型消去コンポーネント配列 -----------------------------------

struct EnttManager {
  // 非trivial型はvector<uint8_t>のmemmove再配置で自己参照が壊れるためcopy_ctor/dtorで手動再構築する。data_.size()はcapacity、count_が有効要素数。
  struct EnttDataImpl {
    std::vector<uint8_t> data_;
    size_t count_                          = 0;
    uint32_t element_size_                = 0;
    void (*copy_ctor)(void*, const void*) = nullptr;
    void (*dtor)(void*)                    = nullptr;

    EnttDataImpl()                                = default;
    EnttDataImpl(const EnttDataImpl&)             = delete;
    EnttDataImpl& operator=(const EnttDataImpl&)  = delete;
    EnttDataImpl(EnttDataImpl&&)                  = default;
    EnttDataImpl& operator=(EnttDataImpl&&)       = default;

    ~EnttDataImpl() {
      if(!dtor) return;
      for(size_t i = 0; i < count_; i++) dtor(&data_[i * element_size_]);
    }
  };

  template <typename T> void add(const T& component) {
    std::type_index typeIndex(typeid(T));
    auto& enttData = entt_[typeIndex];
    if(enttData.element_size_ == 0) {
      enttData.element_size_ = sizeof(T);
      if constexpr(!std::is_trivially_copyable_v<T>) {
        enttData.copy_ctor = [](void* dst, const void* src) { new(dst) T(*reinterpret_cast<const T*>(src)); };
        enttData.dtor      = [](void* obj) { reinterpret_cast<T*>(obj)->~T(); };
      }
    }

    if(enttData.copy_ctor) {
      // 非trivial型: 容量が足りない場合はcopy_ctor/dtorで新バッファへ再構築してから追加する
      size_t needed_bytes = (enttData.count_ + 1) * enttData.element_size_;
      if(needed_bytes > enttData.data_.size()) {
        size_t new_count = enttData.count_ == 0 ? 1 : enttData.count_ * 2;
        std::vector<uint8_t> new_data(new_count * enttData.element_size_);
        for(size_t i = 0; i < enttData.count_; i++) {
          void* old_elem = &enttData.data_[i * enttData.element_size_];
          enttData.copy_ctor(&new_data[i * enttData.element_size_], old_elem);
          enttData.dtor(old_elem);
        }
        enttData.data_.swap(new_data);
      }
      enttData.copy_ctor(&enttData.data_[enttData.count_ * enttData.element_size_], &component);
    } else {
      // trivially copyable: vector<uint8_t>のresizeによる再配置はビットコピーで安全なので一括memcpyで済ませる
      enttData.data_.resize((enttData.count_ + 1) * enttData.element_size_);
      std::memcpy(&enttData.data_[enttData.count_ * enttData.element_size_], &component, sizeof(T));
    }
    enttData.count_++;
  }

  template <typename T> std::vector<T*> get() {
    std::type_index typeIndex(typeid(T));
    auto it = entt_.find(typeIndex);
    if(it == entt_.end()) return {}; // コンポーネントが存在しない場合は空のベクターを返す

    auto& enttData = it->second;
    std::vector<T*> components;
    components.reserve(enttData.count_);
    for(size_t i = 0; i < enttData.count_; i++) {
      components.push_back(reinterpret_cast<T*>(&enttData.data_[i * sizeof(T)]));
    }
    return components;
  }

  void clear() { entt_.clear(); }

  std::unordered_map<std::type_index, EnttDataImpl> entt_; // コンポーネントストレージを型ごとに管理するマップ
};

// --- Prop/PropInfoブリッジ: 型名 -> dump/loadルールのレジストリ ---------------

struct ComponentOps {
  std::type_index type_index = std::type_index(typeid(void));
  std::function<void(EnttManager&, Prop&)> dump_all;       // mgr.get<T>()の全要素をlist(空のProp)へ書き込む
  std::function<void(const Prop&, EnttManager&)> load_all; // listからTを復元しmgr.add<T>()で戻す
};

class ComponentRegistry {
public:
  static ComponentRegistry& instance() {
    static ComponentRegistry inst;
    return inst;
  }

  void register_type(const std::string& name, const ComponentOps& ops) { ops_.put(name, ops); }

  [[nodiscard]] const dictionary<ComponentOps>& all() const { return ops_; }

private:
  dictionary<ComponentOps> ops_;
};

// get_propinfo()を持たないleaf型向けのデフォルトルール(register_component_typeのget_rule省略時に使う)。
template <typename T> const PropInfo* single_field_propinfo() {
  static const PropInfo rule = {
      {"value", prop_type_traits<T>::type, 0, sizeof(T), prop_type_traits<T>::is_pointer},
  };
  return &rule;
}

// offsetofベースのPropInfoルールを持つ型(構造体 or leaf型)を登録する。
template <typename T> void register_component_type(const std::string& name, const PropInfo* (*get_rule)() = &single_field_propinfo<T>) {
  ComponentOps ops;
  ops.type_index = std::type_index(typeid(T));
  ops.dump_all   = [get_rule](EnttManager& mgr, Prop& list) {
    auto elems = mgr.get<T>();
    list.set<int32_t>("count", static_cast<int32_t>(elems.size()));
    for(size_t i = 0; i < elems.size(); i++) {
      Prop elem;
      elem.dump(elems[i], get_rule());
      list.set_child(std::to_string(i).c_str(), elem);
    }
  };
  ops.load_all = [get_rule](const Prop& list, EnttManager& mgr) {
    int32_t count = list.contains("count") ? list.get<int32_t>("count") : 0;
    for(int32_t i = 0; i < count; i++) {
      std::string key = std::to_string(i);
      if(!list.contains(key.c_str())) continue; // 壊れたデータでも安全にスキップ
      T tmp{};
      (void)list.get_child(key.c_str()).load_to(&tmp, get_rule());
      mgr.add(tmp);
    }
  };
  ComponentRegistry::instance().register_type(name, ops);
}

// offsetofで表現できないコンテナ型用。事前にregister_custom_type<T>(custom_type_name)が必要。
template <typename T> void register_component_type_custom(const std::string& name, const std::string& custom_type_name) {
  ComponentOps ops;
  ops.type_index = std::type_index(typeid(T));
  ops.dump_all   = [custom_type_name](EnttManager& mgr, Prop& list) {
    auto elems = mgr.get<T>();
    list.set<int32_t>("count", static_cast<int32_t>(elems.size()));
    for(size_t i = 0; i < elems.size(); i++) {
      list.set_custom(std::to_string(i).c_str(), custom_type_name.c_str(), *elems[i]);
    }
  };
  ops.load_all = [custom_type_name](const Prop& list, EnttManager& mgr) {
    int32_t count = list.contains("count") ? list.get<int32_t>("count") : 0;
    for(int32_t i = 0; i < count; i++) {
      std::string key = std::to_string(i);
      if(!list.contains(key.c_str())) continue;
      mgr.add(list.get_custom<T>(key.c_str()));
    }
  };
  ComponentRegistry::instance().register_type(name, ops);
}

// 登録済みの全コンポーネント型をProp階層へ書き出す/読み戻す。
inline void entt_dump(EnttManager& mgr, Prop& out) {
  for(const auto& [name, ops] : ComponentRegistry::instance().all()) {
    Prop list;
    ops.dump_all(mgr, list);
    out.set_child(name.c_str(), list);
  }
}

inline void entt_load(const Prop& in, EnttManager& mgr) {
  for(const auto& [name, ops] : ComponentRegistry::instance().all()) {
    if(!in.contains(name.c_str())) continue; // 未登録 or このセーブに無かった型はスキップ
    ops.load_all(in.get_child(name.c_str()), mgr);
  }
}

// プロジェクトファイル相当のエントリポイント。フォールバックはprop_load_binaryにそのまま委譲する。
inline bool entt_save_binary(EnttManager& mgr, std::vector<uint8_t>& out) {
  Prop root;
  entt_dump(mgr, root);
  return prop_dump_binary(root, out);
}

inline bool entt_load_binary(EnttManager& mgr, const std::vector<uint8_t>& bytes, const PropLoadFallback& fallback = nullptr) {
  Prop root;
  if(!prop_load_binary(root, bytes, fallback)) return false;
  entt_load(root, mgr);
  return true;
}

} // namespace cutil_test_entt

using namespace cutil;
using namespace cutil_test_entt;

namespace {

// --- テスト用コンポーネント型 -------------------------------------------------

struct EnttModel3D {
  Vec3f pos;
  Str name;
  bool visible  = false;
  int32_t mesh_id = -1; // EnttMeshへの参照(ファイル境界を越えても有効な整数ハンドル)
};

const PropInfo* EnttModel3DInfo() {
  static const PropInfo rule = {
      {"pos", PropType::Vec3, offsetof(EnttModel3D, pos), sizeof(EnttModel3D::pos), false},
      {"name", PropType::Str, offsetof(EnttModel3D, name), sizeof(EnttModel3D::name), true},
      {"visible", PropType::Bool, offsetof(EnttModel3D, visible), sizeof(EnttModel3D::visible), false},
      {"mesh_id", PropType::Int, offsetof(EnttModel3D, mesh_id), sizeof(EnttModel3D::mesh_id), false},
  };
  return &rule;
}

// 頂点1個分のデータ。全フィールドPODなのでtrivially copyable。
struct Vertex {
  Vec3f pos;
  Vec3f normal;
  float u = 0, v = 0;
};
static_assert(std::is_trivially_copyable_v<Vertex>, "Vertex must stay trivially copyable for the memcpy fast path below");

// 頂点情報を持つMesh。offsetofベースのPropInfoはvector<T>を表現できないためMesh丸ごとCustomSlotとして格納する。
struct EnttMesh {
  Str name;
  std::vector<Vertex> vertices;
};

void register_mesh_custom_type() {
  CustomTypeOps ops;
  ops.size      = sizeof(EnttMesh);
  ops.align     = alignof(EnttMesh);
  ops.copy_ctor = [](void* dst, const void* src) {
    const auto* s = reinterpret_cast<const EnttMesh*>(src);
    auto* d       = new(dst) EnttMesh();
    d->name       = s->name;
    d->vertices.resize(s->vertices.size());
    if(!s->vertices.empty()) {
      // Vertexはtrivially copyable: 要素ごとのコピーコンストラクタ呼び出しを回避し一括memcpyする
      std::memcpy(d->vertices.data(), s->vertices.data(), s->vertices.size() * sizeof(Vertex));
    }
  };
  ops.dtor    = [](void* obj) { reinterpret_cast<EnttMesh*>(obj)->~EnttMesh(); };
  ops.to_json = [](const void* obj) -> std::string {
    const auto* m    = reinterpret_cast<const EnttMesh*>(obj);
    json::Value root = json::Value::make_object();
    root.set("name", json::Value::make_string(m->name.c_str()));
    json::Value verts = json::Value::make_array();
    for(const auto& v : m->vertices) {
      json::Value jv = json::Value::make_object();
      jv.set("px", json::Value::make_double(v.pos.data[0]));
      jv.set("py", json::Value::make_double(v.pos.data[1]));
      jv.set("pz", json::Value::make_double(v.pos.data[2]));
      jv.set("nx", json::Value::make_double(v.normal.data[0]));
      jv.set("ny", json::Value::make_double(v.normal.data[1]));
      jv.set("nz", json::Value::make_double(v.normal.data[2]));
      jv.set("u", json::Value::make_double(v.u));
      jv.set("v", json::Value::make_double(v.v));
      verts.push_back(jv);
    }
    root.set("vertices", verts);
    return root.dump();
  };
  ops.from_json = [](void* obj, const std::string& text) -> bool {
    bool ok           = false;
    json::Value root  = json::Value::parse(text, &ok);
    if(!ok || !root.is_object()) return false;
    auto* m  = new(obj) EnttMesh();
    m->name  = Str(root.get("name").as_string().c_str());
    json::Value verts = root.get("vertices");
    size_t n           = verts.size();
    m->vertices.resize(n);
    for(size_t i = 0; i < n; i++) {
      json::Value jv = verts.get(i);
      Vertex v;
      v.pos.data[0]    = static_cast<float>(jv.get("px").as_double());
      v.pos.data[1]    = static_cast<float>(jv.get("py").as_double());
      v.pos.data[2]    = static_cast<float>(jv.get("pz").as_double());
      v.normal.data[0] = static_cast<float>(jv.get("nx").as_double());
      v.normal.data[1] = static_cast<float>(jv.get("ny").as_double());
      v.normal.data[2] = static_cast<float>(jv.get("nz").as_double());
      v.u              = static_cast<float>(jv.get("u").as_double());
      v.v               = static_cast<float>(jv.get("v").as_double());
      m->vertices[i]    = v;
    }
    return true;
  };
  CustomTypeRegistry::instance().register_type("Mesh", ops);
}

// 複数のModel3Dを束ねるScene。Ref/RefListはファイル永続化に使えないため整数ハンドル(model_id)のリストで参照する。
struct EnttScene {
  Str name;
  std::vector<int32_t> model_ids;
};

void register_scene_custom_type() {
  CustomTypeOps ops;
  ops.size      = sizeof(EnttScene);
  ops.align     = alignof(EnttScene);
  ops.copy_ctor = [](void* dst, const void* src) {
    const auto* s = reinterpret_cast<const EnttScene*>(src);
    auto* d       = new(dst) EnttScene();
    d->name       = s->name;
    d->model_ids  = s->model_ids; // int32_tはtrivially copyableなのでvectorのコピー代入自体が一括memcpy相当
  };
  ops.dtor    = [](void* obj) { reinterpret_cast<EnttScene*>(obj)->~EnttScene(); };
  ops.to_json = [](const void* obj) -> std::string {
    const auto* s    = reinterpret_cast<const EnttScene*>(obj);
    json::Value root = json::Value::make_object();
    root.set("name", json::Value::make_string(s->name.c_str()));
    json::Value ids = json::Value::make_array();
    for(int32_t id : s->model_ids) ids.push_back(json::Value::make_int(id));
    root.set("model_ids", ids);
    return root.dump();
  };
  ops.from_json = [](void* obj, const std::string& text) -> bool {
    bool ok          = false;
    json::Value root = json::Value::parse(text, &ok);
    if(!ok || !root.is_object()) return false;
    auto* s = new(obj) EnttScene();
    s->name = Str(root.get("name").as_string().c_str());
    json::Value ids = root.get("model_ids");
    size_t n         = ids.size();
    s->model_ids.resize(n);
    for(size_t i = 0; i < n; i++) s->model_ids[i] = static_cast<int32_t>(ids.get(i).as_int());
    return true;
  };
  CustomTypeRegistry::instance().register_type("Scene", ops);
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
    register_mesh_custom_type();
    register_component_type_custom<EnttMesh>("Mesh", "Mesh");
    register_scene_custom_type();
    register_component_type_custom<EnttScene>("Scene", "Scene");
  }
};

} // namespace

TEST_SUITE("EnttManager - Prop dump/load bridge") {
  TEST_CASE("struct component (offsetof rule) round-trips via entt_save_binary/entt_load_binary") {
    EnttPropFixture fixture;

    EnttManager mgr;
    mgr.add(EnttModel3D{Vec3f(1, 2, 3), Str("a"), true, 0});
    mgr.add(EnttModel3D{Vec3f(4, 5, 6), Str("b"), false, 1});

    std::vector<uint8_t> bytes;
    CHECK(entt_save_binary(mgr, bytes));

    EnttManager loaded;
    CHECK(entt_load_binary(loaded, bytes));

    auto models = loaded.get<EnttModel3D>();
    REQUIRE(models.size() == 2);
    CHECK(models[0]->pos.data[2] == doctest::Approx(3.0f));
    CHECK(models[0]->name == "a");
    CHECK(models[0]->visible == true);
    CHECK(models[0]->mesh_id == 0);
    CHECK(models[1]->name == "b");
    CHECK(models[1]->visible == false);
    CHECK(models[1]->mesh_id == 1);
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
    mgr.add(EnttModel3D{Vec3f(1, 1, 1), Str("x"), true, 0});

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

  TEST_CASE("Mesh component (vertex data via CustomTypeRegistry) round-trips") {
    EnttPropFixture fixture;

    EnttManager mgr;
    EnttMesh mesh;
    mesh.name = Str("cube");
    for(int i = 0; i < 8; i++) {
      Vertex v;
      v.pos    = Vec3f(static_cast<float>(i), static_cast<float>(i) * 2, static_cast<float>(i) * 3);
      v.normal = Vec3f(0, 1, 0);
      v.u      = static_cast<float>(i) * 0.1f;
      v.v      = static_cast<float>(i) * 0.2f;
      mesh.vertices.push_back(v);
    }
    mgr.add(mesh);

    std::vector<uint8_t> bytes;
    CHECK(entt_save_binary(mgr, bytes));

    EnttManager loaded;
    CHECK(entt_load_binary(loaded, bytes));

    auto meshes = loaded.get<EnttMesh>();
    REQUIRE(meshes.size() == 1);
    CHECK(meshes[0]->name == "cube");
    REQUIRE(meshes[0]->vertices.size() == 8);
    for(int i = 0; i < 8; i++) {
      const auto& v = meshes[0]->vertices[static_cast<size_t>(i)];
      CHECK(v.pos.data[0] == doctest::Approx(static_cast<float>(i)));
      CHECK(v.pos.data[1] == doctest::Approx(static_cast<float>(i) * 2));
      CHECK(v.u == doctest::Approx(static_cast<float>(i) * 0.1f));
    }
  }

  TEST_CASE("hundreds of Model3D/Mesh/Scene objects round-trip together, Scene resolving Model3D references by id") {
    EnttPropFixture fixture;

    constexpr int mesh_count  = 40;
    constexpr int model_count = 300;
    constexpr int scene_count = 6;

    EnttManager mgr;

    for(int i = 0; i < mesh_count; i++) {
      EnttMesh mesh;
      mesh.name = Str(("mesh_" + std::to_string(i)).c_str());
      for(int j = 0; j < 12; j++) {
        Vertex v;
        v.pos    = Vec3f(static_cast<float>(i), static_cast<float>(j), static_cast<float>(i + j));
        v.normal = Vec3f(0, 0, 1);
        v.u      = static_cast<float>(j) * 0.5f;
        v.v      = static_cast<float>(i) * 0.5f;
        mesh.vertices.push_back(v);
      }
      mgr.add(mesh);
    }

    for(int i = 0; i < model_count; i++) {
      EnttModel3D m;
      m.pos     = Vec3f(static_cast<float>(i), 0, 0);
      m.name    = Str(("model_" + std::to_string(i)).c_str());
      m.visible = (i % 2 == 0);
      m.mesh_id = i % mesh_count; // 複数のModel3DがMeshを共有する
      mgr.add(m);
    }

    for(int i = 0; i < scene_count; i++) {
      EnttScene scene;
      scene.name = Str(("scene_" + std::to_string(i)).c_str());
      for(int m = i * 50; m < i * 50 + 50; m++) scene.model_ids.push_back(m); // model_countと整合する範囲
      mgr.add(scene);
    }

    std::vector<uint8_t> bytes;
    CHECK(entt_save_binary(mgr, bytes));

    EnttManager loaded;
    CHECK(entt_load_binary(loaded, bytes));

    auto meshes = loaded.get<EnttMesh>();
    auto models = loaded.get<EnttModel3D>();
    auto scenes = loaded.get<EnttScene>();
    REQUIRE(meshes.size() == static_cast<size_t>(mesh_count));
    REQUIRE(models.size() == static_cast<size_t>(model_count));
    REQUIRE(scenes.size() == static_cast<size_t>(scene_count));

    for(int i = 0; i < mesh_count; i++) {
      CHECK(meshes[static_cast<size_t>(i)]->name == ("mesh_" + std::to_string(i)).c_str());
      REQUIRE(meshes[static_cast<size_t>(i)]->vertices.size() == 12);
    }

    for(int i = 0; i < model_count; i++) {
      const auto& m = *models[static_cast<size_t>(i)];
      CHECK(m.pos.data[0] == doctest::Approx(static_cast<float>(i)));
      CHECK(m.visible == (i % 2 == 0));
      CHECK(m.mesh_id == i % mesh_count);
      // Sc0eneがidだけで参照しているModel3Dの実体が、対応するMeshの頂点データまで正しく引ける
      REQUIRE(m.mesh_id >= 0);
      REQUIRE(m.mesh_id < mesh_count);
      CHECK(meshes[static_cast<size_t>(m.mesh_id)]->vertices.size() == 12);
    }

    for(int i = 0; i < scene_count; i++) {
      const auto& s = *scenes[static_cast<size_t>(i)];
      REQUIRE(s.model_ids.size() == 50);
      for(int32_t id : s.model_ids) {
        REQUIRE(id >= 0);
        REQUIRE(id < model_count);
        CHECK(models[static_cast<size_t>(id)]->mesh_id == id % mesh_count); // Scene -> Model3D -> Meshの参照が全部つながる
      }
    }
  }
}
