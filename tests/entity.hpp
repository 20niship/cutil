#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <cutil/pool.hpp>
#include <cutil/prop.hpp>
#include <cutil/ref.hpp>
#include <cutil/string.hpp>

namespace cutil {

// Entity System base class
// enable_ref_from_thisを継承して Ref<T> Create() を実装

class Mesh final : public enable_ref_from_this<Mesh> {
public:
  int vertex_count = 0;

  // Prop::dump()/load_to() 用のルール。vertex_countのみがPOD相当のフィールド。
  // Mesh/ModelはBase(enable_ref_from_this)を持つためstandard-layoutではないが、
  // 単一の非仮想継承なのでoffsetofの結果自体は主要コンパイラで安全に扱える。
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
  static const PropInfo* get_propinfo() {
    static const PropInfo rule = {
        {"vertex_count", PropType::Int, offsetof(Mesh, vertex_count), sizeof(int), false},
    };
    return &rule;
  }
#pragma GCC diagnostic pop

  // get_propinfo()のルールに従って自身をPropへ書き出す/Propから復元する薄いラッパー。
  [[nodiscard]] Prop dump() const {
    Prop p;
    p.dump(this, get_propinfo());
    return p;
  }
  bool load(const Prop& p) { return p.load_to(this, get_propinfo()); }

  template <size_t PoolSize = 64> static Ref<Mesh> Create(int vertex_count, ObjectPool<Mesh, PoolSize>* pool = nullptr) {
    if(!pool) {
      auto ref          = make_ref<Mesh>();
      ref->vertex_count = vertex_count;
      return ref;
    }

    // pool から slot を確保
    auto slot         = pool->allocate_slot_for_external();
    Mesh* obj         = new(slot->storage) Mesh();
    obj->vertex_count = vertex_count;

    // カスタムデリーターで pool へ返却
    auto deleter = [pool, slot](Mesh* p) {
      p->~Mesh();
      pool->deallocate_slot_for_external(slot);
    };

    return Ref<Mesh>(obj, deleter);
  }

private:
  Mesh() = default;
  friend Ref<Mesh>;
  template <typename U, typename... Args> friend Ref<U> make_ref(Args&&...);
};

class Model final : public enable_ref_from_this<Model> {
public:
  Str name;
  float position[3] = {0.0f, 0.0f, 0.0f};
  std::vector<Ref<Mesh>> meshes;
  WeakPtr<Model> parent; // WeakPtr で循環参照を防止
  std::vector<Ref<Model>> children;

  // Prop::dump()/load_to() 用のルール。name/positionはPOD/Str相当、
  // parent/meshes/childrenはPropType::Ref/RefList経由で「生ポインタ」として
  // やり取りする(実体はObjectPool/Ref側に置いたまま、Prop側は参照のみ保持する)。
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
  static const PropInfo* get_propinfo() {
    static const PropInfo rule = {
        {"name", PropType::Str, offsetof(Model, name), sizeof(Str), true},
        {"position", PropType::Vec3, offsetof(Model, position), sizeof(Vec3f), false}, // float[3]とVec3fはバイト互換
        PropInfo::Data::make_ref<Model>("parent", offsetof(Model, parent)),
        PropInfo::Data::make_ref_list<Mesh>("meshes", offsetof(Model, meshes)),
        PropInfo::Data::make_ref_list<Model>("children", offsetof(Model, children)),
    };
    return &rule;
  }
#pragma GCC diagnostic pop

  // get_propinfo()のルールに従って自身をPropへ書き出す/Propから復元する薄いラッパー。
  // parent/meshes/childrenも(生きたオブジェクトへの参照として)含めて復元される。
  [[nodiscard]] Prop dump() const {
    Prop p;
    p.dump(this, get_propinfo());
    return p;
  }
  bool load(const Prop& p) { return p.load_to(this, get_propinfo()); }

  void add_child(Ref<Model> child) {
    if(!child) return;
    child->parent = weak_from_this();
    children.push_back(child);
  }

  void remove_child(Ref<Model> child) {
    auto it = std::find(children.begin(), children.end(), child);
    if(it != children.end()) {
      child->parent.reset();
      children.erase(it);
    }
  }

  void add_mesh(Ref<Mesh> mesh) {
    if(mesh) meshes.push_back(mesh);
  }

  // parent を Ref<Model> にロックする（存在確認）
  Ref<Model> get_parent() const { return parent.lock(); }

  template <size_t PoolSize = 64> static Ref<Model> Create(const std::string& name = "", ObjectPool<Model, PoolSize>* pool = nullptr) {
    if(!pool) {
      auto ref  = make_ref<Model>();
      ref->name = name;
      return ref;
    }

    // pool から slot を確保
    auto slot  = pool->allocate_slot_for_external();
    Model* obj = new(slot->storage) Model();
    obj->name  = name;

    // カスタムデリーターで pool へ返却
    auto deleter = [pool, slot](Model* p) {
      p->~Model();
      pool->deallocate_slot_for_external(slot);
    };

    return Ref<Model>(obj, deleter);
  }

private:
  Model() = default;
  friend Ref<Model>;
  template <typename U, typename... Args> friend Ref<U> make_ref(Args&&...);
};

class Scene final : public enable_ref_from_this<Scene> {
public:
  std::string name;
  std::vector<Ref<Model>> root_models;

  void add_model(Ref<Model> model) {
    if(!model) return;
    model->parent.reset(); // root なので parent を解放
    root_models.push_back(model);
  }

  void remove_model(Ref<Model> model) {
    auto it = std::find(root_models.begin(), root_models.end(), model);
    if(it != root_models.end()) {
      root_models.erase(it);
    }
  }

  template <size_t PoolSize = 64> static Ref<Scene> Create(const std::string& name = "", ObjectPool<Scene, PoolSize>* pool = nullptr) {
    if(!pool) {
      auto ref  = make_ref<Scene>();
      ref->name = name;
      return ref;
    }

    // pool から slot を確保
    auto slot  = pool->allocate_slot_for_external();
    Scene* obj = new(slot->storage) Scene();
    obj->name  = name;

    // カスタムデリーターで pool へ返却
    auto deleter = [pool, slot](Scene* p) {
      p->~Scene();
      pool->deallocate_slot_for_external(slot);
    };

    return Ref<Scene>(obj, deleter);
  }

private:
  Scene() = default;
  friend Ref<Scene>;
  template <typename U, typename... Args> friend Ref<U> make_ref(Args&&...);
};

} // namespace cutil
