#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <cutil/pool.hpp>
#include <cutil/ref.hpp>

namespace cutil {

// Entity System base class
// enable_ref_from_thisを継承して Ref<T> Create() を実装

class Mesh final : public enable_ref_from_this<Mesh> {
public:
  int vertex_count = 0;

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
  std::string name;
  float position[3] = {0.0f, 0.0f, 0.0f};
  std::vector<Ref<Mesh>> meshes;
  WeakPtr<Model> parent;  // WeakPtr で循環参照を防止
  std::vector<Ref<Model>> children;

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
  Ref<Model> get_parent() const {
    return parent.lock();
  }

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
    model->parent.reset();  // root なので parent を解放
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
