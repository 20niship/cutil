#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <vector>

#include <cutil/pool.hpp>
#include <cutil/prop.hpp>
#include <cutil/ref.hpp>
#include <cutil/string.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"

namespace cutil {

// 頂点1個分のデータ。全フィールドPODなのでtrivially copyable。
struct Vertex {
  Vec3f pos;
  Vec3f normal;
  float u = 0, v = 0;
};
static_assert(std::is_trivially_copyable_v<Vertex>, "Vertex must stay trivially copyable for the memcpy fast path in EnttManager");

class Mesh final : public enable_ref_from_this<Mesh> {
public:
  int vertex_count = 0;
  std::vector<Vertex> vertices;

  static const PropInfo* get_propinfo() {
    static const PropInfo rule = {
      {"vertex_count", offsetof(Mesh, vertex_count), prop_info_of<int32_t>()},
      {"vertices", offsetof(Mesh, vertices), prop_info_of<std::vector<Vertex>>()},
    };
    return &rule;
  }

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
      {"name", offsetof(Model, name), prop_info_of<Str>()},
      {"position", offsetof(Model, position), prop_info_of<Vec3f>()}, // float[3]とVec3fはバイト互換
      PropInfo::Field::make_ref<Model>("parent", offsetof(Model, parent)),
      PropInfo::Field::make_ref_list<Mesh>("meshes", offsetof(Model, meshes)),
      PropInfo::Field::make_ref_list<Model>("children", offsetof(Model, children)),
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

// trivially copyableなTはcopy_ctor/dtorを焼き込まず一括memcpyする(非trivial型のみdeep copy)。data_.size()はcapacity、count_が有効要素数。
struct EnttManager {
  struct EnttDataImpl {
    std::vector<uint8_t> data_;
    size_t count_                         = 0;
    uint32_t element_size_                = 0;
    void (*copy_ctor)(void*, const void*) = nullptr;
    void (*dtor)(void*)                   = nullptr;

    EnttDataImpl()                               = default;
    EnttDataImpl(const EnttDataImpl&)            = delete;
    EnttDataImpl& operator=(const EnttDataImpl&) = delete;
    EnttDataImpl(EnttDataImpl&&)                 = default;
    EnttDataImpl& operator=(EnttDataImpl&&)      = default;

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
      enttData.data_.resize((enttData.count_ + 1) * enttData.element_size_);
      std::memcpy(&enttData.data_[enttData.count_ * enttData.element_size_], &component, sizeof(T));
    }
    enttData.count_++;
  }

  template <typename T> std::vector<T*> get() {
    std::type_index typeIndex(typeid(T));
    auto it = entt_.find(typeIndex);
    if(it == entt_.end()) return {};

    auto& enttData = it->second;
    std::vector<T*> components;
    components.reserve(enttData.count_);
    for(size_t i = 0; i < enttData.count_; i++) {
      components.push_back(reinterpret_cast<T*>(&enttData.data_[i * sizeof(T)]));
    }
    return components;
  }

  void clear() { entt_.clear(); }

  std::unordered_map<std::type_index, EnttDataImpl> entt_;
};

// EnttManagerに積む軽量コンポーネント。全フィールドPOD/Str/vectorでprop_info_of<T>()に乗る。
struct EnttModel3D {
  Vec3f pos;
  Str name;
  bool visible    = false;
  int32_t mesh_id = -1; // Meshへの参照(ファイル境界を越えても有効な整数ハンドル)
};

// 複数のModel3Dを束ねるコンポーネント。Ref/RefListはファイル永続化に使えないため整数ハンドル(model_id)のリストで参照する。
struct EnttScene {
  Str name;
  std::vector<int32_t> model_ids;
};

template <> struct PropInfoOf<Vertex> {
  static const PropInfo* get() {
    return register_struct_type<Vertex>("Vertex", {{"pos", offsetof(Vertex, pos), prop_info_of<Vec3f>()}, {"normal", offsetof(Vertex, normal), prop_info_of<Vec3f>()}, {"u", offsetof(Vertex, u), prop_info_of<float>()}, {"v", offsetof(Vertex, v), prop_info_of<float>()}});
  }
};
template <> struct PropInfoOf<EnttModel3D> {
  static const PropInfo* get() {
    return register_struct_type<EnttModel3D>("Model3D", {
                                                          {"pos", offsetof(EnttModel3D, pos), prop_info_of<Vec3f>()},
                                                          {"name", offsetof(EnttModel3D, name), prop_info_of<Str>()},
                                                          {"visible", offsetof(EnttModel3D, visible), prop_info_of<bool>()},
                                                          {"mesh_id", offsetof(EnttModel3D, mesh_id), prop_info_of<int32_t>()},
                                                        });
  }
};
template <> struct PropInfoOf<EnttScene> {
  static const PropInfo* get() {
    return register_struct_type<EnttScene>("Scene", {
                                                      {"name", offsetof(EnttScene, name), prop_info_of<Str>()},
                                                      {"model_ids", offsetof(EnttScene, model_ids), prop_info_of<std::vector<int32_t>>()},
                                                    });
  }
};

#pragma GCC diagnostic pop

} // namespace cutil
