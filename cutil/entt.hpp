#pragma once
#include <cstdint>
#include <new>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace cutil {

struct EnttManager {

  // copy_ctor/dtorはadd<T>()がTごとに焼き込む(memcpyのみだとheap所有型が元オブジェクト破棄でdangling化するため)。
  struct EnttDataImpl {
    std::vector<uint8_t> data_;
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
      for(size_t i = 0; i + element_size_ <= data_.size(); i += element_size_) dtor(&data_[i]);
    }
  };

  template <typename T> void add(const T& component) {
    std::type_index typeIndex(typeid(T));
    auto& enttData = entt_[typeIndex];
    if(!enttData.copy_ctor) {
      enttData.element_size_ = sizeof(T);
      enttData.copy_ctor     = [](void* dst, const void* src) { new(dst) T(*reinterpret_cast<const T*>(src)); };
      enttData.dtor          = [](void* obj) { reinterpret_cast<T*>(obj)->~T(); };
    }
    size_t old_size = enttData.data_.size();
    enttData.data_.resize(old_size + sizeof(T));
    enttData.copy_ctor(&enttData.data_[old_size], &component);
  }

  template <typename T> std::vector<T*> get() {
    std::type_index typeIndex(typeid(T));
    auto it = entt_.find(typeIndex);
    if(it == entt_.end()) return {}; // コンポーネントが存在しない場合は空のベクターを返す

    auto& enttData = it->second;
    std::vector<T*> components;
    for(size_t i = 0; i + sizeof(T) <= enttData.data_.size(); i += sizeof(T)) {
      components.push_back(reinterpret_cast<T*>(&enttData.data_[i]));
    }
    return components;
  }

  void clear() { entt_.clear(); }

  std::unordered_map<std::type_index, EnttDataImpl> entt_; // コンポーネントストレージを型ごとに管理するマップ
};

} // namespace cutil
