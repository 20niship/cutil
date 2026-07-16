#pragma once

#include <cstdlib>
#include <cstring>
#include <new>
#include <stdexcept>
#include <string>

#include <cutil/dictionary.hpp>

// cutil::CustomTypeRegistry / cutil::CustomSlot: Prop に組み込まれていない任意の型
// (Blenderのカスタムプロパティ相当。例: Video)を、cutil本体を変更せず実行時に
// 後付けで登録できるようにする仕組み。
//
// このヘッダーは Prop/PropType に依存しない(単体で完結した型消去ユーティリティ)。
// cutil/prop.hpp 側がこれをincludeし、PropType::Custom のフィールドを CustomSlot
// として扱うように配線する。

namespace cutil {

struct CustomTypeOps {
  size_t size                                  = 0;
  size_t align                                 = 1;
  void (*copy_ctor)(void* dst, const void* src) = nullptr;
  void (*dtor)(void* obj)                       = nullptr;
  // to_json: obj(構築済みのTインスタンス)をJSON文字列へ変換する。
  std::string (*to_json)(const void* obj) = nullptr;
  // from_json: 未構築(malloc直後)のobjへ、jのパース結果を元にTを
  // placement-newで構築する。呼び出し側は既存オブジェクトの破棄を行わないため、
  // 実装は必ず `new(obj) T(...)` の形でobjに新規構築すること(objへの
  // 代入ではない点に注意)。
  bool (*from_json)(void* obj, const std::string& j) = nullptr;
};

class CustomTypeRegistry {
public:
  static CustomTypeRegistry& instance() {
    static CustomTypeRegistry inst;
    return inst;
  }

  void register_type(const std::string& name, const CustomTypeOps& ops) { types_.put(name, ops); }

  [[nodiscard]] const CustomTypeOps* find(const std::string& name) const {
    auto it = types_.find(name);
    if(it == types_.end()) return nullptr;
    return &it->second;
  }

private:
  cutil::dictionary<CustomTypeOps> types_;
};

// T の size/align/copy_ctor/dtor を自動導出してレジストリに登録する。
// to_json/from_json が必要な場合(Phase 6のJSONフォールバック対応)は、
// CustomTypeRegistry::instance().register_type() を直接呼んでCustomTypeOpsの
// 該当フィールドを埋めること。
template <typename T> void register_custom_type(const std::string& type_name) {
  CustomTypeOps ops;
  ops.size      = sizeof(T);
  ops.align     = alignof(T);
  ops.copy_ctor = [](void* dst, const void* src) { new(dst) T(*reinterpret_cast<const T*>(src)); };
  ops.dtor      = [](void* obj) { reinterpret_cast<T*>(obj)->~T(); };
  CustomTypeRegistry::instance().register_type(type_name, ops);
}

// CustomSlot: 型消去された1スロット。type_name をキーに CustomTypeRegistry から
// コピー構築/破棄の関数を引いて、任意の登録済み型のインスタンスをheap上に保持する。
// sizeof(CustomSlot)自体は保持する型に依らず固定なので、Prop::data_内では常に
// この固定サイズのオブジェクトとして扱える。
struct CustomSlot {
  char type_name[32] = {};
  void* ptr          = nullptr;

  CustomSlot() = default;
  ~CustomSlot() { destroy(); }

  CustomSlot(const CustomSlot& other) { copy_from(other); }

  CustomSlot(CustomSlot&& other) noexcept : ptr(other.ptr) {
    std::memcpy(type_name, other.type_name, sizeof(type_name));
    other.ptr = nullptr;
  }

  CustomSlot& operator=(const CustomSlot& other) {
    if(this != &other) {
      destroy();
      copy_from(other);
    }
    return *this;
  }

  CustomSlot& operator=(CustomSlot&& other) noexcept {
    if(this != &other) {
      destroy();
      std::memcpy(type_name, other.type_name, sizeof(type_name));
      ptr       = other.ptr;
      other.ptr = nullptr;
    }
    return *this;
  }

  template <typename T> static CustomSlot make(const char* type_name_, const T& value) {
    CustomSlot slot;
    std::strncpy(slot.type_name, type_name_, sizeof(slot.type_name) - 1);
    const CustomTypeOps* ops = CustomTypeRegistry::instance().find(slot.type_name);
    if(!ops) throw std::logic_error(std::string("CustomSlot: type not registered: ") + type_name_);
    slot.ptr = std::malloc(ops->size);
    if(!slot.ptr) throw std::bad_alloc();
    ops->copy_ctor(slot.ptr, &value);
    return slot;
  }

  // to_jsonでダンプされたJSON文字列から、CustomTypeOps::from_json経由で
  // インスタンスを再構築する(Phase 6のJSONフォールバックで使用)。
  static CustomSlot make_from_json(const char* type_name_, const std::string& json_text) {
    CustomSlot slot;
    std::strncpy(slot.type_name, type_name_, sizeof(slot.type_name) - 1);
    const CustomTypeOps* ops = CustomTypeRegistry::instance().find(slot.type_name);
    if(!ops) throw std::logic_error(std::string("CustomSlot: type not registered: ") + type_name_);
    if(!ops->from_json) throw std::logic_error(std::string("CustomSlot: type has no from_json registered: ") + type_name_);
    slot.ptr = std::malloc(ops->size);
    if(!slot.ptr) throw std::bad_alloc();
    if(!ops->from_json(slot.ptr, json_text)) {
      std::free(slot.ptr);
      slot.ptr = nullptr;
      throw std::logic_error(std::string("CustomSlot: from_json failed for type: ") + type_name_);
    }
    return slot;
  }

  [[nodiscard]] std::string to_json() const {
    const CustomTypeOps* ops = CustomTypeRegistry::instance().find(type_name);
    if(!ops || !ops->to_json) throw std::logic_error(std::string("CustomSlot: type has no to_json registered: ") + type_name);
    return ops->to_json(ptr);
  }

  template <typename T> [[nodiscard]] T& get() { return *reinterpret_cast<T*>(ptr); }
  template <typename T> [[nodiscard]] const T& get() const { return *reinterpret_cast<const T*>(ptr); }

private:
  void destroy() {
    if(ptr) {
      const CustomTypeOps* ops = CustomTypeRegistry::instance().find(type_name);
      if(ops && ops->dtor) ops->dtor(ptr);
      std::free(ptr);
      ptr = nullptr;
    }
  }

  void copy_from(const CustomSlot& other) {
    std::memcpy(type_name, other.type_name, sizeof(type_name));
    if(other.ptr) {
      const CustomTypeOps* ops = CustomTypeRegistry::instance().find(type_name);
      if(!ops) throw std::logic_error(std::string("CustomSlot: type not registered: ") + type_name);
      ptr = std::malloc(ops->size);
      if(!ptr) throw std::bad_alloc();
      ops->copy_ctor(ptr, other.ptr);
    } else {
      ptr = nullptr;
    }
  }
};

} // namespace cutil
