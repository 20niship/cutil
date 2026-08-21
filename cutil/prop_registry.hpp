#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <cutil/dictionary.hpp>
#include <cutil/json.hpp>
#include <cutil/path.hpp>
#include <cutil/quaternion.hpp>
#include <cutil/rect.hpp>
#include <cutil/rect3d.hpp>
#include <cutil/ref.hpp>
#include <cutil/string.hpp>
#include <cutil/vec.hpp>
#include <cutil/vector.hpp>

// cutil::PropInfo: 1つの型を完全に記述する自己再帰的ノード。旧PropInfo::Data(フィールドメタ)とCustomTypeOps(型消去操作テーブル)を統合し、Trivial/Indirect/Dynamicの3klassで表現する。

namespace cutil {

class Prop; // 前方宣言。prop_info_of<Prop>()の定義はprop.hpp側で行う。

enum class PropKlass : uint8_t {
  Trivial,
  Indirect,
  Dynamic,
};

enum class PropWidget {
  Auto,
  Drag,
  Slider,
  ColorPicker,
  FilePath,
  Checkbox,
  Combo,
};

enum class PropFlags : uint32_t {
  None       = 0,
  EditOnly   = 1 << 0,
  Hidden     = 1 << 1,
  ReadOnly   = 1 << 2,
  Animatable = 1 << 3,
};

inline PropFlags operator|(PropFlags a, PropFlags b) { return static_cast<PropFlags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b)); }
inline PropFlags operator&(PropFlags a, PropFlags b) { return static_cast<PropFlags>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b)); }
inline bool has_flag(PropFlags flags, PropFlags test) { return (static_cast<uint32_t>(flags) & static_cast<uint32_t>(test)) != 0; }

inline size_t align_up(size_t offset, size_t align) { return (offset + align - 1) & ~(align - 1); }

struct PropInfo {
  char id[32] = {}; // 型名。PropInfoRegistryのキーと一致させる
  PropKlass klass  = PropKlass::Trivial;
  size_t size      = 0;
  size_t align     = 1;
  uint32_t version = 1; // 型スキーマ全体のバージョン(per-fieldではなく型単位)

  // ライフサイクル(旧CustomTypeOpsを吸収)。Trivialはnullのままでよい。
  void (*copy_ctor)(void* dst, const void* src) = nullptr;
  void (*dtor)(void* obj)                       = nullptr;
  void (*default_ctor)(void* obj)               = nullptr;
  std::string (*to_json)(const void* obj)                = nullptr; // Dynamicは必須
  bool (*from_json)(void* obj, const std::string& j) = nullptr;

  // Struct表現: 自分が集約型の場合のフィールド一覧(offsetofルール)
  struct Field {
    char name[32]  = {};
    char label[64] = {};
    char desc[256] = {};
    PropWidget widget = PropWidget::Auto;
    PropFlags flags   = PropFlags::None;
    float min_value = 0, max_value = 0, drag_speed = 1.0f;

    size_t offset         = 0; // 親の中でのoffset(Prop::data_内 or 外部構造体内、文脈依存)
    const PropInfo* type = nullptr; // 旧PropType enumの代わり。組み込み/ユーザー型を同じ木で辿る

    // Ref/RefList用フック(次段階まで温存)。WeakPtr<T>/vector<Ref<T>>と生ポインタ(群)を相互変換する。
    using RefExtractFn     = void* (*)(const void* field_ptr);
    using RefAssignFn      = void (*)(void* field_ptr, void* raw_ptr);
    using RefListExtractFn = void (*)(const void* field_ptr, std::vector<void*>& out);
    using RefListAssignFn  = void (*)(void* field_ptr, const std::vector<void*>& in);
    RefExtractFn ref_extract          = nullptr;
    RefAssignFn ref_assign            = nullptr;
    RefListExtractFn ref_list_extract = nullptr;
    RefListAssignFn ref_list_assign   = nullptr;

    Field() = default;
    Field(const char* name_, size_t offset_, const PropInfo* type_) : offset(offset_), type(type_) { set_name(name_); }

    void set_name(const char* s) {
      std::strncpy(name, s, sizeof(name) - 1);
      name[sizeof(name) - 1] = '\0';
    }
    void set_label(const char* s) {
      std::strncpy(label, s, sizeof(label) - 1);
      label[sizeof(label) - 1] = '\0';
    }
    void set_desc(const char* s) {
      std::strncpy(desc, s, sizeof(desc) - 1);
      desc[sizeof(desc) - 1] = '\0';
    }

    // make_ref<T>/make_ref_list<T>の定義はprop_info_of_ref_slot()等の後(このファイル下部)。
    template <typename T> static Field make_ref(const char* name_, size_t offset_);
    template <typename T> static Field make_ref_list(const char* name_, size_t offset_);
  };
  std::vector<Field> fields; // 空 = leaf型 or 可変長コンテナ

  // Indirect(B)の可変長コンテナ用アクセサ(配列は最大1本まで、複数本必要ならDynamicとして扱う)。
  const PropInfo* element_type = nullptr;
  size_t (*seq_size)(const void* obj)                                     = nullptr;
  const void* (*seq_data)(const void* obj)                                = nullptr; // element Trivial限定、一括memcpy用
  void (*seq_assign_raw)(void* obj, const void* src, size_t n)            = nullptr; // element Trivial限定、一括memcpy用
  const void* (*seq_at)(const void* obj, size_t i)                        = nullptr; // element Indirect用、要素ごとアクセス
  void (*seq_push_back_copy)(void* obj, const void* elem)                 = nullptr; // element Indirect用、要素ごと再構築

  PropInfo()                            = default;
  PropInfo(const PropInfo&)             = default;
  PropInfo& operator=(const PropInfo&)  = default;
  PropInfo(PropInfo&&) noexcept         = default;
  PropInfo& operator=(PropInfo&&) noexcept = default;

  // {"name", offset, type}の集約初期化でoffsetofルールを書ける(klass/copy_ctorを使わない軽量ルール用)。
  PropInfo(std::initializer_list<Field> list) : fields(list) {}

  void set_id(const char* s) {
    std::strncpy(id, s, sizeof(id) - 1);
    id[sizeof(id) - 1] = '\0';
  }

  [[nodiscard]] const Field* find_field(const char* name) const {
    for(const auto& f : fields) {
      if(std::strncmp(f.name, name, sizeof(f.name)) == 0) return &f;
    }
    return nullptr;
  }
  [[nodiscard]] Field* find_field(const char* name) {
    for(auto& f : fields) {
      if(std::strncmp(f.name, name, sizeof(f.name)) == 0) return &f;
    }
    return nullptr;
  }
};

inline bool validate(const PropInfo::Field& f, float value) {
  if(f.min_value == 0 && f.max_value == 0) return true; // 範囲未設定は無条件許可
  return value >= f.min_value && value <= f.max_value;
}

// prop_info_of<T>(): 関数テンプレートは部分特殊化できないためPropInfoOf<T>::get()へ委譲する(uiVector<T>/std::vector<T>の汎用対応に必要)。
template <typename T> struct PropInfoOf; // 未特殊化はコンパイルエラー

class PropInfoRegistry; // 前方宣言(prop_info_of<T>()が自動登録に使う)。定義は本ファイル下部。
namespace detail {
inline void register_prop_info_auto(const PropInfo* info); // 定義はPropInfoRegistry定義後。
}

// prop_info_of<T>()呼び出し時(実質初回のみ)にPropInfoRegistryへも自動登録する。register_prop_type<T>(name)は別名登録用に残す。
template <typename T> inline const PropInfo* prop_info_of() {
  const PropInfo* info    = PropInfoOf<T>::get();
  static bool registered  = (detail::register_prop_info_auto(info), true);
  (void)registered;
  return info;
}

namespace detail {

inline json::Value floats_to_json_array(const float* data, size_t n) {
  json::Value arr = json::Value::make_array();
  for(size_t i = 0; i < n; i++) arr.push_back(json::Value::make_double(static_cast<double>(data[i])));
  return arr;
}
inline void json_array_to_floats(const json::Value& arr, float* out, size_t n) {
  for(size_t i = 0; i < n; i++) out[i] = static_cast<float>(arr.get(i).as_double());
}

} // namespace detail

// leaf Trivialスカラー型(bool/int32_t/float)。to_json/from_jsonは単一のjson値。
#define CUTIL_PROP_TRIVIAL_SCALAR(CPP_TYPE, MAKE_JSON_EXPR, FROM_JSON_EXPR)                \
  template <> struct PropInfoOf<CPP_TYPE> {                                                \
    static const PropInfo* get() {                                                         \
      static const PropInfo info = [] {                                                    \
        PropInfo p;                                                                         \
        p.set_id(#CPP_TYPE);                                                                \
        p.klass = PropKlass::Trivial;                                                       \
        p.size = sizeof(CPP_TYPE);                                                          \
        p.align = alignof(CPP_TYPE);                                                        \
        p.to_json = [](const void* obj) -> std::string {                                    \
          const CPP_TYPE& v = *reinterpret_cast<const CPP_TYPE*>(obj);                       \
          return (MAKE_JSON_EXPR).dump();                                                    \
        };                                                                                    \
        p.from_json = [](void* obj, const std::string& text) -> bool {                       \
          bool ok           = false;                                                          \
          json::Value value = json::Value::parse(text, &ok);                                  \
          if(!ok) return false;                                                                \
          CPP_TYPE v = (FROM_JSON_EXPR);                                                       \
          std::memcpy(obj, &v, sizeof(CPP_TYPE));                                              \
          return true;                                                                          \
        };                                                                                      \
        return p;                                                                                \
      }();                                                                                        \
      return &info;                                                                                \
    }                                                                                               \
  };

CUTIL_PROP_TRIVIAL_SCALAR(bool, json::Value::make_bool(v), value.as_bool())
CUTIL_PROP_TRIVIAL_SCALAR(int32_t, json::Value::make_int(v), static_cast<int32_t>(value.as_int()))
CUTIL_PROP_TRIVIAL_SCALAR(float, json::Value::make_double(static_cast<double>(v)), static_cast<float>(value.as_double()))
CUTIL_PROP_TRIVIAL_SCALAR(uint8_t, json::Value::make_int(v), static_cast<uint8_t>(value.as_int()))
#undef CUTIL_PROP_TRIVIAL_SCALAR

// leaf Trivial複合型(floatの配列表現)。マクロ化するとカンマ/波括弧を含む式が引数分割で壊れるため個別に書く。
template <> struct PropInfoOf<Vec3f> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("Vec3f");
      p.klass = PropKlass::Trivial;
      p.size  = sizeof(Vec3f);
      p.align = alignof(Vec3f);
      p.to_json = [](const void* obj) -> std::string {
        const auto& v = *reinterpret_cast<const Vec3f*>(obj);
        return detail::floats_to_json_array(v.data, 3).dump();
      };
      p.from_json = [](void* obj, const std::string& text) -> bool {
        bool ok           = false;
        json::Value value = json::Value::parse(text, &ok);
        if(!ok) return false;
        detail::json_array_to_floats(value, reinterpret_cast<Vec3f*>(obj)->data, 3);
        return true;
      };
      return p;
    }();
    return &info;
  }
};

template <> struct PropInfoOf<Vec4f> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("Vec4f");
      p.klass = PropKlass::Trivial;
      p.size  = sizeof(Vec4f);
      p.align = alignof(Vec4f);
      p.to_json = [](const void* obj) -> std::string {
        const auto& v = *reinterpret_cast<const Vec4f*>(obj);
        return detail::floats_to_json_array(v.data, 4).dump();
      };
      p.from_json = [](void* obj, const std::string& text) -> bool {
        bool ok           = false;
        json::Value value = json::Value::parse(text, &ok);
        if(!ok) return false;
        detail::json_array_to_floats(value, reinterpret_cast<Vec4f*>(obj)->data, 4);
        return true;
      };
      return p;
    }();
    return &info;
  }
};

template <> struct PropInfoOf<Quat<float>> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("Quat<float>");
      p.klass = PropKlass::Trivial;
      p.size  = sizeof(Quat<float>);
      p.align = alignof(Quat<float>);
      p.to_json = [](const void* obj) -> std::string {
        const auto& v = *reinterpret_cast<const Quat<float>*>(obj);
        float f[4]     = {v.x, v.y, v.z, v.w};
        return detail::floats_to_json_array(f, 4).dump();
      };
      p.from_json = [](void* obj, const std::string& text) -> bool {
        bool ok           = false;
        json::Value value = json::Value::parse(text, &ok);
        if(!ok) return false;
        float f[4];
        detail::json_array_to_floats(value, f, 4);
        auto* v = reinterpret_cast<Quat<float>*>(obj);
        v->x    = f[0];
        v->y    = f[1];
        v->z    = f[2];
        v->w    = f[3];
        return true;
      };
      return p;
    }();
    return &info;
  }
};

template <> struct PropInfoOf<Range> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("Range");
      p.klass = PropKlass::Trivial;
      p.size  = sizeof(Range);
      p.align = alignof(Range);
      p.to_json = [](const void* obj) -> std::string {
        const auto& v = *reinterpret_cast<const Range*>(obj);
        float f[2]     = {v.min, v.max};
        return detail::floats_to_json_array(f, 2).dump();
      };
      p.from_json = [](void* obj, const std::string& text) -> bool {
        bool ok           = false;
        json::Value value = json::Value::parse(text, &ok);
        if(!ok) return false;
        float f[2];
        detail::json_array_to_floats(value, f, 2);
        auto* v = reinterpret_cast<Range*>(obj);
        v->min  = f[0];
        v->max  = f[1];
        return true;
      };
      return p;
    }();
    return &info;
  }
};

template <> struct PropInfoOf<Rect> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("Rect");
      p.klass = PropKlass::Trivial;
      p.size  = sizeof(Rect);
      p.align = alignof(Rect);
      p.to_json = [](const void* obj) -> std::string {
        const auto& v = *reinterpret_cast<const Rect*>(obj);
        float f[4]     = {v.x.min, v.x.max, v.y.min, v.y.max};
        return detail::floats_to_json_array(f, 4).dump();
      };
      p.from_json = [](void* obj, const std::string& text) -> bool {
        bool ok           = false;
        json::Value value = json::Value::parse(text, &ok);
        if(!ok) return false;
        float f[4];
        detail::json_array_to_floats(value, f, 4);
        auto* v  = reinterpret_cast<Rect*>(obj);
        v->x.min = f[0];
        v->x.max = f[1];
        v->y.min = f[2];
        v->y.max = f[3];
        return true;
      };
      return p;
    }();
    return &info;
  }
};

template <> struct PropInfoOf<Rect3D> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("Rect3D");
      p.klass = PropKlass::Trivial;
      p.size  = sizeof(Rect3D);
      p.align = alignof(Rect3D);
      p.to_json = [](const void* obj) -> std::string {
        const auto& v = *reinterpret_cast<const Rect3D*>(obj);
        float f[6]     = {v.x.min, v.x.max, v.y.min, v.y.max, v.z.min, v.z.max};
        return detail::floats_to_json_array(f, 6).dump();
      };
      p.from_json = [](void* obj, const std::string& text) -> bool {
        bool ok           = false;
        json::Value value = json::Value::parse(text, &ok);
        if(!ok) return false;
        float f[6];
        detail::json_array_to_floats(value, f, 6);
        auto* v  = reinterpret_cast<Rect3D*>(obj);
        v->x.min = f[0];
        v->x.max = f[1];
        v->y.min = f[2];
        v->y.max = f[3];
        v->z.min = f[4];
        v->z.max = f[5];
        return true;
      };
      return p;
    }();
    return &info;
  }
};

// Str/PathはSSO実装差異を安全に吸収するため、uiVector<char>汎用テンプレートに便乗させず専用leaf Indirectとして個別実装する。
template <> struct PropInfoOf<Str> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("Str");
      p.klass       = PropKlass::Indirect;
      p.size        = sizeof(Str);
      p.align       = alignof(Str);
      p.copy_ctor   = [](void* dst, const void* src) { new(dst) Str(*reinterpret_cast<const Str*>(src)); };
      p.dtor        = [](void* obj) { reinterpret_cast<Str*>(obj)->~Str(); };
      p.default_ctor = [](void* obj) { new(obj) Str(); };
      // 文字列本体をuint8_tのコンテナとして表現する(write/read_value_binaryの一括memcpy経路に乗せるため)。
      p.element_type = prop_info_of<uint8_t>();
      p.seq_size     = [](const void* obj) -> size_t { return reinterpret_cast<const Str*>(obj)->size(); };
      p.seq_data     = [](const void* obj) -> const void* { return reinterpret_cast<const void*>(reinterpret_cast<const Str*>(obj)->c_str()); };
      p.seq_assign_raw = [](void* obj, const void* src, size_t n) { *reinterpret_cast<Str*>(obj) = Str(reinterpret_cast<const char*>(src), n); };
      p.to_json     = [](const void* obj) -> std::string { return json::Value::make_string(reinterpret_cast<const Str*>(obj)->c_str()).dump(); };
      p.from_json   = [](void* obj, const std::string& text) -> bool {
        bool ok           = false;
        json::Value value = json::Value::parse(text, &ok);
        if(!ok) return false;
        new(obj) Str(value.as_string().c_str());
        return true;
      };
      return p;
    }();
    return &info;
  }
};

template <> struct PropInfoOf<Path> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("Path");
      p.klass       = PropKlass::Indirect;
      p.size        = sizeof(Path);
      p.align       = alignof(Path);
      p.copy_ctor   = [](void* dst, const void* src) { new(dst) Path(*reinterpret_cast<const Path*>(src)); };
      p.dtor        = [](void* obj) { reinterpret_cast<Path*>(obj)->~Path(); };
      p.default_ctor = [](void* obj) { new(obj) Path(); };
      p.element_type = prop_info_of<uint8_t>();
      p.seq_size     = [](const void* obj) -> size_t { return reinterpret_cast<const Path*>(obj)->str().size(); };
      p.seq_data     = [](const void* obj) -> const void* { return reinterpret_cast<const void*>(reinterpret_cast<const Path*>(obj)->str().c_str()); };
      p.seq_assign_raw = [](void* obj, const void* src, size_t n) { *reinterpret_cast<Path*>(obj) = Path(Str(reinterpret_cast<const char*>(src), n)); };
      p.to_json     = [](const void* obj) -> std::string { return json::Value::make_string(reinterpret_cast<const Path*>(obj)->str().c_str()).dump(); };
      p.from_json   = [](void* obj, const std::string& text) -> bool {
        bool ok           = false;
        json::Value value = json::Value::parse(text, &ok);
        if(!ok) return false;
        new(obj) Path(Str(value.as_string().c_str()));
        return true;
      };
      return p;
    }();
    return &info;
  }
};

template <> struct PropInfoOf<std::vector<uint8_t>> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("Binary");
      p.klass       = PropKlass::Indirect;
      p.size        = sizeof(std::vector<uint8_t>);
      p.align       = alignof(std::vector<uint8_t>);
      p.copy_ctor   = [](void* dst, const void* src) { new(dst) std::vector<uint8_t>(*reinterpret_cast<const std::vector<uint8_t>*>(src)); };
      p.dtor        = [](void* obj) { reinterpret_cast<std::vector<uint8_t>*>(obj)->~vector(); };
      p.default_ctor = [](void* obj) { new(obj) std::vector<uint8_t>(); };
      p.element_type = prop_info_of<uint8_t>();
      p.seq_size     = [](const void* obj) -> size_t { return reinterpret_cast<const std::vector<uint8_t>*>(obj)->size(); };
      p.seq_data     = [](const void* obj) -> const void* { return reinterpret_cast<const void*>(reinterpret_cast<const std::vector<uint8_t>*>(obj)->data()); };
      p.seq_assign_raw = [](void* obj, const void* src, size_t n) {
        auto* v = reinterpret_cast<std::vector<uint8_t>*>(obj);
        v->resize(n);
        if(n) std::memcpy(v->data(), src, n);
      };
      p.to_json     = [](const void* obj) -> std::string {
        const auto* v   = reinterpret_cast<const std::vector<uint8_t>*>(obj);
        json::Value arr = json::Value::make_array();
        for(uint8_t b : *v) arr.push_back(json::Value::make_int(b));
        return arr.dump();
      };
      p.from_json = [](void* obj, const std::string& text) -> bool {
        bool ok           = false;
        json::Value value = json::Value::parse(text, &ok);
        if(!ok) return false;
        auto* v = new(obj) std::vector<uint8_t>();
        v->reserve(value.size());
        for(size_t i = 0; i < value.size(); i++) v->push_back(static_cast<uint8_t>(value.get(i).as_int()));
        return true;
      };
      return p;
    }();
    return &info;
  }
};

// uiVector<T>はmalloc/memcpyベースで要素移動コンストラクタを呼ばないため要素はTrivial限定、B-in-Bにはstd::vector<T>を使うこと。
template <typename T> struct PropInfoOf<uiVector<T>> {
  static const PropInfo* get() {
    static_assert(std::is_trivially_copyable_v<T>, "uiVector<T>: T must be trivially copyable (use std::vector<T> for non-trivial T)");
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("uiVector<T>");
      p.klass         = PropKlass::Indirect;
      p.size          = sizeof(uiVector<T>);
      p.align         = alignof(uiVector<T>);
      p.copy_ctor     = [](void* dst, const void* src) { new(dst) uiVector<T>(*reinterpret_cast<const uiVector<T>*>(src)); };
      p.dtor          = [](void* obj) { reinterpret_cast<uiVector<T>*>(obj)->~uiVector<T>(); };
      p.default_ctor  = [](void* obj) { new(obj) uiVector<T>(); };
      p.element_type  = prop_info_of<T>();
      p.seq_size      = [](const void* obj) -> size_t { return static_cast<size_t>(reinterpret_cast<const uiVector<T>*>(obj)->size()); };
      p.seq_data      = [](const void* obj) -> const void* { return reinterpret_cast<const void*>(reinterpret_cast<const uiVector<T>*>(obj)->begin()); };
      p.seq_assign_raw = [](void* obj, const void* src, size_t n) {
        auto* v = reinterpret_cast<uiVector<T>*>(obj);
        v->resize(static_cast<int>(n));
        if(n) std::memcpy(v->data(), src, n * sizeof(T));
      };
      p.to_json = [](const void* obj) -> std::string {
        const auto* v   = reinterpret_cast<const uiVector<T>*>(obj);
        json::Value arr = json::Value::make_array();
        for(int i = 0; i < v->size(); i++) {
          bool ok;
          arr.push_back(json::Value::parse(prop_info_of<T>()->to_json(v->begin() + i), &ok));
        }
        return arr.dump();
      };
      p.from_json = [](void* obj, const std::string& text) -> bool {
        bool ok           = false;
        json::Value value = json::Value::parse(text, &ok);
        if(!ok) return false;
        auto* v = new(obj) uiVector<T>();
        v->resize(static_cast<int>(value.size()));
        for(size_t i = 0; i < value.size(); i++) {
          bool elem_ok;
          T elem;
          json::Value ev = value.get(i);
          if(!prop_info_of<T>()->from_json(&elem, ev.dump())) return (void)(elem_ok = false), false;
          (*v)[static_cast<int>(i)] = elem;
        }
        return true;
      };
      return p;
    }();
    return &info;
  }
};

// std::vector<T>は要素がIndirect(非trivial)でも安全に扱える汎用コンテナ(B-in-Bに使う)。要素Trivialなら一括memcpy、Indirectなら要素ごと再帰する。
template <typename T> struct PropInfoOf<std::vector<T>> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("std::vector<T>");
      p.klass        = PropKlass::Indirect;
      p.size         = sizeof(std::vector<T>);
      p.align        = alignof(std::vector<T>);
      p.copy_ctor    = [](void* dst, const void* src) { new(dst) std::vector<T>(*reinterpret_cast<const std::vector<T>*>(src)); };
      p.dtor         = [](void* obj) { reinterpret_cast<std::vector<T>*>(obj)->~vector(); };
      p.default_ctor = [](void* obj) { new(obj) std::vector<T>(); };
      p.element_type = prop_info_of<T>();
      p.seq_size     = [](const void* obj) -> size_t { return reinterpret_cast<const std::vector<T>*>(obj)->size(); };
      if constexpr(std::is_trivially_copyable_v<T>) {
        p.seq_data       = [](const void* obj) -> const void* { return reinterpret_cast<const void*>(reinterpret_cast<const std::vector<T>*>(obj)->data()); };
        p.seq_assign_raw = [](void* obj, const void* src, size_t n) {
          auto* v = reinterpret_cast<std::vector<T>*>(obj);
          v->resize(n);
          if(n) std::memcpy(v->data(), src, n * sizeof(T));
        };
      } else {
        p.seq_at             = [](const void* obj, size_t i) -> const void* { return reinterpret_cast<const void*>(&(*reinterpret_cast<const std::vector<T>*>(obj))[i]); };
        p.seq_push_back_copy = [](void* obj, const void* elem) { reinterpret_cast<std::vector<T>*>(obj)->push_back(*reinterpret_cast<const T*>(elem)); };
      }
      p.to_json = [](const void* obj) -> std::string {
        const auto* v   = reinterpret_cast<const std::vector<T>*>(obj);
        json::Value arr = json::Value::make_array();
        for(const auto& elem : *v) {
          bool ok;
          arr.push_back(json::Value::parse(prop_info_of<T>()->to_json(&elem), &ok));
        }
        return arr.dump();
      };
      p.from_json = [](void* obj, const std::string& text) -> bool {
        bool ok           = false;
        json::Value value = json::Value::parse(text, &ok);
        if(!ok) return false;
        auto* v = new(obj) std::vector<T>();
        v->reserve(value.size());
        for(size_t i = 0; i < value.size(); i++) {
          json::Value ev = value.get(i);
          std::vector<uint8_t> storage(prop_info_of<T>()->size);
          if(prop_info_of<T>()->default_ctor) prop_info_of<T>()->default_ctor(storage.data());
          if(!prop_info_of<T>()->from_json(storage.data(), ev.dump())) return false;
          v->push_back(*reinterpret_cast<T*>(storage.data()));
          if(prop_info_of<T>()->dtor) prop_info_of<T>()->dtor(storage.data());
        }
        return true;
      };
      return p;
    }();
    return &info;
  }
};

// ---- Ref/RefList用のダミーPropInfo(次段階までの一時的な表現) ----

inline const PropInfo* prop_info_of_ref_slot() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("Ref");
    p.klass = PropKlass::Trivial;
    p.size  = sizeof(void*);
    p.align = alignof(void*);
    return p;
  }();
  return &info;
}

inline const PropInfo* prop_info_of_ref_list_slot() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("RefList");
    p.klass        = PropKlass::Indirect;
    p.size         = sizeof(std::vector<void*>);
    p.align        = alignof(std::vector<void*>);
    p.copy_ctor    = [](void* dst, const void* src) { new(dst) std::vector<void*>(*reinterpret_cast<const std::vector<void*>*>(src)); };
    p.dtor         = [](void* obj) { reinterpret_cast<std::vector<void*>*>(obj)->~vector(); };
    p.default_ctor = [](void* obj) { new(obj) std::vector<void*>(); };
    return p;
  }();
  return &info;
}

template <typename T> inline PropInfo::Field PropInfo::Field::make_ref(const char* name_, size_t offset_) {
  Field f(name_, offset_, prop_info_of_ref_slot());
  f.ref_extract = [](const void* field_ptr) -> void* {
    const auto* wp = reinterpret_cast<const WeakPtr<T>*>(field_ptr);
    return const_cast<void*>(static_cast<const void*>(wp->lock().get()));
  };
  f.ref_assign = [](void* field_ptr, void* raw_ptr) {
    auto* wp = reinterpret_cast<WeakPtr<T>*>(field_ptr);
    if(raw_ptr) {
      *wp = reinterpret_cast<T*>(raw_ptr)->weak_from_this();
    } else {
      wp->reset();
    }
  };
  return f;
}

template <typename T> inline PropInfo::Field PropInfo::Field::make_ref_list(const char* name_, size_t offset_) {
  Field f(name_, offset_, prop_info_of_ref_list_slot());
  f.ref_list_extract = [](const void* field_ptr, std::vector<void*>& out) {
    const auto* vec = reinterpret_cast<const std::vector<Ref<T>>*>(field_ptr);
    out.clear();
    out.reserve(vec->size());
    for(const auto& r : *vec) out.push_back(static_cast<void*>(r.get()));
  };
  f.ref_list_assign = [](void* field_ptr, const std::vector<void*>& in) {
    auto* vec = reinterpret_cast<std::vector<Ref<T>>*>(field_ptr);
    vec->clear();
    vec->reserve(in.size());
    for(void* raw_ptr : in) {
      if(raw_ptr) vec->push_back(reinterpret_cast<T*>(raw_ptr)->ref_from_this());
    }
  };
  return f;
}

// offsetofルール構造体の完全な型登録: fieldsを辿るcopy_ctor/dtor/default_ctorを汎用実装する。static変数はキャプチャなしラムダから直接参照できるC++の規則を利用し関数ポインタのまま自己参照する。
namespace detail {

inline void generic_struct_copy_ctor(const PropInfo* self, void* dst, const void* src) {
  std::memcpy(dst, src, self->size); // 浅いコピー(POD部分はこれで完結)
  for(const auto& f : self->fields) {
    if(f.type->klass != PropKlass::Trivial && f.type->copy_ctor) {
      f.type->copy_ctor(reinterpret_cast<uint8_t*>(dst) + f.offset, reinterpret_cast<const uint8_t*>(src) + f.offset);
    }
  }
}
inline void generic_struct_dtor(const PropInfo* self, void* obj) {
  for(const auto& f : self->fields) {
    if(f.type->klass != PropKlass::Trivial && f.type->dtor) {
      f.type->dtor(reinterpret_cast<uint8_t*>(obj) + f.offset);
    }
  }
}
inline void generic_struct_default_ctor(const PropInfo* self, void* obj) {
  std::memset(obj, 0, self->size);
  for(const auto& f : self->fields) {
    if(f.type->klass != PropKlass::Trivial && f.type->default_ctor) {
      f.type->default_ctor(reinterpret_cast<uint8_t*>(obj) + f.offset);
    }
  }
}
inline std::string generic_struct_to_json(const PropInfo* self, const void* obj) {
  json::Value root = json::Value::make_object();
  for(const auto& f : self->fields) {
    const auto* fptr = reinterpret_cast<const uint8_t*>(obj) + f.offset;
    bool ok;
    root.set(f.name, json::Value::parse(f.type->to_json(fptr), &ok));
  }
  return root.dump();
}
inline bool generic_struct_from_json(const PropInfo* self, void* obj, const std::string& text) {
  bool ok           = false;
  json::Value root = json::Value::parse(text, &ok);
  if(!ok || !root.is_object()) return false;
  if(self->default_ctor) self->default_ctor(obj);
  else std::memset(obj, 0, self->size);
  for(const auto& f : self->fields) {
    if(!root.contains(f.name)) continue;
    auto* fptr = reinterpret_cast<uint8_t*>(obj) + f.offset;
    if(f.type->klass != PropKlass::Trivial && f.type->dtor) f.type->dtor(fptr); // default_ctorが構築した仮の値を破棄してから再構築する
    if(!f.type->from_json(fptr, root.get(f.name).dump())) return false;
  }
  return true;
}

} // namespace detail

// T用の完全なPropInfoを型ごとに1回だけ構築する。klassはis_trivially_copyable_v<T>から自動判定する。
template <typename T> const PropInfo* register_struct_type(const char* name, std::initializer_list<PropInfo::Field> field_list) {
  static PropInfo info;
  static bool initialized = false;
  if(!initialized) {
    info.set_id(name);
    info.size  = sizeof(T);
    info.align = alignof(T);
    info.klass = std::is_trivially_copyable_v<T> ? PropKlass::Trivial : PropKlass::Indirect;
    info.fields.assign(field_list);
    if(info.klass != PropKlass::Trivial) {
      info.copy_ctor    = [](void* dst, const void* src) { detail::generic_struct_copy_ctor(&info, dst, src); };
      info.dtor          = [](void* obj) { detail::generic_struct_dtor(&info, obj); };
      info.default_ctor = [](void* obj) { detail::generic_struct_default_ctor(&info, obj); };
    }
    info.to_json   = [](const void* obj) -> std::string { return detail::generic_struct_to_json(&info, obj); };
    info.from_json = [](void* obj, const std::string& text) -> bool { return detail::generic_struct_from_json(&info, obj, text); };
    initialized = true;
  }
  return &info;
}

// PropInfoRegistry: 型名文字列 -> const PropInfo*。std::type_indexはビルド間で安定せずファイル永続化キーに使えないため文字列をキーにする(旧CustomTypeRegistryの後継)。
class PropInfoRegistry {
public:
  static PropInfoRegistry& instance() {
    static PropInfoRegistry inst;
    return inst;
  }

  void register_type(const std::string& name, const PropInfo* info) { types_.put(name, info); }

  [[nodiscard]] const PropInfo* find(const std::string& name) const {
    auto it = types_.find(name);
    if(it == types_.end()) return nullptr;
    return it->second;
  }

private:
  cutil::dictionary<const PropInfo*> types_;
};

template <typename T> void register_prop_type(const std::string& name) { PropInfoRegistry::instance().register_type(name, prop_info_of<T>()); }

// Dynamic(C)型のsize/align/copy_ctor/dtorを自動導出して登録する(旧register_custom_type<T>の後継、to_json/from_json省略時はJSON化不可)。
template <typename T> const PropInfo* register_dynamic_type(const std::string& name, std::string (*to_json)(const void*) = nullptr, bool (*from_json)(void*, const std::string&) = nullptr) {
  static PropInfo info;
  static bool initialized = false;
  if(!initialized) {
    info.set_id(name.c_str());
    info.klass     = PropKlass::Dynamic;
    info.size      = sizeof(T);
    info.align     = alignof(T);
    info.copy_ctor = [](void* dst, const void* src) { new(dst) T(*reinterpret_cast<const T*>(src)); };
    info.dtor       = [](void* obj) { reinterpret_cast<T*>(obj)->~T(); };
    info.to_json   = to_json;
    info.from_json = from_json;
    initialized     = true;
  }
  PropInfoRegistry::instance().register_type(name, &info);
  return &info;
}

namespace detail {
inline void register_prop_info_auto(const PropInfo* info) { PropInfoRegistry::instance().register_type(info->id, info); }
} // namespace detail

// CustomSlot: Dynamic(C)型の値を型消去して1個保持するラッパー。const PropInfo*を直接持つため旧来のtype_name文字列での毎回のレジストリ再検索が不要(高速化もする)。
struct CustomSlot {
  const PropInfo* info = nullptr;
  void* ptr             = nullptr;

  CustomSlot() = default;
  ~CustomSlot() { destroy(); }

  CustomSlot(const CustomSlot& other) { copy_from(other); }
  CustomSlot(CustomSlot&& other) noexcept : info(other.info), ptr(other.ptr) { other.ptr = nullptr; }

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
      info      = other.info;
      ptr       = other.ptr;
      other.ptr = nullptr;
    }
    return *this;
  }

  template <typename T> static CustomSlot make(const std::string& type_name, const T& value) {
    const PropInfo* type_info = PropInfoRegistry::instance().find(type_name);
    if(!type_info) throw std::logic_error(std::string("CustomSlot: type not registered: ") + type_name);
    return make<T>(type_info, value);
  }

  template <typename T> static CustomSlot make(const PropInfo* type_info, const T& value) {
    CustomSlot slot;
    slot.info = type_info;
    slot.ptr  = std::malloc(type_info->size);
    if(!slot.ptr) throw std::bad_alloc();
    if(type_info->copy_ctor) {
      type_info->copy_ctor(slot.ptr, &value);
    } else {
      std::memcpy(slot.ptr, &value, type_info->size);
    }
    return slot;
  }

  static CustomSlot make_from_json(const PropInfo* type_info, const std::string& json_text) {
    CustomSlot slot;
    slot.info = type_info;
    if(!type_info->from_json) throw std::logic_error(std::string("CustomSlot: type has no from_json registered: ") + type_info->id);
    slot.ptr = std::malloc(type_info->size);
    if(!slot.ptr) throw std::bad_alloc();
    if(!type_info->from_json(slot.ptr, json_text)) {
      std::free(slot.ptr);
      slot.ptr = nullptr;
      throw std::logic_error(std::string("CustomSlot: from_json failed for type: ") + type_info->id);
    }
    return slot;
  }

  [[nodiscard]] std::string to_json() const {
    if(!info || !info->to_json) throw std::logic_error("CustomSlot: type has no to_json registered");
    return info->to_json(ptr);
  }

  template <typename T> [[nodiscard]] T& get() { return *reinterpret_cast<T*>(ptr); }
  template <typename T> [[nodiscard]] const T& get() const { return *reinterpret_cast<const T*>(ptr); }

private:
  void destroy() {
    if(ptr) {
      if(info && info->dtor) info->dtor(ptr);
      std::free(ptr);
      ptr = nullptr;
    }
  }
  void copy_from(const CustomSlot& other) {
    info = other.info;
    if(other.ptr) {
      if(!info) throw std::logic_error("CustomSlot: copy from slot with null info");
      ptr = std::malloc(info->size);
      if(!ptr) throw std::bad_alloc();
      if(info->copy_ctor) {
        info->copy_ctor(ptr, other.ptr);
      } else {
        std::memcpy(ptr, other.ptr, info->size);
      }
    } else {
      ptr = nullptr;
    }
  }
};

template <> struct PropInfoOf<CustomSlot> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("CustomSlot");
      p.klass        = PropKlass::Dynamic;
      p.size         = sizeof(CustomSlot);
      p.align        = alignof(CustomSlot);
      p.copy_ctor    = [](void* dst, const void* src) { new(dst) CustomSlot(*reinterpret_cast<const CustomSlot*>(src)); };
      p.dtor         = [](void* obj) { reinterpret_cast<CustomSlot*>(obj)->~CustomSlot(); };
      p.default_ctor = [](void* obj) { new(obj) CustomSlot(); };
      // CustomSlot自身のto_json()は中身の値のjsonのみを返すため、中身の実際の型名(復元に必要)も併せて包む。
      p.to_json = [](const void* obj) -> std::string {
        const auto* slot   = reinterpret_cast<const CustomSlot*>(obj);
        json::Value wrapper = json::Value::make_object();
        wrapper.set("custom_type", json::Value::make_string(slot->info ? slot->info->id : ""));
        bool ok;
        wrapper.set("value", json::Value::parse(slot->to_json(), &ok));
        return wrapper.dump();
      };
      p.from_json = [](void* obj, const std::string& text) -> bool {
        bool ok           = false;
        json::Value root = json::Value::parse(text, &ok);
        if(!ok || !root.is_object()) return false;
        std::string custom_type    = root.get("custom_type").as_string();
        const PropInfo* inner_type = PropInfoRegistry::instance().find(custom_type);
        if(!inner_type) return false;
        CustomSlot slot = CustomSlot::make_from_json(inner_type, root.get("value").dump());
        new(obj) CustomSlot(std::move(slot));
        return true;
      };
      return p;
    }();
    return &info;
  }
};

} // namespace cutil
