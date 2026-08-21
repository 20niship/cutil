#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <cutil/dictionary.hpp>
#include <cutil/path.hpp>
#include <cutil/quaternion.hpp>
#include <cutil/rect.hpp>
#include <cutil/rect3d.hpp>
#include <cutil/ref.hpp>
#include <cutil/string.hpp>
#include <cutil/vec.hpp>
#include <cutil/vector.hpp>

// 非テンプレート実装本体はCUTIL_IMPLEMENTATIONを定義した唯一のTUでのみ実体化する(テンプレートはC++の制約上ヘッダに残す)。JSONは外部ライブラリ非依存。

namespace cutil {

// 参考用(旧設計、現在は未使用enum)。PropInfo::Field::type(const PropInfo*)による自己記述に置き換えた。
enum class PropType : uint16_t {
  Bool,
  Int,
  Float,
  Str,
  Path,
  Vec3,
  Vec4,
  Quat,
  Range,
  Rect,
  Rect3D,
  Binary,
  Nested,
  Custom,
  Ref,
  RefList,
};

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
void register_prop_info_auto(const PropInfo* info); // 定義はCUTIL_IMPLEMENTATIONブロック。
}

// prop_info_of<T>()呼び出し時(実質初回のみ)にPropInfoRegistryへも自動登録する。register_prop_type<T>(name)は別名登録用に残す。
template <typename T> inline const PropInfo* prop_info_of() {
  const PropInfo* info    = PropInfoOf<T>::get();
  static bool registered  = (detail::register_prop_info_auto(info), true);
  (void)registered;
  return info;
}

// ---- 軽量JSON文字列ヘルパー(PropInfo::to_json/from_jsonがstd::string契約のためDOM構造を経由しない) ----
namespace detail {

inline std::string json_trim(const std::string& s) {
  size_t b = s.find_first_not_of(" \t\n\r");
  if(b == std::string::npos) return "";
  size_t e = s.find_last_not_of(" \t\n\r");
  return s.substr(b, e - b + 1);
}

inline std::string json_number(double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

inline std::string json_quote(const std::string& s) {
  std::string out = "\"";
  for(unsigned char c : s) {
    switch(c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if(c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += static_cast<char>(c);
        }
    }
  }
  out += "\"";
  return out;
}

// "..." 形式(前後の空白は除去済み想定)の文字列リテラルをデコードする。
inline std::string json_unquote(const std::string& s) {
  std::string out;
  if(s.size() < 2 || s.front() != '"' || s.back() != '"') return out;
  for(size_t i = 1; i + 1 < s.size(); i++) {
    char c = s[i];
    if(c == '\\' && i + 2 < s.size()) {
      i++;
      switch(s[i]) {
        case '"': out += '"'; break;
        case '\\': out += '\\'; break;
        case '/': out += '/'; break;
        case 'n': out += '\n'; break;
        case 'r': out += '\r'; break;
        case 't': out += '\t'; break;
        case 'b': out += '\b'; break;
        case 'f': out += '\f'; break;
        case 'u':
          if(i + 4 < s.size()) {
            int code = std::stoi(s.substr(i + 1, 4), nullptr, 16); // BMP内ASCII相当のみを想定した簡易デコード
            out += static_cast<char>(code);
            i += 4;
          }
          break;
        default: out += s[i];
      }
    } else {
      out += c;
    }
  }
  return out;
}

// "[...]"または"{...}"のトップレベル要素(ネスト/文字列内のカンマは無視)をカンマで分割する。
inline std::vector<std::string> json_split_top_level(const std::string& s) {
  std::vector<std::string> result;
  std::string trimmed = json_trim(s);
  if(trimmed.size() < 2) return result;
  std::string body = trimmed.substr(1, trimmed.size() - 2);
  size_t n          = body.size();
  size_t start       = 0;
  int depth          = 0;
  bool in_string     = false;
  for(size_t i = 0; i < n; i++) {
    char c = body[i];
    if(in_string) {
      if(c == '\\') {
        i++;
        continue;
      }
      if(c == '"') in_string = false;
    } else {
      if(c == '"') in_string = true;
      else if(c == '[' || c == '{') depth++;
      else if(c == ']' || c == '}') depth--;
      else if(c == ',' && depth == 0) {
        result.push_back(json_trim(body.substr(start, i - start)));
        start = i + 1;
      }
    }
  }
  std::string last = json_trim(body.substr(start));
  if(!last.empty()) result.push_back(last);
  return result;
}

// "key":value 形式の1要素からkeyとvalueを取り出す(json_split_top_levelの各要素に対して使う)。
inline bool json_split_kv(const std::string& s, std::string& key, std::string& value) {
  bool in_string = false;
  for(size_t i = 0; i < s.size(); i++) {
    char c = s[i];
    if(in_string) {
      if(c == '\\') {
        i++;
        continue;
      }
      if(c == '"') in_string = false;
    } else {
      if(c == '"') in_string = true;
      else if(c == ':') {
        key   = json_unquote(json_trim(s.substr(0, i)));
        value = json_trim(s.substr(i + 1));
        return true;
      }
    }
  }
  return false;
}

inline std::string json_array_of_floats(const float* data, size_t n) {
  std::string out = "[";
  for(size_t i = 0; i < n; i++) {
    if(i) out += ",";
    out += json_number(static_cast<double>(data[i]));
  }
  out += "]";
  return out;
}
inline void json_floats_from_array(const std::string& text, float* out, size_t n) {
  auto parts = json_split_top_level(text);
  for(size_t i = 0; i < n && i < parts.size(); i++) out[i] = static_cast<float>(std::stod(parts[i]));
}

// element_type/seq_*アクセサだけを頼りに任意コンテナ型をJSON化する共通実装(uiVector<T>/std::vector<T>が共有する)。
inline std::string generic_container_to_json(const PropInfo* type, const void* obj) {
  size_t n = type->seq_size(obj);
  std::string out = "[";
  for(size_t i = 0; i < n; i++) {
    if(i) out += ",";
    if(type->seq_data) {
      const auto* base = reinterpret_cast<const uint8_t*>(type->seq_data(obj));
      out += type->element_type->to_json(base + i * type->element_type->size);
    } else {
      out += type->element_type->to_json(type->seq_at(obj, i));
    }
  }
  out += "]";
  return out;
}

inline bool generic_container_from_json(const PropInfo* type, void* obj, const std::string& text) {
  auto parts = json_split_top_level(text);

  if(type->seq_assign_raw) {
    std::vector<uint8_t> buf(parts.size() * type->element_type->size);
    for(size_t i = 0; i < parts.size(); i++) {
      if(!type->element_type->from_json(buf.data() + i * type->element_type->size, parts[i])) return false;
    }
    if(type->default_ctor) type->default_ctor(obj);
    type->seq_assign_raw(obj, buf.data(), parts.size());
  } else {
    if(type->default_ctor) type->default_ctor(obj);
    for(const auto& part : parts) {
      std::vector<uint8_t> storage(type->element_type->size);
      if(!type->element_type->from_json(storage.data(), part)) return false; // from_json自体がplacement-newで構築する規約
      type->seq_push_back_copy(obj, storage.data());
      if(type->element_type->dtor) type->element_type->dtor(storage.data());
    }
  }
  return true;
}

// Vec3f/Vec4fのような固定長float配列(data[N])を持つTrivial型の共通実装。
template <typename VecT, size_t N> const PropInfo* make_vec_propinfo(const char* id) {
  static const PropInfo info = [id] {
    PropInfo p;
    p.set_id(id);
    p.klass = PropKlass::Trivial;
    p.size  = sizeof(VecT);
    p.align = alignof(VecT);
    p.to_json = [](const void* obj) -> std::string { return json_array_of_floats(reinterpret_cast<const VecT*>(obj)->data, N); };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      json_floats_from_array(text, reinterpret_cast<VecT*>(obj)->data, N);
      return true;
    };
    return p;
  }();
  return &info;
}

// offsetofルール構造体の完全な型登録が使う共通実装: fieldsを辿ってcopy_ctor/dtor/default_ctor/to_json/from_jsonを汎用化する。
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
  std::string out = "{";
  bool first        = true;
  for(const auto& f : self->fields) {
    if(!first) out += ",";
    first              = false;
    const auto* fptr = reinterpret_cast<const uint8_t*>(obj) + f.offset;
    out += json_quote(f.name) + ":" + f.type->to_json(fptr);
  }
  out += "}";
  return out;
}
inline bool generic_struct_from_json(const PropInfo* self, void* obj, const std::string& text) {
  if(self->default_ctor) self->default_ctor(obj);
  else std::memset(obj, 0, self->size);
  for(const auto& part : json_split_top_level(text)) {
    std::string key, value;
    if(!json_split_kv(part, key, value)) continue;
    const PropInfo::Field* f = self->find_field(key.c_str());
    if(!f) continue;
    auto* fptr = reinterpret_cast<uint8_t*>(obj) + f->offset;
    if(f->type->klass != PropKlass::Trivial && f->type->dtor) f->type->dtor(fptr); // default_ctorが構築した仮の値を破棄してから再構築する
    if(!f->type->from_json(fptr, value)) return false;
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

// ---- 組み込みleaf型のPropInfoOf特殊化(宣言のみ、実装本体はCUTIL_IMPLEMENTATIONブロック) ----
template <> struct PropInfoOf<bool> { static const PropInfo* get(); };
template <> struct PropInfoOf<int32_t> { static const PropInfo* get(); };
template <> struct PropInfoOf<float> { static const PropInfo* get(); };
template <> struct PropInfoOf<uint8_t> { static const PropInfo* get(); };
template <> struct PropInfoOf<Vec3f> { static const PropInfo* get(); };
template <> struct PropInfoOf<Vec4f> { static const PropInfo* get(); };
template <> struct PropInfoOf<Quat<float>> { static const PropInfo* get(); };
template <> struct PropInfoOf<Range> { static const PropInfo* get(); };
template <> struct PropInfoOf<Rect> { static const PropInfo* get(); };
template <> struct PropInfoOf<Rect3D> { static const PropInfo* get(); };
template <> struct PropInfoOf<Str> { static const PropInfo* get(); };
template <> struct PropInfoOf<Path> { static const PropInfo* get(); };
template <> struct PropInfoOf<std::vector<uint8_t>> { static const PropInfo* get(); };

// uiVector<T>はmalloc/memcpyベースで要素移動コンストラクタを呼ばないため要素はTrivial限定、B-in-Bにはstd::vector<T>を使うこと。
template <typename T> struct PropInfoOf<uiVector<T>> {
  static const PropInfo* get() {
    static_assert(std::is_trivially_copyable_v<T>, "uiVector<T>: T must be trivially copyable (use std::vector<T> for non-trivial T)");
    static PropInfo info;
    static bool initialized = false;
    if(!initialized) {
      info.set_id("uiVector<T>");
      info.klass         = PropKlass::Indirect;
      info.size          = sizeof(uiVector<T>);
      info.align         = alignof(uiVector<T>);
      info.copy_ctor     = [](void* dst, const void* src) { new(dst) uiVector<T>(*reinterpret_cast<const uiVector<T>*>(src)); };
      info.dtor          = [](void* obj) { reinterpret_cast<uiVector<T>*>(obj)->~uiVector<T>(); };
      info.default_ctor  = [](void* obj) { new(obj) uiVector<T>(); };
      info.element_type  = prop_info_of<T>();
      info.seq_size      = [](const void* obj) -> size_t { return static_cast<size_t>(reinterpret_cast<const uiVector<T>*>(obj)->size()); };
      info.seq_data      = [](const void* obj) -> const void* { return reinterpret_cast<const void*>(reinterpret_cast<const uiVector<T>*>(obj)->begin()); };
      info.seq_assign_raw = [](void* obj, const void* src, size_t n) {
        auto* v = reinterpret_cast<uiVector<T>*>(obj);
        v->resize(static_cast<int>(n));
        if(n) std::memcpy(v->data(), src, n * sizeof(T));
      };
      info.to_json   = [](const void* obj) -> std::string { return detail::generic_container_to_json(&info, obj); };
      info.from_json = [](void* obj, const std::string& text) -> bool { return detail::generic_container_from_json(&info, obj, text); };
      initialized     = true;
    }
    return &info;
  }
};

// std::vector<T>は要素がIndirect(非trivial)でも安全に扱える汎用コンテナ(B-in-Bに使う)。要素Trivialなら一括memcpy、Indirectなら要素ごと再帰する。
template <typename T> struct PropInfoOf<std::vector<T>> {
  static const PropInfo* get() {
    static PropInfo info;
    static bool initialized = false;
    if(!initialized) {
      info.set_id("std::vector<T>");
      info.klass        = PropKlass::Indirect;
      info.size         = sizeof(std::vector<T>);
      info.align        = alignof(std::vector<T>);
      info.copy_ctor    = [](void* dst, const void* src) { new(dst) std::vector<T>(*reinterpret_cast<const std::vector<T>*>(src)); };
      info.dtor         = [](void* obj) { reinterpret_cast<std::vector<T>*>(obj)->~vector(); };
      info.default_ctor = [](void* obj) { new(obj) std::vector<T>(); };
      info.element_type = prop_info_of<T>();
      info.seq_size     = [](const void* obj) -> size_t { return reinterpret_cast<const std::vector<T>*>(obj)->size(); };
      if constexpr(std::is_trivially_copyable_v<T>) {
        info.seq_data       = [](const void* obj) -> const void* { return reinterpret_cast<const void*>(reinterpret_cast<const std::vector<T>*>(obj)->data()); };
        info.seq_assign_raw = [](void* obj, const void* src, size_t n) {
          auto* v = reinterpret_cast<std::vector<T>*>(obj);
          v->resize(n);
          if(n) std::memcpy(v->data(), src, n * sizeof(T));
        };
      } else {
        info.seq_at             = [](const void* obj, size_t i) -> const void* { return reinterpret_cast<const void*>(&(*reinterpret_cast<const std::vector<T>*>(obj))[i]); };
        info.seq_push_back_copy = [](void* obj, const void* elem) { reinterpret_cast<std::vector<T>*>(obj)->push_back(*reinterpret_cast<const T*>(elem)); };
      }
      info.to_json   = [](const void* obj) -> std::string { return detail::generic_container_to_json(&info, obj); };
      info.from_json = [](void* obj, const std::string& text) -> bool { return detail::generic_container_from_json(&info, obj, text); };
      initialized     = true;
    }
    return &info;
  }
};

// ---- Ref/RefList用のダミーPropInfo(次段階までの一時的な表現、宣言のみ) ----
const PropInfo* prop_info_of_ref_slot();
const PropInfo* prop_info_of_ref_list_slot();

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

// PropInfoRegistry: 型名文字列 -> const PropInfo*。std::type_indexはビルド間で安定せずファイル永続化キーに使えないため文字列をキーにする。
class PropInfoRegistry {
public:
  static PropInfoRegistry& instance();
  void register_type(const std::string& name, const PropInfo* info);
  [[nodiscard]] const PropInfo* find(const std::string& name) const;

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

// CustomSlot: Dynamic(C)型の値を型消去して1個保持するラッパー。const PropInfo*を直接持つため旧来のtype_name文字列での毎回のレジストリ再検索が不要(高速化もする)。
struct CustomSlot {
  const PropInfo* info = nullptr;
  void* ptr             = nullptr;

  CustomSlot() = default;
  ~CustomSlot();

  CustomSlot(const CustomSlot& other);
  CustomSlot(CustomSlot&& other) noexcept : info(other.info), ptr(other.ptr) { other.ptr = nullptr; }

  CustomSlot& operator=(const CustomSlot& other);
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

  static CustomSlot make_from_json(const PropInfo* type_info, const std::string& json_text);
  [[nodiscard]] std::string to_json() const;

  template <typename T> [[nodiscard]] T& get() { return *reinterpret_cast<T*>(ptr); }
  template <typename T> [[nodiscard]] const T& get() const { return *reinterpret_cast<const T*>(ptr); }

private:
  void destroy();
  void copy_from(const CustomSlot& other);
};

template <> struct PropInfoOf<CustomSlot> { static const PropInfo* get(); };

// Prop: フィールドの型情報はPropInfoへ完全委譲し、Prop自身は型ごとのswitchを持たない。
class Prop {
public:
  Prop() = default;

  Prop(const Prop& other) : data_(other.data_.size()), fields_(other.fields_) {
    for(const auto& f : fields_) {
      const uint8_t* src = other.data_.data() + f.offset;
      uint8_t* dst        = data_.data() + f.offset;
      if(f.type->klass == PropKlass::Trivial) {
        std::memcpy(dst, src, f.type->size);
      } else {
        f.type->copy_ctor(dst, src);
      }
    }
  }

  Prop(Prop&& other) noexcept : data_(std::move(other.data_)), fields_(std::move(other.fields_)) {}

  Prop& operator=(const Prop& other) {
    if(this != &other) {
      Prop tmp(other);
      swap(tmp);
    }
    return *this;
  }

  Prop& operator=(Prop&& other) noexcept {
    if(this != &other) {
      destroy_all_fields();
      data_   = std::move(other.data_);
      fields_ = std::move(other.fields_);
    }
    return *this;
  }

  ~Prop() { destroy_all_fields(); }

  void swap(Prop& other) noexcept {
    data_.swap(other.data_);
    fields_.swap(other.fields_);
  }

  template <typename T> void set(const char* name, const T& value, const char* label = nullptr, const char* desc = nullptr) {
    const PropInfo* type = prop_info_of<T>();
    PropInfo::Field* f    = find_field(name);
    bool is_new           = false;
    if(!f) {
      f = &add_field(name, type);
      if(label) f->set_label(label);
      if(desc) f->set_desc(desc);
      is_new = true;
    } else if(f->type != type) {
      throw std::logic_error(std::string("Prop::set: field '") + name + "' already exists with a different type");
    }

    uint8_t* dst = data_.data() + f->offset;
    if(type->klass == PropKlass::Trivial) {
      std::memcpy(dst, &value, sizeof(T));
    } else {
      if(!is_new && type->dtor) type->dtor(dst); // 既存フィールドは一旦破棄してから再構築する(assign用の別関数は持たない)
      type->copy_ctor(dst, &value);
    }
  }

  template <typename T> T& get(const char* name) {
    PropInfo::Field* f = find_field(name);
    if(!f) throw std::out_of_range(std::string("Prop: field not found: ") + name);
    if(f->type != prop_info_of<T>()) throw std::logic_error(std::string("Prop::get: field '") + name + "' has a different type");
    return *reinterpret_cast<T*>(data_.data() + f->offset);
  }

  template <typename T> const T& get(const char* name) const {
    const PropInfo::Field* f = find_field(name);
    if(!f) throw std::out_of_range(std::string("Prop: field not found: ") + name);
    if(f->type != prop_info_of<T>()) throw std::logic_error(std::string("Prop::get: field '") + name + "' has a different type");
    return *reinterpret_cast<const T*>(data_.data() + f->offset);
  }

  [[nodiscard]] bool contains(const char* name) const { return find_field(name) != nullptr; }

  bool erase(const char* name) {
    for(auto it = fields_.begin(); it != fields_.end(); ++it) {
      if(std::strncmp(it->name, name, sizeof(it->name)) == 0) {
        if(it->type->klass != PropKlass::Trivial && it->type->dtor) it->type->dtor(data_.data() + it->offset);
        fields_.erase(it);
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] const std::vector<PropInfo::Field>& fields() const { return fields_; }
  [[nodiscard]] size_t field_count() const { return fields_.size(); }

  bool set_field_version(const char* name, uint32_t) {
    // PropInfo::versionは型スキーマ単位に統合され、フィールド単位のversion書き換えは廃止した。
    return find_field(name) != nullptr;
  }

  [[nodiscard]] const uint8_t* raw_data() const { return data_.data(); }
  [[nodiscard]] size_t raw_size() const { return data_.size(); }

  void set_child(const char* name, const Prop& child) { set<Prop>(name, child); }
  Prop& get_child(const char* name) { return get<Prop>(name); }
  [[nodiscard]] const Prop& get_child(const char* name) const { return get<Prop>(name); }

  // dump: 外部構造体からruleに従い値を読み取りProp自身へセットする。Ref/RefListはフックで生ポインタ(群)を取り出す。
  bool dump(const void* data, const PropInfo* rule) {
    const auto* base = reinterpret_cast<const uint8_t*>(data);
    for(const auto& rf : rule->fields) {
      if(rf.ref_extract) {
        void* raw_ptr = rf.ref_extract(base + rf.offset);
        set_raw(rf.name, prop_info_of_ref_slot(), &raw_ptr);
        continue;
      }
      if(rf.ref_list_extract) {
        std::vector<void*> tmp;
        rf.ref_list_extract(base + rf.offset, tmp);
        set_raw(rf.name, prop_info_of_ref_list_slot(), &tmp);
        continue;
      }
      set_raw(rf.name, rf.type, base + rf.offset);
    }
    return true;
  }

  // load_to: Prop自身の値をruleに従い構造体(data、構築済み前提)へ書き込む。該当フィールドが無い/型不一致ならスキップしfalseを返す。
  [[nodiscard]] bool load_to(void* data, const PropInfo* rule) const {
    auto* base    = reinterpret_cast<uint8_t*>(data);
    bool complete = true;
    for(const auto& rf : rule->fields) {
      if(rf.ref_extract) { // Ref
        const PropInfo::Field* f = find_field(rf.name);
        if(!f) {
          complete = false;
          continue;
        }
        void* raw_ptr = nullptr;
        std::memcpy(&raw_ptr, data_.data() + f->offset, sizeof(void*));
        if(rf.ref_assign) rf.ref_assign(base + rf.offset, raw_ptr);
        continue;
      }
      if(rf.ref_list_extract) { // RefList
        const PropInfo::Field* f = find_field(rf.name);
        if(!f) {
          complete = false;
          continue;
        }
        const auto* raw_list = reinterpret_cast<const std::vector<void*>*>(data_.data() + f->offset);
        if(rf.ref_list_assign) rf.ref_list_assign(base + rf.offset, *raw_list);
        continue;
      }

      const PropInfo::Field* f = find_field(rf.name);
      if(!f || f->type != rf.type) {
        complete = false;
        continue;
      }
      const uint8_t* src = data_.data() + f->offset;
      uint8_t* dst        = base + rf.offset;
      if(rf.type->klass == PropKlass::Trivial) {
        std::memcpy(dst, src, rf.type->size);
      } else {
        // dataは既に構築済みという呼び出し規約のため、既存オブジェクトを破棄してから再構築する
        if(rf.type->dtor) rf.type->dtor(dst);
        rf.type->copy_ctor(dst, src);
      }
    }
    return complete;
  }

  // Dynamic(C)型のフィールドをセットする。type_nameでPropInfoRegistryを引きCustomSlotとして格納する。
  template <typename T> void set_custom(const char* name, const char* type_name, const T& value, const char* label = nullptr, const char* desc = nullptr) {
    const PropInfo* type_info = PropInfoRegistry::instance().find(type_name);
    if(!type_info) throw std::logic_error(std::string("Prop::set_custom: type not registered: ") + type_name);
    CustomSlot slot = CustomSlot::make<T>(type_info, value);
    set(name, slot, label, desc);
  }

  template <typename T> [[nodiscard]] T& get_custom(const char* name) { return get<CustomSlot>(name).get<T>(); }
  template <typename T> [[nodiscard]] const T& get_custom(const char* name) const { return get<CustomSlot>(name).get<T>(); }

  // 既に構築済みのCustomSlotを名前付きフィールドとして取り込む(所有権を移動、prop_io.hppのJSON経由loadが使う)。
  void adopt_custom_slot(const char* name, CustomSlot&& slot) { set<CustomSlot>(name, slot); }

  // prop_io.hpp向け実行時PropInfoベースの低レベルAPI(型がバイナリ/JSONのtype_idからしか分からない場面用)。
  void set_raw_pod_by_info(const char* name, const PropInfo* type, const void* bytes) {
    PropInfo::Field* f = find_field(name);
    if(!f) {
      f = &add_field(name, type);
    } else if(f->type != type) {
      throw std::logic_error(std::string("Prop::set_raw_pod_by_info: field '") + name + "' already exists with a different type");
    }
    std::memcpy(data_.data() + f->offset, bytes, type->size);
  }

  void adopt_raw_by_info(const char* name, const PropInfo* type, void* constructed_src) {
    PropInfo::Field* f = find_field(name);
    bool is_new         = false;
    if(!f) {
      f      = &add_field(name, type);
      is_new = true;
    } else if(f->type != type) {
      throw std::logic_error(std::string("Prop::adopt_raw_by_info: field '") + name + "' already exists with a different type");
    }
    uint8_t* dst = data_.data() + f->offset;
    if(!is_new && type->dtor) type->dtor(dst);
    type->copy_ctor(dst, constructed_src);
    if(type->dtor) type->dtor(constructed_src);
  }

private:
  std::vector<uint8_t> data_;
  std::vector<PropInfo::Field> fields_;

  [[nodiscard]] PropInfo::Field* find_field(const char* name) {
    for(auto& f : fields_) {
      if(std::strncmp(f.name, name, sizeof(f.name)) == 0) return &f;
    }
    return nullptr;
  }
  [[nodiscard]] const PropInfo::Field* find_field(const char* name) const {
    for(const auto& f : fields_) {
      if(std::strncmp(f.name, name, sizeof(f.name)) == 0) return &f;
    }
    return nullptr;
  }

  PropInfo::Field& add_field(const char* name, const PropInfo* type) {
    size_t new_offset = align_up(data_.size(), type->align);
    data_.resize(new_offset + type->size, 0);
    fields_.emplace_back(name, new_offset, type);
    return fields_.back();
  }

  // dump()用の実行時PropInfoに基づく汎用set(set<T>()と異なりコンパイル時型情報を使わない)。
  void set_raw(const char* name, const PropInfo* type, const void* src_ptr) {
    PropInfo::Field* f = find_field(name);
    bool is_new        = false;
    if(!f) {
      f      = &add_field(name, type);
      is_new = true;
    } else if(f->type != type) {
      throw std::logic_error(std::string("Prop::dump: field '") + name + "' already exists with a different type");
    }

    uint8_t* dst = data_.data() + f->offset;
    if(type->klass == PropKlass::Trivial) {
      std::memcpy(dst, src_ptr, type->size);
    } else {
      if(!is_new && type->dtor) type->dtor(dst);
      type->copy_ctor(dst, src_ptr);
    }
  }

  void destroy_all_fields() {
    for(const auto& f : fields_) {
      if(f.type->klass != PropKlass::Trivial && f.type->dtor) f.type->dtor(data_.data() + f.offset);
    }
  }
};

template <> struct PropInfoOf<Prop> { static const PropInfo* get(); };

} // namespace cutil

#ifdef CUTIL_IMPLEMENTATION

namespace cutil {

const PropInfo* PropInfoOf<bool>::get() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("bool");
    p.klass   = PropKlass::Trivial;
    p.size    = sizeof(bool);
    p.align   = alignof(bool);
    p.to_json = [](const void* obj) -> std::string { return *reinterpret_cast<const bool*>(obj) ? "true" : "false"; };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      bool v = detail::json_trim(text) == "true";
      std::memcpy(obj, &v, sizeof(bool));
      return true;
    };
    return p;
  }();
  return &info;
}

const PropInfo* PropInfoOf<int32_t>::get() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("int32_t");
    p.klass   = PropKlass::Trivial;
    p.size    = sizeof(int32_t);
    p.align   = alignof(int32_t);
    p.to_json = [](const void* obj) -> std::string { return std::to_string(*reinterpret_cast<const int32_t*>(obj)); };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      int32_t v = static_cast<int32_t>(std::stoll(detail::json_trim(text)));
      std::memcpy(obj, &v, sizeof(int32_t));
      return true;
    };
    return p;
  }();
  return &info;
}

const PropInfo* PropInfoOf<float>::get() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("float");
    p.klass   = PropKlass::Trivial;
    p.size    = sizeof(float);
    p.align   = alignof(float);
    p.to_json = [](const void* obj) -> std::string { return detail::json_number(static_cast<double>(*reinterpret_cast<const float*>(obj))); };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      float v = static_cast<float>(std::stod(detail::json_trim(text)));
      std::memcpy(obj, &v, sizeof(float));
      return true;
    };
    return p;
  }();
  return &info;
}

const PropInfo* PropInfoOf<uint8_t>::get() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("uint8_t");
    p.klass   = PropKlass::Trivial;
    p.size    = sizeof(uint8_t);
    p.align   = alignof(uint8_t);
    p.to_json = [](const void* obj) -> std::string { return std::to_string(*reinterpret_cast<const uint8_t*>(obj)); };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      uint8_t v = static_cast<uint8_t>(std::stoi(detail::json_trim(text)));
      std::memcpy(obj, &v, sizeof(uint8_t));
      return true;
    };
    return p;
  }();
  return &info;
}

const PropInfo* PropInfoOf<Vec3f>::get() { return detail::make_vec_propinfo<Vec3f, 3>("Vec3f"); }
const PropInfo* PropInfoOf<Vec4f>::get() { return detail::make_vec_propinfo<Vec4f, 4>("Vec4f"); }

const PropInfo* PropInfoOf<Quat<float>>::get() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("Quat<float>");
    p.klass = PropKlass::Trivial;
    p.size  = sizeof(Quat<float>);
    p.align = alignof(Quat<float>);
    p.to_json = [](const void* obj) -> std::string {
      const auto& v = *reinterpret_cast<const Quat<float>*>(obj);
      float f[4]     = {v.x, v.y, v.z, v.w};
      return detail::json_array_of_floats(f, 4);
    };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      float f[4];
      detail::json_floats_from_array(text, f, 4);
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

const PropInfo* PropInfoOf<Range>::get() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("Range");
    p.klass = PropKlass::Trivial;
    p.size  = sizeof(Range);
    p.align = alignof(Range);
    p.to_json = [](const void* obj) -> std::string {
      const auto& v = *reinterpret_cast<const Range*>(obj);
      float f[2]     = {v.min, v.max};
      return detail::json_array_of_floats(f, 2);
    };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      float f[2];
      detail::json_floats_from_array(text, f, 2);
      auto* v = reinterpret_cast<Range*>(obj);
      v->min  = f[0];
      v->max  = f[1];
      return true;
    };
    return p;
  }();
  return &info;
}

const PropInfo* PropInfoOf<Rect>::get() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("Rect");
    p.klass = PropKlass::Trivial;
    p.size  = sizeof(Rect);
    p.align = alignof(Rect);
    p.to_json = [](const void* obj) -> std::string {
      const auto& v = *reinterpret_cast<const Rect*>(obj);
      float f[4]     = {v.x.min, v.x.max, v.y.min, v.y.max};
      return detail::json_array_of_floats(f, 4);
    };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      float f[4];
      detail::json_floats_from_array(text, f, 4);
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

const PropInfo* PropInfoOf<Rect3D>::get() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("Rect3D");
    p.klass = PropKlass::Trivial;
    p.size  = sizeof(Rect3D);
    p.align = alignof(Rect3D);
    p.to_json = [](const void* obj) -> std::string {
      const auto& v = *reinterpret_cast<const Rect3D*>(obj);
      float f[6]     = {v.x.min, v.x.max, v.y.min, v.y.max, v.z.min, v.z.max};
      return detail::json_array_of_floats(f, 6);
    };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      float f[6];
      detail::json_floats_from_array(text, f, 6);
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

namespace detail {
struct StrTraits {
  using T = Str;
  static const char* c_str(const T& v) { return v.c_str(); }
  static size_t size(const T& v) { return v.size(); }
  static void assign(T& v, const char* s, size_t n) { v = Str(s, n); }
};
struct PathTraits {
  using T = Path;
  static const char* c_str(const T& v) { return v.str().c_str(); }
  static size_t size(const T& v) { return v.str().size(); }
  static void assign(T& v, const char* s, size_t n) { v = Path(Str(s, n)); }
};

template <typename Traits> const PropInfo* make_string_like_propinfo(const char* id) {
  using T = typename Traits::T;
  static const PropInfo info = [id] {
    PropInfo p;
    p.set_id(id);
    p.klass         = PropKlass::Indirect;
    p.size          = sizeof(T);
    p.align         = alignof(T);
    p.copy_ctor     = [](void* dst, const void* src) { new(dst) T(*reinterpret_cast<const T*>(src)); };
    p.dtor          = [](void* obj) { reinterpret_cast<T*>(obj)->~T(); };
    p.default_ctor  = [](void* obj) { new(obj) T(); };
    p.element_type  = prop_info_of<uint8_t>(); // 文字列本体をuint8_tのコンテナとして表現し一括memcpy経路に乗せる
    p.seq_size      = [](const void* obj) -> size_t { return Traits::size(*reinterpret_cast<const T*>(obj)); };
    p.seq_data      = [](const void* obj) -> const void* { return reinterpret_cast<const void*>(Traits::c_str(*reinterpret_cast<const T*>(obj))); };
    p.seq_assign_raw = [](void* obj, const void* src, size_t n) { Traits::assign(*reinterpret_cast<T*>(obj), reinterpret_cast<const char*>(src), n); };
    p.to_json       = [](const void* obj) -> std::string { return json_quote(Traits::c_str(*reinterpret_cast<const T*>(obj))); };
    p.from_json     = [](void* obj, const std::string& text) -> bool {
      std::string s = json_unquote(json_trim(text));
      new(obj) T();
      Traits::assign(*reinterpret_cast<T*>(obj), s.data(), s.size());
      return true;
    };
    return p;
  }();
  return &info;
}
} // namespace detail

const PropInfo* PropInfoOf<Str>::get() { return detail::make_string_like_propinfo<detail::StrTraits>("Str"); }
const PropInfo* PropInfoOf<Path>::get() { return detail::make_string_like_propinfo<detail::PathTraits>("Path"); }

const PropInfo* PropInfoOf<std::vector<uint8_t>>::get() {
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
    p.to_json = [](const void* obj) -> std::string {
      const auto* v   = reinterpret_cast<const std::vector<uint8_t>*>(obj);
      std::string out = "[";
      for(size_t i = 0; i < v->size(); i++) {
        if(i) out += ",";
        out += std::to_string((*v)[i]);
      }
      out += "]";
      return out;
    };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      auto* v = new(obj) std::vector<uint8_t>();
      for(const auto& part : detail::json_split_top_level(text)) v->push_back(static_cast<uint8_t>(std::stoi(part)));
      return true;
    };
    return p;
  }();
  return &info;
}

const PropInfo* prop_info_of_ref_slot() {
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

const PropInfo* prop_info_of_ref_list_slot() {
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

PropInfoRegistry& PropInfoRegistry::instance() {
  static PropInfoRegistry inst;
  return inst;
}
void PropInfoRegistry::register_type(const std::string& name, const PropInfo* info) { types_.put(name, info); }
const PropInfo* PropInfoRegistry::find(const std::string& name) const {
  auto it = types_.find(name);
  if(it == types_.end()) return nullptr;
  return it->second;
}

namespace detail {
void register_prop_info_auto(const PropInfo* info) { PropInfoRegistry::instance().register_type(info->id, info); }
} // namespace detail

CustomSlot::~CustomSlot() { destroy(); }
CustomSlot::CustomSlot(const CustomSlot& other) { copy_from(other); }
CustomSlot& CustomSlot::operator=(const CustomSlot& other) {
  if(this != &other) {
    destroy();
    copy_from(other);
  }
  return *this;
}

CustomSlot CustomSlot::make_from_json(const PropInfo* type_info, const std::string& json_text) {
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

std::string CustomSlot::to_json() const {
  if(!info || !info->to_json) throw std::logic_error("CustomSlot: type has no to_json registered");
  return info->to_json(ptr);
}

void CustomSlot::destroy() {
  if(ptr) {
    if(info && info->dtor) info->dtor(ptr);
    std::free(ptr);
    ptr = nullptr;
  }
}

void CustomSlot::copy_from(const CustomSlot& other) {
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

const PropInfo* PropInfoOf<CustomSlot>::get() {
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
      const auto* slot = reinterpret_cast<const CustomSlot*>(obj);
      return "{" + detail::json_quote("custom_type") + ":" + detail::json_quote(slot->info ? slot->info->id : "") + "," + detail::json_quote("value") + ":" + slot->to_json() + "}";
    };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      std::string custom_type, value;
      for(const auto& part : detail::json_split_top_level(text)) {
        std::string k, v;
        if(!detail::json_split_kv(part, k, v)) continue;
        if(k == "custom_type") custom_type = detail::json_unquote(v);
        else if(k == "value") value = v;
      }
      const PropInfo* inner_type = PropInfoRegistry::instance().find(custom_type);
      if(!inner_type) return false;
      CustomSlot slot = CustomSlot::make_from_json(inner_type, value);
      new(obj) CustomSlot(std::move(slot));
      return true;
    };
    return p;
  }();
  return &info;
}

// prop_info_of<Prop>()のto_json/from_jsonはprop_io.hppがprop_dump_json/prop_load_json定義後にconst_castで遅延バインドする。
const PropInfo* PropInfoOf<Prop>::get() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("Prop");
    p.klass        = PropKlass::Indirect;
    p.size         = sizeof(Prop);
    p.align        = alignof(Prop);
    p.copy_ctor    = [](void* dst, const void* src) { new(dst) Prop(*reinterpret_cast<const Prop*>(src)); };
    p.dtor         = [](void* obj) { reinterpret_cast<Prop*>(obj)->~Prop(); };
    p.default_ctor = [](void* obj) { new(obj) Prop(); };
    return p;
  }();
  return &info;
}

} // namespace cutil

#endif // CUTIL_IMPLEMENTATION
