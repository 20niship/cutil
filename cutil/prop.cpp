#include <cutil/prop.hpp>

namespace cutil {

const PropInfo* PropInfoOf<bool>::get() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("bool");
    p.klass     = PropClass::Trivial;
    p.size      = sizeof(bool);
    p.align     = alignof(bool);
    p.to_json   = [](const void* obj) -> std::string { return *reinterpret_cast<const bool*>(obj) ? "true" : "false"; };
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
    p.klass     = PropClass::Trivial;
    p.size      = sizeof(int32_t);
    p.align     = alignof(int32_t);
    p.to_json   = [](const void* obj) -> std::string { return std::to_string(*reinterpret_cast<const int32_t*>(obj)); };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      try {
        int32_t v = static_cast<int32_t>(std::stoll(detail::json_trim(text)));
        std::memcpy(obj, &v, sizeof(int32_t));
        return true;
      } catch(const std::exception& e) {
        CUTIL_PRINTF("[cutil::PropInfoOf<int32_t>::from_json] '%s' is not a number: %s\n", text.c_str(), e.what());
        return false;
      }
    };
    return p;
  }();
  return &info;
}

const PropInfo* PropInfoOf<float>::get() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("float");
    p.klass     = PropClass::Trivial;
    p.size      = sizeof(float);
    p.align     = alignof(float);
    p.to_json   = [](const void* obj) -> std::string { return detail::json_number(static_cast<double>(*reinterpret_cast<const float*>(obj))); };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      try {
        float v = static_cast<float>(std::stod(detail::json_trim(text)));
        std::memcpy(obj, &v, sizeof(float));
        return true;
      } catch(const std::exception& e) {
        CUTIL_PRINTF("[cutil::PropInfoOf<float>::from_json] '%s' is not a number: %s\n", text.c_str(), e.what());
        return false;
      }
    };
    return p;
  }();
  return &info;
}

const PropInfo* PropInfoOf<uint8_t>::get() {
  static const PropInfo info = [] {
    PropInfo p;
    p.set_id("uint8_t");
    p.klass     = PropClass::Trivial;
    p.size      = sizeof(uint8_t);
    p.align     = alignof(uint8_t);
    p.to_json   = [](const void* obj) -> std::string { return std::to_string(*reinterpret_cast<const uint8_t*>(obj)); };
    p.from_json = [](void* obj, const std::string& text) -> bool {
      try {
        uint8_t v = static_cast<uint8_t>(std::stoi(detail::json_trim(text)));
        std::memcpy(obj, &v, sizeof(uint8_t));
        return true;
      } catch(const std::exception& e) {
        CUTIL_PRINTF("[cutil::PropInfoOf<uint8_t>::from_json] '%s' is not a number: %s\n", text.c_str(), e.what());
        return false;
      }
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
    p.klass   = PropClass::Trivial;
    p.size    = sizeof(Quat<float>);
    p.align   = alignof(Quat<float>);
    p.to_json = [](const void* obj) -> std::string {
      const auto& v = *reinterpret_cast<const Quat<float>*>(obj);
      float f[4]    = {v.x, v.y, v.z, v.w};
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
    p.klass   = PropClass::Trivial;
    p.size    = sizeof(Range);
    p.align   = alignof(Range);
    p.to_json = [](const void* obj) -> std::string {
      const auto& v = *reinterpret_cast<const Range*>(obj);
      float f[2]    = {v.min, v.max};
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
    p.klass   = PropClass::Trivial;
    p.size    = sizeof(Rect);
    p.align   = alignof(Rect);
    p.to_json = [](const void* obj) -> std::string {
      const auto& v = *reinterpret_cast<const Rect*>(obj);
      float f[4]    = {v.x.min, v.x.max, v.y.min, v.y.max};
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
    p.klass   = PropClass::Trivial;
    p.size    = sizeof(Rect3D);
    p.align   = alignof(Rect3D);
    p.to_json = [](const void* obj) -> std::string {
      const auto& v = *reinterpret_cast<const Rect3D*>(obj);
      float f[6]    = {v.x.min, v.x.max, v.y.min, v.y.max, v.z.min, v.z.max};
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
  using T                    = typename Traits::T;
  static const PropInfo info = [id] {
    PropInfo p;
    p.set_id(id);
    p.klass          = PropClass::Indirect;
    p.size           = sizeof(T);
    p.align          = alignof(T);
    p.copy_ctor      = [](void* dst, const void* src) { new(dst) T(*reinterpret_cast<const T*>(src)); };
    p.dtor           = [](void* obj) { reinterpret_cast<T*>(obj)->~T(); };
    p.default_ctor   = [](void* obj) { new(obj) T(); };
    p.element_type   = prop_info_of<uint8_t>(); // 文字列本体をuint8_tのコンテナとして表現し一括memcpy経路に乗せる
    p.seq_size       = [](const void* obj) -> size_t { return Traits::size(*reinterpret_cast<const T*>(obj)); };
    p.seq_data       = [](const void* obj) -> const void* { return reinterpret_cast<const void*>(Traits::c_str(*reinterpret_cast<const T*>(obj))); };
    p.seq_assign_raw = [](void* obj, const void* src, size_t n) { Traits::assign(*reinterpret_cast<T*>(obj), reinterpret_cast<const char*>(src), n); };
    p.to_json        = [](const void* obj) -> std::string { return json_quote(Traits::c_str(*reinterpret_cast<const T*>(obj))); };
    p.from_json      = [](void* obj, const std::string& text) -> bool {
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
    p.klass          = PropClass::Indirect;
    p.size           = sizeof(std::vector<uint8_t>);
    p.align          = alignof(std::vector<uint8_t>);
    p.copy_ctor      = [](void* dst, const void* src) { new(dst) std::vector<uint8_t>(*reinterpret_cast<const std::vector<uint8_t>*>(src)); };
    p.dtor           = [](void* obj) { reinterpret_cast<std::vector<uint8_t>*>(obj)->~vector(); };
    p.default_ctor   = [](void* obj) { new(obj) std::vector<uint8_t>(); };
    p.element_type   = prop_info_of<uint8_t>();
    p.seq_size       = [](const void* obj) -> size_t { return reinterpret_cast<const std::vector<uint8_t>*>(obj)->size(); };
    p.seq_data       = [](const void* obj) -> const void* { return reinterpret_cast<const void*>(reinterpret_cast<const std::vector<uint8_t>*>(obj)->data()); };
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
      for(const auto& part : detail::json_split_top_level(text)) {
        try {
          v->push_back(static_cast<uint8_t>(std::stoi(part)));
        } catch(const std::exception& e) {
          CUTIL_PRINTF("[cutil::PropInfoOf<vector<uint8_t>>::from_json] '%s' is not a number: %s\n", part.c_str(), e.what());
          v->~vector(); // 既にplacement-newした分を破棄してから失敗を返す
          return false;
        }
      }
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
    p.klass = PropClass::Trivial;
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
    p.klass        = PropClass::Indirect;
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
    p.klass        = PropClass::Dynamic;
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
        if(k == "custom_type")
          custom_type = detail::json_unquote(v);
        else if(k == "value")
          value = v;
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
    p.klass        = PropClass::Indirect;
    p.size         = sizeof(Prop);
    p.align        = alignof(Prop);
    p.copy_ctor    = [](void* dst, const void* src) { new(dst) Prop(*reinterpret_cast<const Prop*>(src)); };
    p.dtor         = [](void* obj) { reinterpret_cast<Prop*>(obj)->~Prop(); };
    p.default_ctor = [](void* obj) { new(obj) Prop(); };
    return p;
  }();
  return &info;
}


namespace detail {

std::string json_trim(const std::string& s) {
  size_t b = s.find_first_not_of(" \t\n\r");
  if(b == std::string::npos) return "";
  size_t e = s.find_last_not_of(" \t\n\r");
  return s.substr(b, e - b + 1);
}

std::string json_number(double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

std::string json_quote(const std::string& s) {
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
std::string json_unquote(const std::string& s) {
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
            try {
              int code = std::stoi(s.substr(i + 1, 4), nullptr, 16); // BMP内ASCII相当のみを想定した簡易デコード
              out += static_cast<char>(code);
            } catch(const std::exception& e) {
              CUTIL_PRINTF("[cutil::json_unquote] invalid \\u escape '%s': %s\n", s.substr(i + 1, 4).c_str(), e.what());
            }
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
std::vector<std::string> json_split_top_level(const std::string& s) {
  std::vector<std::string> result;
  std::string trimmed = json_trim(s);
  if(trimmed.size() < 2) return result;
  std::string body = trimmed.substr(1, trimmed.size() - 2);
  size_t n         = body.size();
  size_t start     = 0;
  int depth        = 0;
  bool in_string   = false;
  for(size_t i = 0; i < n; i++) {
    char c = body[i];
    if(in_string) {
      if(c == '\\') {
        i++;
        continue;
      }
      if(c == '"') in_string = false;
    } else {
      if(c == '"')
        in_string = true;
      else if(c == '[' || c == '{')
        depth++;
      else if(c == ']' || c == '}')
        depth--;
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
bool json_split_kv(const std::string& s, std::string& key, std::string& value) {
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
      if(c == '"')
        in_string = true;
      else if(c == ':') {
        key   = json_unquote(json_trim(s.substr(0, i)));
        value = json_trim(s.substr(i + 1));
        return true;
      }
    }
  }
  return false;
}

std::string json_array_of_floats(const float* data, size_t n) {
  std::string out = "[";
  for(size_t i = 0; i < n; i++) {
    if(i) out += ",";
    out += json_number(static_cast<double>(data[i]));
  }
  out += "]";
  return out;
}
void json_floats_from_array(const std::string& text, float* out, size_t n) {
  auto parts = json_split_top_level(text);
  for(size_t i = 0; i < n && i < parts.size(); i++) {
    try {
      out[i] = static_cast<float>(std::stod(parts[i]));
    } catch(const std::exception& e) {
      CUTIL_PRINTF("[cutil::json_floats_from_array] '%s' is not a number: %s\n", parts[i].c_str(), e.what());
      out[i] = 0.0f; // 数値として読めない要素は0として扱う
    }
  }
}

// element_type/seq_*アクセサだけを頼りに任意コンテナ型をJSON化する共通実装(uiVector<T>/std::vector<T>が共有する)。
std::string generic_container_to_json(const PropInfo* type, const void* obj) {
  size_t n        = type->seq_size(obj);
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

bool generic_container_from_json(const PropInfo* type, void* obj, const std::string& text) {
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

} // namespace detail
} // namespace cutil
