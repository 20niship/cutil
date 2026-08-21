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
    p.klass     = PropClass::Trivial;
    p.size      = sizeof(float);
    p.align     = alignof(float);
    p.to_json   = [](const void* obj) -> std::string { return detail::json_number(static_cast<double>(*reinterpret_cast<const float*>(obj))); };
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
    p.klass     = PropClass::Trivial;
    p.size      = sizeof(uint8_t);
    p.align     = alignof(uint8_t);
    p.to_json   = [](const void* obj) -> std::string { return std::to_string(*reinterpret_cast<const uint8_t*>(obj)); };
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

} // namespace cutil
