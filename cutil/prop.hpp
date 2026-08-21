#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <cutil/prop_registry.hpp>

// cutil::Prop: DNA相当の動的プロパティコンテナ。フィールドの型情報はPropInfoへ完全委譲し、Prop自身は型ごとのswitchを持たない。

namespace cutil {

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
    bool is_new;
    PropInfo::Field& f = find_or_add_field(name, type, is_new);
    if(is_new) {
      if(label) f.set_label(label);
      if(desc) f.set_desc(desc);
    }
    assign_value(f, type, &value, is_new);
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
    bool is_new;
    PropInfo::Field& f = find_or_add_field(name, type, is_new);
    std::memcpy(data_.data() + f.offset, bytes, type->size);
  }

  // constructed_srcは既に構築済みの一時オブジェクト。コピーして取り込んだ後、呼び出し側の代わりにここで破棄する(所有権移動)。
  void adopt_raw_by_info(const char* name, const PropInfo* type, void* constructed_src) {
    bool is_new;
    PropInfo::Field& f = find_or_add_field(name, type, is_new);
    assign_value(f, type, constructed_src, is_new);
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

  // 名前でフィールドを探し無ければ追加する(set<T>/set_raw/set_raw_pod_by_info/adopt_raw_by_infoが共有する前段ロジック)。
  PropInfo::Field& find_or_add_field(const char* name, const PropInfo* type, bool& is_new) {
    PropInfo::Field* f = find_field(name);
    if(!f) {
      f      = &add_field(name, type);
      is_new = true;
    } else if(f->type != type) {
      throw std::logic_error(std::string("Prop: field '") + name + "' already exists with a different type");
    } else {
      is_new = false;
    }
    return *f;
  }

  // fのバッファへsrc_ptrの値を書き込む。Trivialはmemcpy、非Trivialは既存なら破棄してから再構築する。
  void assign_value(PropInfo::Field& f, const PropInfo* type, const void* src_ptr, bool is_new) {
    uint8_t* dst = data_.data() + f.offset;
    if(type->klass == PropKlass::Trivial) {
      std::memcpy(dst, src_ptr, type->size);
    } else {
      if(!is_new && type->dtor) type->dtor(dst); // 既存フィールドは一旦破棄してから再構築する(assign用の別関数は持たない)
      type->copy_ctor(dst, src_ptr);
    }
  }

  // dump()用の実行時PropInfoに基づく汎用set(set<T>()と異なりコンパイル時型情報を使わない)。
  void set_raw(const char* name, const PropInfo* type, const void* src_ptr) {
    bool is_new;
    PropInfo::Field& f = find_or_add_field(name, type, is_new);
    assign_value(f, type, src_ptr, is_new);
  }

  void destroy_all_fields() {
    for(const auto& f : fields_) {
      if(f.type->klass != PropKlass::Trivial && f.type->dtor) f.type->dtor(data_.data() + f.offset);
    }
  }
};

} // namespace cutil

namespace cutil {

// prop_info_of<Prop>(): Prop自身をIndirect型として登録し、旧PropType::Nestedの特別扱いを廃した(他のIndirect型と同列に扱える)。
template <> struct PropInfoOf<Prop> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("Prop");
      p.klass        = PropKlass::Indirect;
      p.size         = sizeof(Prop);
      p.align        = alignof(Prop);
      p.copy_ctor    = [](void* dst, const void* src) { new(dst) Prop(*reinterpret_cast<const Prop*>(src)); };
      p.dtor         = [](void* obj) { reinterpret_cast<Prop*>(obj)->~Prop(); };
      p.default_ctor = [](void* obj) { new(obj) Prop(); };
      // to_json/from_jsonはprop_io.hppがprop_dump_json/prop_load_json定義後に遅延バインドする(循環依存回避)。
      return p;
    }();
    return &info;
  }
};

} // namespace cutil
