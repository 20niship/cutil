#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <cutil/path.hpp>
#include <cutil/prop_registry.hpp>
#include <cutil/quaternion.hpp>
#include <cutil/rect.hpp>
#include <cutil/rect3d.hpp>
#include <cutil/ref.hpp>
#include <cutil/string.hpp>
#include <cutil/vec.hpp>
#include <cutil/vector.hpp>

// cutil::Prop / cutil::PropInfo: Blender の DNA/RNA に相当するシリアライズシステム。
//
// PropInfo (RNA相当) は、複数フィールド分の PropInfo::Data (1フィールドの意味・
// UI表示・値域・生バイナリデータ内でのoffset/sizeを記述するメタ情報) のリストを
// 保持するスキーマ型。PropInfoは2通りの使われ方をする:
//   (a) Prop自身が自分のdata_用に内部で持つもの(値へのアクセサとして、
//       Prop::data_内でのoffsetを指す)
//   (b) 呼び出し側が特定のC++構造体(例: Model3D)用に一度だけ静的に生成し、
//       Prop::dump()/load_to() へ「ルール」として渡すもの(この場合の
//       offsetは呼び出し側の構造体のメモリ内でのoffsetを指す)
// どちらの場合も PropInfo::Data::name をキーとした線形探索で解決する。

namespace cutil {

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
  Ref,     // 単一の参照(Ref<T>/WeakPtr<T>)。実体は持たず生ポインタのみを保持する
  RefList, // 複数の参照(std::vector<Ref<T>>)。std::vector<void*>として保持する
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

// POD型(Bool/Int/Float/Vec3/Vec4/Quat/Range/Rect/Rect3D/Ref)かどうか。
// POD型はdata_へ直接memcpyできる。それ以外(Str/Path/Binary/Nested/Custom/RefList)は
// data_内の該当offsetに実際のライブオブジェクトをplacement-newで構築する必要がある
// (= is_pointer == true)。Refは「生ポインタ1個」というPOD値として扱うため、
// 参照であってもis_pointer==falseになる点に注意。
inline bool prop_type_is_pointer(PropType type) {
  switch(type) {
    case PropType::Str:
    case PropType::Path:
    case PropType::Binary:
    case PropType::Nested:
    case PropType::Custom:
    case PropType::RefList: return true;
    default: return false;
  }
}

struct PropInfo {
  // 1フィールド分のメタデータ(旧PropInfo相当)。PropInfoはこのDataのリストを持つ。
  struct Data {
    char name[32]  = {}; // フィールド名 (キー相当)
    char label[64] = {}; // UI表示名
    char desc[256] = {}; // 説明(ツールチップ用)

    PropType type     = PropType::Int;
    PropWidget widget = PropWidget::Auto;
    PropFlags flags   = PropFlags::None;
    float min_value   = 0;
    float max_value   = 0;
    float drag_speed  = 1.0f;

    size_t offset    = 0; // 値へのアクセサ: Prop自身のdata_内 or 外部構造体のメモリ内でのoffset(文脈依存)
    size_t size      = 0;
    bool is_pointer  = false; // Str/Path/Binary/Nested/Custom/RefListなど間接参照/非トリビアルコピーが必要か
    uint32_t version = 1;     // per-fieldバージョン。prop_io.hppのバイナリload時の互換性チェックに使う

    char custom_type_name[32]       = {}; // PropType::Customの場合のみ使用 (CustomTypeRegistryのキー)
    unsigned char default_value[32] = {}; // 初期値。typeに応じて解釈する固定長バッファ

    // PropType::Ref/RefListの場合のみ使用するフック。外部構造体側のRef<T>/WeakPtr<T>
    // フィールドと、Prop側が保持する生ポインタ(群)との相互変換を担う(実装はmake_ref/
    // make_ref_listが型ごとに生成する)。関数ポインタのみなのでData自体のtrivial
    // copyabilityは保たれる。
    using RefExtractFn     = void* (*)(const void* field_ptr);
    using RefAssignFn      = void (*)(void* field_ptr, void* raw_ptr);
    using RefListExtractFn = void (*)(const void* field_ptr, std::vector<void*>& out);
    using RefListAssignFn  = void (*)(void* field_ptr, const std::vector<void*>& in);

    RefExtractFn ref_extract         = nullptr;
    RefAssignFn ref_assign           = nullptr;
    RefListExtractFn ref_list_extract = nullptr;
    RefListAssignFn ref_list_assign   = nullptr;

    Data() = default;
    Data(const char* name_, PropType type_, size_t offset_, size_t size_, bool is_pointer_) : type(type_), offset(offset_), size(size_), is_pointer(is_pointer_) { set_name(name_); }

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

    // 参照フィールド用のルール生成ヘルパー。
    // make_ref<T>: WeakPtr<T>フィールド1個を、生ポインタ(void*)1個として
    // dump/load_toできるようにする(実体はT側のenable_ref_from_this経由で再構築する)。
    template <typename T> static Data make_ref(const char* name_, size_t offset_) {
      Data d(name_, PropType::Ref, offset_, sizeof(void*), false);
      d.ref_extract = [](const void* field_ptr) -> void* {
        const auto* wp = reinterpret_cast<const WeakPtr<T>*>(field_ptr);
        return const_cast<void*>(static_cast<const void*>(wp->lock().get()));
      };
      d.ref_assign = [](void* field_ptr, void* raw_ptr) {
        auto* wp = reinterpret_cast<WeakPtr<T>*>(field_ptr);
        if(raw_ptr) {
          *wp = reinterpret_cast<T*>(raw_ptr)->weak_from_this();
        } else {
          wp->reset();
        }
      };
      return d;
    }

    // make_ref_list<T>: std::vector<Ref<T>>フィールドを、生ポインタのリスト
    // (std::vector<void*>)としてdump/load_toできるようにする。
    template <typename T> static Data make_ref_list(const char* name_, size_t offset_) {
      Data d(name_, PropType::RefList, offset_, sizeof(std::vector<void*>), true);
      d.ref_list_extract = [](const void* field_ptr, std::vector<void*>& out) {
        const auto* vec = reinterpret_cast<const std::vector<Ref<T>>*>(field_ptr);
        out.clear();
        out.reserve(vec->size());
        for(const auto& r : *vec) out.push_back(static_cast<void*>(r.get()));
      };
      d.ref_list_assign = [](void* field_ptr, const std::vector<void*>& in) {
        auto* vec = reinterpret_cast<std::vector<Ref<T>>*>(field_ptr);
        vec->clear();
        vec->reserve(in.size());
        for(void* raw_ptr : in) {
          if(raw_ptr) vec->push_back(reinterpret_cast<T*>(raw_ptr)->ref_from_this());
        }
      };
      return d;
    }
  };

  char id[32] = {}; // このPropInfo(スキーマ)自体の名前。例: "Model3D"。任意で未設定でもよい
  uiVector<Data> infos;

  PropInfo()                          = default;
  PropInfo(const PropInfo&)           = default;
  PropInfo& operator=(const PropInfo&) = default;

  PropInfo(PropInfo&& other) noexcept { swap(other); }
  PropInfo& operator=(PropInfo&& other) noexcept {
    swap(other);
    return *this;
  }

  // Model3DInfoのような「型ごとに一度だけ静的に生成されるルール」を、従来の
  // PropInfoList(std::vector<PropInfo>)と同じ集約初期化の書き味で書けるようにする。
  PropInfo(std::initializer_list<Data> list) {
    infos.reserve(static_cast<int>(list.size()));
    for(const auto& d : list) infos.push_back(d);
  }

  void set_id(const char* s) {
    std::strncpy(id, s, sizeof(id) - 1);
    id[sizeof(id) - 1] = '\0';
  }

  Data& emplace_back(const char* name_, PropType type_, size_t offset_, size_t size_, bool is_pointer_) {
    infos.push_back(Data(name_, type_, offset_, size_, is_pointer_));
    return infos.back();
  }

  void swap(PropInfo& other) noexcept {
    infos.swap(other.infos);
    char tmp[32];
    std::memcpy(tmp, id, sizeof(id));
    std::memcpy(id, other.id, sizeof(id));
    std::memcpy(other.id, tmp, sizeof(id));
  }

  [[nodiscard]] size_t size() const { return static_cast<size_t>(infos.size()); }
  [[nodiscard]] bool empty() const { return infos.empty(); }

  Data* begin() { return infos.begin(); }
  Data* end() { return infos.end(); }
  [[nodiscard]] const Data* begin() const { return infos.begin(); }
  [[nodiscard]] const Data* end() const { return infos.end(); }

  Data& operator[](size_t i) { return infos[static_cast<int>(i)]; }
  const Data& operator[](size_t i) const { return infos[static_cast<int>(i)]; }
};

inline size_t align_up(size_t offset, size_t align) { return (offset + align - 1) & ~(align - 1); }

inline const PropInfo::Data* find_info(const PropInfo& list, const char* name) {
  for(const auto& info : list) {
    if(std::strncmp(info.name, name, sizeof(info.name)) == 0) return &info;
  }
  return nullptr;
}

inline PropInfo::Data* find_info(PropInfo& list, const char* name) {
  for(auto& info : list) {
    if(std::strncmp(info.name, name, sizeof(info.name)) == 0) return &info;
  }
  return nullptr;
}

inline bool validate(const PropInfo::Data& info, float value) {
  if(info.min_value == 0 && info.max_value == 0) return true; // 範囲未設定は無条件許可
  return value >= info.min_value && value <= info.max_value;
}

// C++の型 T <-> PropType の対応。Prop::set<T>/get<T>で使う。
// 新しい組み込み型を追加する場合はここに特殊化を追加する。
template <typename T> struct prop_type_traits; // 未特殊化の型はコンパイルエラーになる

#define CUTIL_PROP_TRAIT(CPP_TYPE, PROP_TYPE, IS_POINTER) \
  template <> struct prop_type_traits<CPP_TYPE> {         \
    static constexpr PropType type   = PROP_TYPE;         \
    static constexpr bool is_pointer = IS_POINTER;        \
  }

CUTIL_PROP_TRAIT(bool, PropType::Bool, false);
CUTIL_PROP_TRAIT(int32_t, PropType::Int, false);
CUTIL_PROP_TRAIT(float, PropType::Float, false);
CUTIL_PROP_TRAIT(Str, PropType::Str, true);
CUTIL_PROP_TRAIT(Path, PropType::Path, true);
CUTIL_PROP_TRAIT(Vec3f, PropType::Vec3, false);
CUTIL_PROP_TRAIT(Vec4f, PropType::Vec4, false);
CUTIL_PROP_TRAIT(Quat<float>, PropType::Quat, false);
CUTIL_PROP_TRAIT(Range, PropType::Range, false);
CUTIL_PROP_TRAIT(Rect, PropType::Rect, false);
CUTIL_PROP_TRAIT(Rect3D, PropType::Rect3D, false);
CUTIL_PROP_TRAIT(std::vector<uint8_t>, PropType::Binary, true);
CUTIL_PROP_TRAIT(CustomSlot, PropType::Custom, true);

#undef CUTIL_PROP_TRAIT

// 実行時のPropTypeからsize/alignを引く(Prop::dump()が新規フィールドを追加する際に使う)。
// 定義はProp classの後(sizeof(Prop)を使うため)。
size_t prop_type_size(PropType type);
size_t prop_type_align(PropType type);

// Prop - DNA相当。生バイナリバッファ(data_)とPropInfo(infos_)を保持する
// 動的プロパティコンテナ。名前でフィールドを出し入れする(Blenderのカスタム
// プロパティ相当)ほか、別のPropを子として持てる(PropType::Nestedによる
// 階層構造)。
class Prop {
public:
  Prop() = default;

  Prop(const Prop& other) : data_(other.data_.size()), infos_(other.infos_) {
    for(const auto& info : infos_) {
      const uint8_t* src = other.data_.data() + info.offset;
      uint8_t* dst        = data_.data() + info.offset;
      if(info.is_pointer) {
        copy_construct_by_type(info.type, dst, src);
      } else {
        std::memcpy(dst, src, info.size);
      }
    }
  }

  Prop(Prop&& other) noexcept : data_(std::move(other.data_)), infos_(std::move(other.infos_)) {}

  Prop& operator=(const Prop& other) {
    if(this != &other) {
      Prop tmp(other);
      swap(tmp);
    }
    return *this;
  }

  Prop& operator=(Prop&& other) noexcept {
    if(this != &other) {
      destroy_all_pointer_fields();
      data_  = std::move(other.data_);
      infos_ = std::move(other.infos_);
    }
    return *this;
  }

  ~Prop() { destroy_all_pointer_fields(); }

  void swap(Prop& other) noexcept {
    data_.swap(other.data_);
    infos_.swap(other.infos_);
  }

  template <typename T> void set(const char* name, const T& value, const char* label = nullptr, const char* desc = nullptr) {
    using Traits       = prop_type_traits<T>;
    PropInfo::Data* info = find_info(infos_, name);
    bool is_new_field    = false;
    if(!info) {
      info = &add_field(name, Traits::type, sizeof(T), alignof(T), Traits::is_pointer);
      if(label) info->set_label(label);
      if(desc) info->set_desc(desc);
      is_new_field = true;
    } else if(info->type != Traits::type) {
      throw std::logic_error(std::string("Prop::set: field '") + name + "' already exists with a different type");
    }

    uint8_t* dst = data_.data() + info->offset;
    if constexpr(Traits::is_pointer) {
      if(is_new_field) {
        new(dst) T(value); // 新規フィールドはまだ生存オブジェクトがないためplacement-new
      } else {
        *reinterpret_cast<T*>(dst) = value; // 既存フィールドはコピー代入(placement-newだと旧値がリークする)
      }
    } else {
      std::memcpy(dst, &value, sizeof(T));
    }
  }

  template <typename T> T& get(const char* name) {
    PropInfo::Data* info = find_info(infos_, name);
    if(!info) throw std::out_of_range(std::string("Prop: field not found: ") + name);
    if(info->type != prop_type_traits<T>::type) throw std::logic_error(std::string("Prop::get: field '") + name + "' has a different type");
    return *reinterpret_cast<T*>(data_.data() + info->offset);
  }

  template <typename T> const T& get(const char* name) const {
    const PropInfo::Data* info = find_info(infos_, name);
    if(!info) throw std::out_of_range(std::string("Prop: field not found: ") + name);
    if(info->type != prop_type_traits<T>::type) throw std::logic_error(std::string("Prop::get: field '") + name + "' has a different type");
    return *reinterpret_cast<const T*>(data_.data() + info->offset);
  }

  [[nodiscard]] bool contains(const char* name) const { return find_info(infos_, name) != nullptr; }

  bool erase(const char* name) {
    for(auto it = infos_.infos.begin(); it != infos_.infos.end(); ++it) {
      if(std::strncmp(it->name, name, sizeof(it->name)) == 0) {
        if(it->is_pointer) destroy_by_type(it->type, data_.data() + it->offset);
        infos_.infos.erase(it);
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] const PropInfo& infos() const { return infos_; }
  [[nodiscard]] size_t field_count() const { return infos_.size(); }

  // フィールドのversionを直接書き換える(cutil/prop_io.hpp のJSON経由load(Phase6)が、
  // JSON側に記録されていたversionをPropInfoへ反映するために使う)。
  bool set_field_version(const char* name, uint32_t version) {
    PropInfo::Data* info = find_info(infos_, name);
    if(!info) return false;
    info->version = version;
    return true;
  }

  // Prop自身の生バイナリバッファへの読み取り専用アクセス。cutil/prop_io.hpp の
  // binary dump/load(Phase5以降)が、POD型フィールドの実データをそのままファイルへ
  // コピーするために使う。
  [[nodiscard]] const uint8_t* raw_data() const { return data_.data(); }
  [[nodiscard]] size_t raw_size() const { return data_.size(); }

  // POD型のみ対応の汎用バイナリload(cutil/prop_io.hppが使う)。ポインタ型は
  // 非対応(呼び出し元でチェックすること)。
  void set_raw_pod(const char* name, PropType type, const void* bytes, size_t size) {
    if(prop_type_is_pointer(type)) throw std::logic_error(std::string("Prop::set_raw_pod: '") + name + "' is a pointer type, not supported here");
    PropInfo::Data* info = find_info(infos_, name);
    if(!info) {
      info = &add_field(name, type, size, prop_type_align(type), false);
    } else if(info->type != type) {
      throw std::logic_error(std::string("Prop::set_raw_pod: field '") + name + "' already exists with a different type");
    }
    std::memcpy(data_.data() + info->offset, bytes, size);
  }

  // 階層構造: 子Propを名前付きフィールドとして持てる (PropType::Nested)
  void set_child(const char* name, const Prop& child) { set<Prop>(name, child); }
  Prop& get_child(const char* name) { return get<Prop>(name); }
  [[nodiscard]] const Prop& get_child(const char* name) const { return get<Prop>(name); }

  // 外部C++構造体との高速相互変換。
  // rule(PropInfo)は呼び出し側の構造体用に型ごとに一度だけ静的に生成される想定
  // (offsetofで各フィールドのoffsetを記述したもの)。rule::Data::offsetは`data`が指す
  // 構造体のメモリ内でのoffsetを表す(Prop自身のdata_内offsetとは別物)。
  //
  // dump: data(構造体の生メモリ)からruleに従って値を読み取り、Prop自身にセットする。
  // PropType::CustomなフィールドはCustomSlotとして呼び出し側の構造体に直接埋め込まれて
  // いる前提で扱う(ruleのis_pointer/sizeはCustomSlot自身のものを指定する)。
  // PropType::Ref/RefListなフィールドはruleのref_extract/ref_list_extractフック経由で
  // 生ポインタ(群)を取り出す(Str/Pathのようなメモリレイアウト一致には頼らない)。
  bool dump(const void* data, const PropInfo* rule) {
    const auto* base = reinterpret_cast<const uint8_t*>(data);
    for(const auto& src_info : *rule) {
      set_raw(src_info, base + src_info.offset);
    }
    return true;
  }

  // load_to: Prop自身が持つ値を、ruleに従って構造体(data)の生メモリへ書き込む。
  // dataは既に構築済み(コンストラクタが走っている)オブジェクトへのポインタである
  // ことを前提とする(ポインタ型フィールドはコピー代入で書き込む)。
  // Propに該当フィールドが無い/型が一致しない場合はそのフィールドをスキップし、
  // 戻り値はfalseになる。PropType::Ref/RefListはruleのref_assign/ref_list_assign
  // フック経由で、生ポインタから呼び出し側のRef<T>/WeakPtr<T>を再構築する
  // (T::ref_from_this()/weak_from_this()を使うため、Tはenable_ref_from_this<T>を
  // 継承している必要がある)。
  [[nodiscard]] bool load_to(void* data, const PropInfo* rule) const {
    auto* base    = reinterpret_cast<uint8_t*>(data);
    bool complete = true;
    for(const auto& dst_info : *rule) {
      const PropInfo::Data* info = find_info(infos_, dst_info.name);
      if(!info || info->type != dst_info.type) {
        complete = false;
        continue;
      }
      const uint8_t* src = data_.data() + info->offset;
      uint8_t* dst        = base + dst_info.offset;

      if(dst_info.type == PropType::Ref) {
        void* raw_ptr = nullptr;
        std::memcpy(&raw_ptr, src, sizeof(void*));
        if(dst_info.ref_assign) dst_info.ref_assign(dst, raw_ptr);
        continue;
      }
      if(dst_info.type == PropType::RefList) {
        const auto* raw_list = reinterpret_cast<const std::vector<void*>*>(src);
        if(dst_info.ref_list_assign) dst_info.ref_list_assign(dst, *raw_list);
        continue;
      }

      if(info->is_pointer) {
        assign_by_type(info->type, dst, src);
      } else {
        std::memcpy(dst, src, info->size);
      }
    }
    return complete;
  }

private:
  std::vector<uint8_t> data_;
  PropInfo infos_;

  PropInfo::Data& add_field(const char* name, PropType type, size_t sz, size_t align, bool is_pointer_) {
    size_t new_offset = align_up(data_.size(), align);
    data_.resize(new_offset + sz, 0);
    return infos_.emplace_back(name, type, new_offset, sz, is_pointer_);
  }

  // dump()から呼ばれる、実行時PropTypeに基づく汎用set。set<T>()と異なりコンパイル時の
  // 型情報を使わない(呼び出し元の構造体のフィールドをPropTypeのタグだけを頼りに
  // 読み込むため)。PropType::Ref/RefListはref_extract/ref_list_extractフックを経由し、
  // それ以外は従来通りoffset/sizeベースのmemcpy or copy_construct_by_typeで処理する。
  void set_raw(const PropInfo::Data& src_info, const void* src_ptr) {
    PropInfo::Data* info = find_info(infos_, src_info.name);
    bool is_new_field    = false;
    if(!info) {
      info             = &add_field(src_info.name, src_info.type, prop_type_size(src_info.type), prop_type_align(src_info.type), src_info.is_pointer);
      info->widget     = src_info.widget;
      info->flags      = src_info.flags;
      info->min_value  = src_info.min_value;
      info->max_value  = src_info.max_value;
      info->drag_speed = src_info.drag_speed;
      std::memcpy(info->label, src_info.label, sizeof(info->label));
      std::memcpy(info->desc, src_info.desc, sizeof(info->desc));
      is_new_field = true;
    } else if(info->type != src_info.type) {
      throw std::logic_error(std::string("Prop::dump: field '") + src_info.name + "' already exists with a different type");
    }

    uint8_t* dst = data_.data() + info->offset;

    if(src_info.type == PropType::Ref) {
      void* raw_ptr = src_info.ref_extract ? src_info.ref_extract(src_ptr) : nullptr;
      std::memcpy(dst, &raw_ptr, sizeof(void*));
      return;
    }
    if(src_info.type == PropType::RefList) {
      std::vector<void*> tmp;
      if(src_info.ref_list_extract) src_info.ref_list_extract(src_ptr, tmp);
      if(is_new_field) {
        new(dst) std::vector<void*>(std::move(tmp));
      } else {
        *reinterpret_cast<std::vector<void*>*>(dst) = std::move(tmp);
      }
      return;
    }

    if(info->is_pointer) {
      if(is_new_field) {
        copy_construct_by_type(info->type, dst, src_ptr);
      } else {
        assign_by_type(info->type, dst, src_ptr);
      }
    } else {
      std::memcpy(dst, src_ptr, info->size);
    }
  }

  void destroy_all_pointer_fields() {
    for(const auto& info : infos_) {
      if(info.is_pointer) destroy_by_type(info.type, data_.data() + info.offset);
    }
  }

  // is_pointer==trueな型(Str/Path/Binary/Nested/RefList)のコピー構築/破棄をPropType経由で
  // ディスパッチする。Custom型(型消去)はPhase4のCustomTypeRegistryで対応する。
  static void copy_construct_by_type(PropType type, void* dst, const void* src) {
    switch(type) {
      case PropType::Str: new(dst) Str(*reinterpret_cast<const Str*>(src)); break;
      case PropType::Path: new(dst) Path(*reinterpret_cast<const Path*>(src)); break;
      case PropType::Binary: new(dst) std::vector<uint8_t>(*reinterpret_cast<const std::vector<uint8_t>*>(src)); break;
      case PropType::Nested: new(dst) Prop(*reinterpret_cast<const Prop*>(src)); break;
      case PropType::Custom: new(dst) CustomSlot(*reinterpret_cast<const CustomSlot*>(src)); break;
      case PropType::RefList: new(dst) std::vector<void*>(*reinterpret_cast<const std::vector<void*>*>(src)); break;
      default: break; // POD型(Refを含む)はここに来ない
    }
  }

  static void destroy_by_type(PropType type, void* obj) {
    switch(type) {
      case PropType::Str: reinterpret_cast<Str*>(obj)->~Str(); break;
      case PropType::Path: reinterpret_cast<Path*>(obj)->~Path(); break;
      case PropType::Binary: reinterpret_cast<std::vector<uint8_t>*>(obj)->~vector(); break;
      case PropType::Nested: reinterpret_cast<Prop*>(obj)->~Prop(); break;
      case PropType::Custom: reinterpret_cast<CustomSlot*>(obj)->~CustomSlot(); break;
      case PropType::RefList: reinterpret_cast<std::vector<void*>*>(obj)->~vector(); break;
      default: break; // POD型(Refを含む)はここに来ない
    }
  }

  // is_pointer==trueな型への「既に生存しているオブジェクトへのコピー代入」。
  // load_to()は呼び出し元の構造体が既に構築済みであることを前提とするため、
  // placement-newではなくoperator=を使う。
  static void assign_by_type(PropType type, void* dst, const void* src) {
    switch(type) {
      case PropType::Str: *reinterpret_cast<Str*>(dst) = *reinterpret_cast<const Str*>(src); break;
      case PropType::Path: *reinterpret_cast<Path*>(dst) = *reinterpret_cast<const Path*>(src); break;
      case PropType::Binary: *reinterpret_cast<std::vector<uint8_t>*>(dst) = *reinterpret_cast<const std::vector<uint8_t>*>(src); break;
      case PropType::Nested: *reinterpret_cast<Prop*>(dst) = *reinterpret_cast<const Prop*>(src); break;
      case PropType::Custom: *reinterpret_cast<CustomSlot*>(dst) = *reinterpret_cast<const CustomSlot*>(src); break;
      case PropType::RefList: *reinterpret_cast<std::vector<void*>*>(dst) = *reinterpret_cast<const std::vector<void*>*>(src); break;
      default: break; // POD型(Refを含む)はここに来ない
    }
  }

public:
  // Custom型(CustomTypeRegistryに登録済みの任意の型)のフィールドをセットする。
  // 組み込み型のset<T>とは異なり、type_nameを明示的に指定してレジストリ経由で
  // コピー構築/破棄を行う。
  template <typename T> void set_custom(const char* name, const char* type_name, const T& value, const char* label = nullptr, const char* desc = nullptr) {
    CustomSlot slot       = CustomSlot::make<T>(type_name, value);
    PropInfo::Data* info  = find_info(infos_, name);
    bool is_new_field     = false;
    if(!info) {
      info = &add_field(name, PropType::Custom, sizeof(CustomSlot), alignof(CustomSlot), true);
      std::strncpy(info->custom_type_name, type_name, sizeof(info->custom_type_name) - 1);
      if(label) info->set_label(label);
      if(desc) info->set_desc(desc);
      is_new_field = true;
    } else if(info->type != PropType::Custom) {
      throw std::logic_error(std::string("Prop::set_custom: field '") + name + "' already exists with a different type");
    }
    uint8_t* dst = data_.data() + info->offset;
    if(is_new_field) {
      new(dst) CustomSlot(std::move(slot));
    } else {
      *reinterpret_cast<CustomSlot*>(dst) = std::move(slot);
    }
  }

  template <typename T> [[nodiscard]] T& get_custom(const char* name) {
    PropInfo::Data* info = find_info(infos_, name);
    if(!info) throw std::out_of_range(std::string("Prop: field not found: ") + name);
    if(info->type != PropType::Custom) throw std::logic_error(std::string("Prop::get_custom: field '") + name + "' is not a Custom field");
    return reinterpret_cast<CustomSlot*>(data_.data() + info->offset)->get<T>();
  }

  template <typename T> [[nodiscard]] const T& get_custom(const char* name) const {
    const PropInfo::Data* info = find_info(infos_, name);
    if(!info) throw std::out_of_range(std::string("Prop: field not found: ") + name);
    if(info->type != PropType::Custom) throw std::logic_error(std::string("Prop::get_custom: field '") + name + "' is not a Custom field");
    return reinterpret_cast<const CustomSlot*>(data_.data() + info->offset)->get<T>();
  }

  // 既に構築済みのCustomSlotを、名前付きフィールドとして取り込む(所有権を移動)。
  // cutil/prop_io.hpp のJSON経由load(Phase 6)が、CustomTypeOps::from_json経由で
  // 構築したCustomSlotをPropへ組み込む際に使う。
  void adopt_custom_slot(const char* name, const char* custom_type_name, CustomSlot&& slot) {
    PropInfo::Data* info = find_info(infos_, name);
    bool is_new_field     = false;
    if(!info) {
      info = &add_field(name, PropType::Custom, sizeof(CustomSlot), alignof(CustomSlot), true);
      std::strncpy(info->custom_type_name, custom_type_name, sizeof(info->custom_type_name) - 1);
      is_new_field = true;
    } else if(info->type != PropType::Custom) {
      throw std::logic_error(std::string("Prop::adopt_custom_slot: field '") + name + "' already exists with a different type");
    }
    uint8_t* dst = data_.data() + info->offset;
    if(is_new_field) {
      new(dst) CustomSlot(std::move(slot));
    } else {
      *reinterpret_cast<CustomSlot*>(dst) = std::move(slot);
    }
  }
};

template <> struct prop_type_traits<Prop> {
  static constexpr PropType type   = PropType::Nested;
  static constexpr bool is_pointer = true;
};

inline size_t prop_type_size(PropType type) {
  switch(type) {
    case PropType::Bool: return sizeof(bool);
    case PropType::Int: return sizeof(int32_t);
    case PropType::Float: return sizeof(float);
    case PropType::Str: return sizeof(Str);
    case PropType::Path: return sizeof(Path);
    case PropType::Vec3: return sizeof(Vec3f);
    case PropType::Vec4: return sizeof(Vec4f);
    case PropType::Quat: return sizeof(Quat<float>);
    case PropType::Range: return sizeof(Range);
    case PropType::Rect: return sizeof(Rect);
    case PropType::Rect3D: return sizeof(Rect3D);
    case PropType::Binary: return sizeof(std::vector<uint8_t>);
    case PropType::Nested: return sizeof(Prop);
    case PropType::Custom: return sizeof(CustomSlot); // CustomSlot自体は保持する型に依らず固定サイズ
    case PropType::Ref: return sizeof(void*);
    case PropType::RefList: return sizeof(std::vector<void*>);
  }
  return 0;
}

inline size_t prop_type_align(PropType type) {
  switch(type) {
    case PropType::Bool: return alignof(bool);
    case PropType::Int: return alignof(int32_t);
    case PropType::Float: return alignof(float);
    case PropType::Str: return alignof(Str);
    case PropType::Path: return alignof(Path);
    case PropType::Vec3: return alignof(Vec3f);
    case PropType::Vec4: return alignof(Vec4f);
    case PropType::Quat: return alignof(Quat<float>);
    case PropType::Range: return alignof(Range);
    case PropType::Rect: return alignof(Rect);
    case PropType::Rect3D: return alignof(Rect3D);
    case PropType::Binary: return alignof(std::vector<uint8_t>);
    case PropType::Nested: return alignof(Prop);
    case PropType::Custom: return alignof(CustomSlot);
    case PropType::Ref: return alignof(void*);
    case PropType::RefList: return alignof(std::vector<void*>);
  }
  return 1;
}

} // namespace cutil
