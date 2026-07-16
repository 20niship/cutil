#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

// cutil::Prop / cutil::PropInfo: Blender の DNA/RNA に相当するシリアライズシステム。
//
// PropInfo (RNA相当) は、あるフィールドの意味・UI表示・値域に加えて、
// 生バイナリデータ内でのoffset/sizeを記述するメタ情報。PropInfoListは
// 2通りの使われ方をする:
//   (a) Prop自身が自分のdata_用に内部で持つもの(値へのアクセサとして、
//       Prop::data_内でのoffsetを指す)
//   (b) 呼び出し側が特定のC++構造体(例: Model3D)用に一度だけ静的に生成し、
//       Prop::dump()/load_to() へ「ルール」として渡すもの(この場合の
//       offsetは呼び出し側の構造体のメモリ内でのoffsetを指す)
// どちらの場合も PropInfo::name をキーとした線形探索で解決する。

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

// POD型(Bool/Int/Float/Vec3/Vec4/Quat/Range/Rect/Rect3D)かどうか。
// POD型はdata_へ直接memcpyできる。それ以外(Str/Path/Binary/Nested/Custom)は
// data_内の該当offsetに実際のライブオブジェクトをplacement-newで構築する必要がある
// (= is_pointer == true)。
inline bool prop_type_is_pointer(PropType type) {
  switch(type) {
    case PropType::Str:
    case PropType::Path:
    case PropType::Binary:
    case PropType::Nested:
    case PropType::Custom: return true;
    default: return false;
  }
}

struct PropInfo {
  char name[32]  = {}; // フィールド名 (キー相当)
  char label[64] = {}; // UI表示名
  char desc[256] = {}; // 説明(ツールチップ用)

  PropType type          = PropType::Int;
  PropWidget widget      = PropWidget::Auto;
  PropFlags flags        = PropFlags::None;
  float min_value        = 0;
  float max_value        = 0;
  float drag_speed       = 1.0f;

  size_t offset     = 0;     // 値へのアクセサ: Prop自身のdata_内 or 外部構造体のメモリ内でのoffset(文脈依存)
  size_t size       = 0;
  bool is_pointer   = false; // Str/Path/Binary/Nested/Customなど間接参照/非トリビアルコピーが必要か
  uint32_t version  = 1;

  char custom_type_name[32]        = {}; // PropType::Customの場合のみ使用 (CustomTypeRegistryのキー)
  unsigned char default_value[32]  = {}; // 初期値。typeに応じて解釈する固定長バッファ

  PropInfo() = default;
  PropInfo(const char* name_, PropType type_, size_t offset_, size_t size_, bool is_pointer_) : type(type_), offset(offset_), size(size_), is_pointer(is_pointer_) {
    set_name(name_);
  }

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
};

using PropInfoList = std::vector<PropInfo>;

inline size_t align_up(size_t offset, size_t align) { return (offset + align - 1) & ~(align - 1); }

inline const PropInfo* find_info(const PropInfoList& list, const char* name) {
  for(const auto& info : list) {
    if(std::strncmp(info.name, name, sizeof(info.name)) == 0) return &info;
  }
  return nullptr;
}

inline PropInfo* find_info(PropInfoList& list, const char* name) {
  for(auto& info : list) {
    if(std::strncmp(info.name, name, sizeof(info.name)) == 0) return &info;
  }
  return nullptr;
}

inline bool validate(const PropInfo& info, float value) {
  if(info.min_value == 0 && info.max_value == 0) return true; // 範囲未設定は無条件許可
  return value >= info.min_value && value <= info.max_value;
}

} // namespace cutil
