#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include <cutil/json.hpp>
#include <cutil/prop.hpp>

// cutil::prop_dump_binary / cutil::prop_load_binary: Prop を独自バイナリ形式で
// dump/loadする(Phase 5: POD型限定、ファイルI/O)。
// cutil::prop_dump_json / cutil::prop_load_json: JSON経由のフォールバック
// load/save(Phase 6)。全フィールド(POD/Str/Path/Binary/Nested/Custom)を
// 人間可読なJSONとして保存/復元でき、prop_load_binaryのper-fieldバージョン
// 不一致時に自動的に使われる。
//
// バイナリフォーマット:
//   [FileHeader]
//     magic[4]        = "CPRP"
//     format_version  : u32  (このフォーマット自体のバージョン)
//     endianness_tag  : u32  (= 0x01020304。ホストと不一致ならload失敗として扱う。
//                              バイトスワップには対応しない)
//     entry_count     : u32
//     flags           : u32  (bit0 = has_pointer_blob(Phase7)、bit1 = has_json_block)
//   [EntryTable] (entry_count個)
//     char     name[32]
//     uint16_t prop_type
//     uint16_t prop_info_version
//     uint32_t data_offset  ([DataBlock]内でのoffset)
//     uint32_t data_size
//     uint32_t flags        (bit0 = is_pointer。Phase5では常に0のみ許可)
//   [DataBlock]
//     is_pointer==falseなエントリの実データ(Prop::raw_data()の該当バイト範囲を
//     そのままコピーしたもの)
//   [JsonBlockHeader] { uint32_t json_size; }   (has_json_blockの場合のみ)
//   [JsonBlock]                                  全フィールドの人間可読JSON
//     (フォールバック/デバッグ/新旧混在ファイルの単一ファイル完結のために
//      常に併載する)

namespace cutil {

constexpr uint32_t PROP_BINARY_FORMAT_VERSION = 1;
constexpr uint32_t PROP_BINARY_ENDIANNESS_TAG = 0x01020304;

struct PropFileHeader {
  char magic[4]           = {'C', 'P', 'R', 'P'};
  uint32_t format_version = PROP_BINARY_FORMAT_VERSION;
  uint32_t endianness_tag = PROP_BINARY_ENDIANNESS_TAG;
  uint32_t entry_count    = 0;
  uint32_t flags          = 0; // bit0: has_pointer_blob (Phase7)
};

struct PropEntryHeader {
  char name[32]             = {};
  uint16_t prop_type        = 0;
  uint16_t prop_info_version = 1;
  uint32_t data_offset      = 0; // [DataBlock]内でのoffset
  uint32_t data_size        = 0;
  uint32_t flags            = 0; // bit0: is_pointer (Phase7)
};

namespace detail {

inline json::Value floats_to_json(const float* data, size_t n) {
  json::Value arr = json::Value::make_array();
  for(size_t i = 0; i < n; i++) arr.push_back(json::Value::make_double(static_cast<double>(data[i])));
  return arr;
}

inline void json_to_floats(const json::Value& arr, float* out, size_t n) {
  for(size_t i = 0; i < n; i++) out[i] = static_cast<float>(arr.get(i).as_double());
}

// POD型(Bool/Int/Float/Vec3/Vec4/Quat/Range/Rect/Rect3D)をJSON値へ変換する。
inline json::Value pod_to_json(PropType type, const void* ptr) {
  switch(type) {
    case PropType::Bool: return json::Value::make_bool(*reinterpret_cast<const bool*>(ptr));
    case PropType::Int: return json::Value::make_int(*reinterpret_cast<const int32_t*>(ptr));
    case PropType::Float: return json::Value::make_double(static_cast<double>(*reinterpret_cast<const float*>(ptr)));
    case PropType::Vec3: return floats_to_json(reinterpret_cast<const Vec3f*>(ptr)->data, 3);
    case PropType::Vec4: return floats_to_json(reinterpret_cast<const Vec4f*>(ptr)->data, 4);
    case PropType::Quat: {
      const auto* q  = reinterpret_cast<const Quat<float>*>(ptr);
      float f[4]     = {q->x, q->y, q->z, q->w};
      return floats_to_json(f, 4);
    }
    case PropType::Range: {
      const auto* r = reinterpret_cast<const Range*>(ptr);
      float f[2]    = {r->min, r->max};
      return floats_to_json(f, 2);
    }
    case PropType::Rect: {
      const auto* r = reinterpret_cast<const Rect*>(ptr);
      float f[4]    = {r->x.min, r->x.max, r->y.min, r->y.max};
      return floats_to_json(f, 4);
    }
    case PropType::Rect3D: {
      const auto* r = reinterpret_cast<const Rect3D*>(ptr);
      float f[6]    = {r->x.min, r->x.max, r->y.min, r->y.max, r->z.min, r->z.max};
      return floats_to_json(f, 6);
    }
    default: return json::Value();
  }
}

inline void json_to_pod(PropType type, const json::Value& v, void* ptr) {
  switch(type) {
    case PropType::Bool: *reinterpret_cast<bool*>(ptr) = v.as_bool(); break;
    case PropType::Int: *reinterpret_cast<int32_t*>(ptr) = static_cast<int32_t>(v.as_int()); break;
    case PropType::Float: *reinterpret_cast<float*>(ptr) = static_cast<float>(v.as_double()); break;
    case PropType::Vec3: json_to_floats(v, reinterpret_cast<Vec3f*>(ptr)->data, 3); break;
    case PropType::Vec4: json_to_floats(v, reinterpret_cast<Vec4f*>(ptr)->data, 4); break;
    case PropType::Quat: {
      float f[4];
      json_to_floats(v, f, 4);
      auto* q = reinterpret_cast<Quat<float>*>(ptr);
      q->x = f[0];
      q->y = f[1];
      q->z = f[2];
      q->w = f[3];
      break;
    }
    case PropType::Range: {
      float f[2];
      json_to_floats(v, f, 2);
      auto* r  = reinterpret_cast<Range*>(ptr);
      r->min   = f[0];
      r->max   = f[1];
      break;
    }
    case PropType::Rect: {
      float f[4];
      json_to_floats(v, f, 4);
      auto* r    = reinterpret_cast<Rect*>(ptr);
      r->x.min   = f[0];
      r->x.max   = f[1];
      r->y.min   = f[2];
      r->y.max   = f[3];
      break;
    }
    case PropType::Rect3D: {
      float f[6];
      json_to_floats(v, f, 6);
      auto* r    = reinterpret_cast<Rect3D*>(ptr);
      r->x.min   = f[0];
      r->x.max   = f[1];
      r->y.min   = f[2];
      r->y.max   = f[3];
      r->z.min   = f[4];
      r->z.max   = f[5];
      break;
    }
    default: break;
  }
}

} // namespace detail

// dump: Prop全体(POD/Str/Path/Binary/Nested/Custom全て)を人間可読なJSON文字列
// へ変換する。各フィールドは {"type":.., "version":.., "value":..} の形で保存し、
// load時に自身が保持する型情報を頼りに復元できるようにする。
inline bool prop_dump_json(const Prop& prop, std::string& out) {
  json::Value root = json::Value::make_object();
  const uint8_t* base = prop.raw_data();

  for(const auto& info : prop.infos()) {
    json::Value field = json::Value::make_object();
    field.set("type", json::Value::make_int(static_cast<int>(info.type)));
    field.set("version", json::Value::make_int(info.version));

    const uint8_t* ptr = base + info.offset;
    switch(info.type) {
      case PropType::Str: field.set("value", json::Value::make_string(reinterpret_cast<const Str*>(ptr)->c_str())); break;
      case PropType::Path: field.set("value", json::Value::make_string(reinterpret_cast<const Path*>(ptr)->str().c_str())); break;
      case PropType::Binary: {
        const auto* bin = reinterpret_cast<const std::vector<uint8_t>*>(ptr);
        json::Value arr = json::Value::make_array();
        for(uint8_t b : *bin) arr.push_back(json::Value::make_int(b));
        field.set("value", arr);
        break;
      }
      case PropType::Nested: {
        const auto* child = reinterpret_cast<const Prop*>(ptr);
        std::string child_json;
        prop_dump_json(*child, child_json);
        bool ok;
        field.set("value", json::Value::parse(child_json, &ok));
        break;
      }
      case PropType::Custom: {
        const auto* slot = reinterpret_cast<const CustomSlot*>(ptr);
        field.set("custom_type_name", json::Value::make_string(info.custom_type_name));
        bool ok;
        field.set("value", json::Value::parse(slot->to_json(), &ok));
        break;
      }
      default: field.set("value", detail::pod_to_json(info.type, ptr)); break;
    }

    root.set(info.name, field);
  }

  out = root.dump();
  return true;
}

// load: prop_dump_jsonで保存されたJSON文字列からPropを復元する。JSON内に
// 埋め込まれた各フィールドの"type"タグを頼りに、新規フィールドとして
// Propへ追加していく。
inline bool prop_load_json(Prop& prop, const std::string& text) {
  bool ok           = false;
  json::Value root = json::Value::parse(text, &ok);
  if(!ok || !root.is_object()) return false;

  for(const std::string& name : root.keys()) {
    json::Value field = root.get(name);
    auto type          = static_cast<PropType>(field.get("type").as_int());
    uint32_t version    = static_cast<uint32_t>(field.get("version").as_int());
    json::Value value  = field.get("value");

    switch(type) {
      case PropType::Str: prop.set<Str>(name.c_str(), Str(value.as_string().c_str())); break;
      case PropType::Path: prop.set<Path>(name.c_str(), Path(Str(value.as_string().c_str()))); break;
      case PropType::Binary: {
        std::vector<uint8_t> bin;
        bin.reserve(value.size());
        for(size_t i = 0; i < value.size(); i++) bin.push_back(static_cast<uint8_t>(value.get(i).as_int()));
        prop.set<std::vector<uint8_t>>(name.c_str(), bin);
        break;
      }
      case PropType::Nested: {
        Prop child;
        prop_load_json(child, value.dump());
        prop.set_child(name.c_str(), child);
        break;
      }
      case PropType::Custom: {
        std::string custom_type_name = field.get("custom_type_name").as_string();
        CustomSlot slot                = CustomSlot::make_from_json(custom_type_name.c_str(), value.dump());
        prop.adopt_custom_slot(name.c_str(), custom_type_name.c_str(), std::move(slot));
        break;
      }
      default: {
        // POD型は一旦ゼロ初期化したバッファに変換してからset_raw_podで反映する
        std::vector<uint8_t> tmp(prop_type_size(type), 0);
        detail::json_to_pod(type, value, tmp.data());
        prop.set_raw_pod(name.c_str(), type, tmp.data(), tmp.size());
        break;
      }
    }
    // versionは元のPropInfoに反映する(以後このversionを基準にバイナリload時の
    // per-fieldバージョン比較が行われる)。
    prop.set_field_version(name.c_str(), version);
  }

  return true;
}

// format_version不一致・見つからないキー・per-fieldのversion不一致を検知した際に
// 呼ばれるフォールバック。Phase6でJSON経由のロードをここに接続する。
using PropLoadFallback = std::function<bool(Prop&, const std::vector<uint8_t>&)>;

constexpr uint32_t PROP_FILEFLAG_HAS_JSON_BLOCK = 1u << 1;

// dump: is_pointer==falseなフィールドのみを対象に、Prop::raw_data()の該当バイト
// 範囲をそのままファイル用バイナリへコピーする。ポインタ型フィールドは非対応
// (skipped_fieldsが渡されていれば、そこにフィールド名を積んで呼び出し元へ知らせる)。
// 併せて、新旧混在ファイルでも1ファイルで完結できるよう、全フィールドの
// JSONダンプ([JsonBlockHeader]+[JsonBlock])を末尾に常に併載する。
inline bool prop_dump_binary(const Prop& prop, std::vector<uint8_t>& out, std::vector<std::string>* skipped_fields = nullptr) {
  std::vector<const PropInfo*> pod_fields;
  for(const auto& info : prop.infos()) {
    if(info.is_pointer) {
      if(skipped_fields) skipped_fields->push_back(info.name);
      continue;
    }
    pod_fields.push_back(&info);
  }

  std::string json_text;
  prop_dump_json(prop, json_text);

  PropFileHeader header;
  header.entry_count = static_cast<uint32_t>(pod_fields.size());
  header.flags        = PROP_FILEFLAG_HAS_JSON_BLOCK;

  // DataBlock内の各エントリのoffsetを事前に計算する(alignof(T)に揃える)。
  std::vector<uint32_t> data_offsets(pod_fields.size());
  size_t data_cursor = 0;
  for(size_t i = 0; i < pod_fields.size(); i++) {
    size_t align    = prop_type_align(pod_fields[i]->type);
    data_cursor     = align_up(data_cursor, align);
    data_offsets[i] = static_cast<uint32_t>(data_cursor);
    data_cursor += pod_fields[i]->size;
  }

  size_t entry_table_offset = sizeof(PropFileHeader);
  size_t data_block_offset  = entry_table_offset + pod_fields.size() * sizeof(PropEntryHeader);
  size_t json_header_offset = data_block_offset + data_cursor;
  size_t json_block_offset  = json_header_offset + sizeof(uint32_t);
  size_t total_size          = json_block_offset + json_text.size();

  out.assign(total_size, 0);
  std::memcpy(out.data(), &header, sizeof(header));

  const uint8_t* prop_data = prop.raw_data();
  for(size_t i = 0; i < pod_fields.size(); i++) {
    const PropInfo* info = pod_fields[i];

    PropEntryHeader eh;
    std::strncpy(eh.name, info->name, sizeof(eh.name) - 1);
    eh.prop_type         = static_cast<uint16_t>(info->type);
    eh.prop_info_version = static_cast<uint16_t>(info->version);
    eh.data_offset       = data_offsets[i];
    eh.data_size         = static_cast<uint32_t>(info->size);
    eh.flags             = 0;
    std::memcpy(out.data() + entry_table_offset + i * sizeof(PropEntryHeader), &eh, sizeof(eh));

    std::memcpy(out.data() + data_block_offset + data_offsets[i], prop_data + info->offset, info->size);
  }

  auto json_size = static_cast<uint32_t>(json_text.size());
  std::memcpy(out.data() + json_header_offset, &json_size, sizeof(json_size));
  std::memcpy(out.data() + json_block_offset, json_text.data(), json_text.size());

  return true;
}

// load: format_versionとエンディアンタグ、各エントリのper-fieldバージョンを検証し、
// 問題なければPOD型フィールドをそのままPropへ復元する。
// 検証に失敗した場合(フォーマット不一致、破損データ、per-fieldバージョン不一致など)は、
// fallbackが明示的に渡されていればそれを、渡されていなければファイルに併載された
// [JsonBlock](prop_load_json)を自動的に使ってフォールバックする。
inline bool prop_load_binary(Prop& prop, const std::vector<uint8_t>& bytes, const PropLoadFallback& fallback = nullptr) {
  auto do_fallback = [&]() -> bool {
    if(fallback) return fallback(prop, bytes);

    // fallback未指定時は、ファイルに併載されたJsonBlockを自動的に使う。
    if(bytes.size() < sizeof(PropFileHeader)) return false;
    PropFileHeader h;
    std::memcpy(&h, bytes.data(), sizeof(h));
    if(std::memcmp(h.magic, "CPRP", 4) != 0) return false;
    if((h.flags & PROP_FILEFLAG_HAS_JSON_BLOCK) == 0) return false;

    size_t entry_table_offset = sizeof(PropFileHeader);
    size_t entry_table_size    = static_cast<size_t>(h.entry_count) * sizeof(PropEntryHeader);
    size_t data_block_offset  = entry_table_offset + entry_table_size;
    if(bytes.size() < data_block_offset) return false;

    // DataBlockのサイズはEntryTableの各エントリから再計算する必要があるため、
    // 最大の(data_offset+data_size)を探して境界とする。
    size_t data_block_size = 0;
    for(uint32_t i = 0; i < h.entry_count; i++) {
      size_t eh_off = entry_table_offset + i * sizeof(PropEntryHeader);
      if(eh_off + sizeof(PropEntryHeader) > bytes.size()) return false;
      PropEntryHeader eh;
      std::memcpy(&eh, bytes.data() + eh_off, sizeof(eh));
      data_block_size = std::max(data_block_size, static_cast<size_t>(eh.data_offset) + eh.data_size);
    }

    size_t json_header_offset = data_block_offset + data_block_size;
    if(json_header_offset + sizeof(uint32_t) > bytes.size()) return false;
    uint32_t json_size = 0;
    std::memcpy(&json_size, bytes.data() + json_header_offset, sizeof(json_size));
    size_t json_block_offset = json_header_offset + sizeof(uint32_t);
    if(json_block_offset + json_size > bytes.size()) return false;

    std::string json_text(reinterpret_cast<const char*>(bytes.data() + json_block_offset), json_size);
    return prop_load_json(prop, json_text);
  };

  if(bytes.size() < sizeof(PropFileHeader)) return do_fallback();

  PropFileHeader header;
  std::memcpy(&header, bytes.data(), sizeof(header));

  if(std::memcmp(header.magic, "CPRP", 4) != 0) return do_fallback();
  if(header.endianness_tag != PROP_BINARY_ENDIANNESS_TAG) return do_fallback();
  if(header.format_version != PROP_BINARY_FORMAT_VERSION) return do_fallback();

  size_t entry_table_offset = sizeof(PropFileHeader);
  size_t entry_table_size    = static_cast<size_t>(header.entry_count) * sizeof(PropEntryHeader);
  if(bytes.size() < entry_table_offset + entry_table_size) return do_fallback();
  size_t data_block_offset = entry_table_offset + entry_table_size;

  for(uint32_t i = 0; i < header.entry_count; i++) {
    PropEntryHeader eh;
    std::memcpy(&eh, bytes.data() + entry_table_offset + i * sizeof(PropEntryHeader), sizeof(eh));

    if((eh.flags & 0x1u) != 0) continue; // is_pointerなフィールドはPhase7で対応、Phase5では読み飛ばす

    // Prop側に既に同名フィールドが存在し、バージョンが食い違う場合は
    // フォーマット変更とみなしフォールバックへ丸ごと処理を委譲する。
    const PropInfo* existing = find_info(prop.infos(), eh.name);
    if(existing && existing->version != eh.prop_info_version) return do_fallback();

    size_t src_offset = data_block_offset + eh.data_offset;
    if(src_offset + eh.data_size > bytes.size()) return do_fallback(); // 破損データからのOOB読み取りを防ぐ

    prop.set_raw_pod(eh.name, static_cast<PropType>(eh.prop_type), bytes.data() + src_offset, eh.data_size);
  }

  return true;
}

} // namespace cutil
