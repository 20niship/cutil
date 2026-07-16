#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include <cutil/prop.hpp>

// cutil::prop_dump_binary / cutil::prop_load_binary: Prop を独自バイナリ形式で
// dump/loadする(Phase 5: POD型限定、ファイルI/O)。
//
// バイナリフォーマット:
//   [FileHeader]
//     magic[4]        = "CPRP"
//     format_version  : u32  (このフォーマット自体のバージョン)
//     endianness_tag  : u32  (= 0x01020304。ホストと不一致ならload失敗として扱う。
//                              バイトスワップには対応しない)
//     entry_count     : u32
//     flags           : u32  (bit0 = has_pointer_blob。Phase7で使用)
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

// format_version不一致・見つからないキー・per-fieldのversion不一致を検知した際に
// 呼ばれるフォールバック。Phase6でJSON経由のロードをここに接続する。
using PropLoadFallback = std::function<bool(Prop&, const std::vector<uint8_t>&)>;

// dump: is_pointer==falseなフィールドのみを対象に、Prop::raw_data()の該当バイト
// 範囲をそのままファイル用バイナリへコピーする。ポインタ型フィールドは非対応
// (skipped_fieldsが渡されていれば、そこにフィールド名を積んで呼び出し元へ知らせる)。
inline bool prop_dump_binary(const Prop& prop, std::vector<uint8_t>& out, std::vector<std::string>* skipped_fields = nullptr) {
  std::vector<const PropInfo*> pod_fields;
  for(const auto& info : prop.infos()) {
    if(info.is_pointer) {
      if(skipped_fields) skipped_fields->push_back(info.name);
      continue;
    }
    pod_fields.push_back(&info);
  }

  PropFileHeader header;
  header.entry_count = static_cast<uint32_t>(pod_fields.size());

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
  size_t total_size          = data_block_offset + data_cursor;

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

  return true;
}

// load: format_versionとエンディアンタグ、各エントリのper-fieldバージョンを検証し、
// 問題なければPOD型フィールドをそのままPropへ復元する。
// 検証に失敗した場合(フォーマット不一致、破損データ、per-fieldバージョン不一致など)は
// fallbackが渡されていればそれを呼び出し、その戻り値を返す。fallbackが無ければfalse。
inline bool prop_load_binary(Prop& prop, const std::vector<uint8_t>& bytes, const PropLoadFallback& fallback = nullptr) {
  auto do_fallback = [&]() -> bool {
    if(fallback) return fallback(prop, bytes);
    return false;
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
