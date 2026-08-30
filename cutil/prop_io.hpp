#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include <cutil/prop.hpp>

// cutil::prop_dump_binary/prop_load_binary: PropClass(Trivial/Indirect/Dynamic)に基づく新バイナリフォーマット(Issue #21)。
//
// 型ごとに1回だけ[SchemaSection]へスキーマを記録.
// [EntryTable]は「名前 + schema参照index + offset/size」のみの軽量レコードにすることでこれを解消する。
// JsonBlockの常時併載は廃止しDynamic型のみBlobBlock内に個別のjsonを持つ。
//
// ファイルレイアウト:
//   [FileHeader]     magic/format_version/endianness_tag/schema_count/entry_count
//   [SchemaSection]  型ごとに1回: type_id/version/size/フィールド一覧(name+type_id+offset+size)
//   [EntryTable]     値ごと: name + schema_index + DataBlock内のoffset/size
//   [DataBlock]      Trivialは実データそのまま、Indirect/Dynamicは{blob_offset,blob_size}の8バイト固定ヘッダのみ
//   [BlobBlock]      Indirectの可変長部(配列要素・文字列本体・再帰的な子構造)、Dynamicはto_json()の結果

namespace cutil {

constexpr uint32_t PROP_BINARY_FORMAT_VERSION = 3;
constexpr uint32_t PROP_BINARY_ENDIANNESS_TAG = 0x01020304;

struct PropFileHeader {
  char magic[4]           = {'C', 'P', 'R', '2'};
  uint32_t format_version = PROP_BINARY_FORMAT_VERSION;
  uint32_t endianness_tag = PROP_BINARY_ENDIANNESS_TAG;
  uint32_t schema_count   = 0;
  uint32_t entry_count    = 0;
};

struct PropSchemaFieldDesc {
  char name[32]    = {};
  char type_id[64] = {};
  uint32_t offset  = 0;
  uint32_t size    = 0;
};

struct PropSchemaEntry {
  char type_id[64]     = {};
  uint32_t version     = 1;
  uint32_t size        = 0; // Trivial型のversion不一致時、レイアウト同一かどうかの判定に使う
  uint32_t field_count = 0;
};

struct PropValueEntry {
  char name[32]         = {};
  uint32_t schema_index = 0;
  uint32_t data_offset  = 0; // [DataBlock]内でのoffset
  uint32_t data_size    = 0;
};

namespace detail {

inline void append_bytes(std::vector<uint8_t>& out, const void* data, size_t n) {
  const auto* p = reinterpret_cast<const uint8_t*>(data);
  out.insert(out.end(), p, p + n);
}
inline void append_u32(std::vector<uint8_t>& out, uint32_t v) { append_bytes(out, &v, sizeof(v)); }

// typeの値objをバイナリでblob_outへ再帰的に書き込む。Trivial要素の配列は1回のappend_bytes(一括memcpy相当)、Indirect要素はelement_typeを再帰呼び出しする(B-in-Bはここで要素ごとループになる)。
inline void write_value_binary(const PropInfo* type, const void* obj, std::vector<uint8_t>& blob_out) {
  if(!type->fields.empty()) {
    for(const auto& f : type->fields) {
      const auto* fptr = reinterpret_cast<const uint8_t*>(obj) + f.offset;
      if(f.type->klass == PropClass::Trivial) {
        append_bytes(blob_out, fptr, f.type->size);
      } else {
        write_value_binary(f.type, fptr, blob_out);
      }
    }
    return;
  }
  if(type->element_type) {
    size_t n = type->seq_size(obj);
    append_u32(blob_out, static_cast<uint32_t>(n));
    if(type->element_type->klass == PropClass::Trivial) {
      if(n) append_bytes(blob_out, type->seq_data(obj), n * type->element_type->size);
    } else {
      for(size_t i = 0; i < n; i++) write_value_binary(type->element_type, type->seq_at(obj, i), blob_out);
    }
    return;
  }
  // fields/element_typeどちらも持たないIndirect/Dynamic(CustomSlot, Prop等)はto_json経由にフォールバックする。
  std::string json = type->to_json(obj);
  append_u32(blob_out, static_cast<uint32_t>(json.size()));
  append_bytes(blob_out, json.data(), json.size());
}

// blob内に埋め込まれた長さ/要素数はファイル由来で信用できないため、読む前に必ずblob_sizeへ収まるか確認する。収まらなければ投げる。
struct BinaryTruncated {};
inline void check_blob_bounds(size_t offset, size_t need, size_t blob_size) {
  if(offset > blob_size || need > blob_size - offset) throw BinaryTruncated{};
}

// blob[offset..]からtypeの値をobj(未構築)へ再構築する。読み終えた次のoffsetを返す。範囲外読み出しはBinaryTruncatedを投げる。
inline size_t read_value_binary(const PropInfo* type, void* obj, const uint8_t* blob, size_t offset, size_t blob_size) {
  if(!type->fields.empty()) {
    if(type->default_ctor)
      type->default_ctor(obj);
    else
      std::memset(obj, 0, type->size);
    for(const auto& f : type->fields) {
      auto* fptr = reinterpret_cast<uint8_t*>(obj) + f.offset;
      if(f.type->klass == PropClass::Trivial) {
        check_blob_bounds(offset, f.type->size, blob_size);
        std::memcpy(fptr, blob + offset, f.type->size);
        offset += f.type->size;
      } else {
        if(f.type->dtor) f.type->dtor(fptr); // default_ctorが構築した仮の値を破棄してから再構築する
        offset = read_value_binary(f.type, fptr, blob, offset, blob_size);
      }
    }
    return offset;
  }
  if(type->element_type) {
    uint32_t n = 0;
    check_blob_bounds(offset, sizeof(n), blob_size);
    std::memcpy(&n, blob + offset, sizeof(n));
    offset += sizeof(n);
    if(type->default_ctor)
      type->default_ctor(obj);
    else
      std::memset(obj, 0, type->size);
    if(type->element_type->klass == PropClass::Trivial) {
      check_blob_bounds(offset, static_cast<size_t>(n) * type->element_type->size, blob_size);
      type->seq_assign_raw(obj, blob + offset, n);
      offset += static_cast<size_t>(n) * type->element_type->size;
    } else {
      std::vector<uint8_t> elem_storage(type->element_type->size);
      for(uint32_t i = 0; i < n; i++) {
        offset = read_value_binary(type->element_type, elem_storage.data(), blob, offset, blob_size);
        type->seq_push_back_copy(obj, elem_storage.data());
        if(type->element_type->dtor) type->element_type->dtor(elem_storage.data());
      }
    }
    return offset;
  }
  // fields/element_typeどちらも持たないIndirect/Dynamic(CustomSlot, Prop等)はfrom_json経由にフォールバックする。
  uint32_t len = 0;
  check_blob_bounds(offset, sizeof(len), blob_size);
  std::memcpy(&len, blob + offset, sizeof(len));
  offset += sizeof(len);
  check_blob_bounds(offset, len, blob_size);
  std::string json(reinterpret_cast<const char*>(blob + offset), len);
  offset += len;
  type->from_json(obj, json); // from_jsonはplacement-newで構築する規約
  return offset;
}

} // namespace detail

// dump: 全フィールドを人間可読なJSON文字列へ変換する。各フィールドの型のto_json()を呼ぶだけで済み型ごとのswitch分岐は不要になった。
inline bool prop_dump_json(const Prop& prop, std::string& out) {
  const uint8_t* base = prop.raw_data();
  std::string body    = "{";
  bool first          = true;
  for(const auto& f : prop.fields()) {
    if(!first) body += ",";
    first = false;
    body += detail::json_quote(f.name) + ":{" + detail::json_quote("type_id") + ":" + detail::json_quote(f.type->id) + "," + detail::json_quote("version") + ":" + std::to_string(f.type->version) + "," + detail::json_quote("value") + ":" + f.type->to_json(base + f.offset) + "}";
  }
  body += "}";
  out = body;
  return true;
}

// load: JSON文字列からPropを復元する。フィールドのtype_idをPropInfoRegistryで引き直しfrom_json()で値を再構築する。
inline bool prop_load_json(Prop& prop, const std::string& text) {
  std::string trimmed = detail::json_trim(text);
  if(trimmed.size() < 2 || trimmed.front() != '{' || trimmed.back() != '}') return false;
  try {
    for(const auto& part : detail::json_split_top_level(text)) {
      std::string field_name, field_body;
      if(!detail::json_split_kv(part, field_name, field_body)) continue;

      std::string type_id, value_json;
      for(const auto& kv : detail::json_split_top_level(field_body)) {
        std::string k, v;
        if(!detail::json_split_kv(kv, k, v)) continue;
        if(k == "type_id")
          type_id = detail::json_unquote(v);
        else if(k == "value")
          value_json = v;
      }

      const PropInfo* type = PropInfoRegistry::instance().find(type_id);
      if(!type || !type->from_json) continue; // 現行コードに存在しない/from_json未登録の型は復元不能としてスキップする

      std::vector<uint8_t> storage(type->size);
      if(!type->from_json(storage.data(), value_json)) continue;

      if(type->klass == PropClass::Trivial) {
        prop.set_raw_pod_by_info(field_name.c_str(), type, storage.data());
      } else {
        prop.adopt_raw_by_info(field_name.c_str(), type, storage.data());
      }
    }
  } catch(const std::exception&) {
    // 同名フィールドが型違いで重複するなど壊れたJSON特有の状態はエラーとして扱う(set/adopt_raw_by_infoはlogic_errorを投げる)
    return false;
  }
  return true;
}

namespace detail {
// prop_info_of<Prop>()はprop_dump_json/prop_load_json定義前にbindできないため、このファイル読み込み時に一度だけ遅延バインドする。
inline bool bind_prop_info_of_prop_to_json() {
  auto* info    = const_cast<PropInfo*>(prop_info_of<Prop>());
  info->to_json = [](const void* obj) -> std::string {
    std::string json;
    prop_dump_json(*reinterpret_cast<const Prop*>(obj), json);
    return json;
  };
  info->from_json = [](void* obj, const std::string& text) -> bool {
    new(obj) Prop();
    return prop_load_json(*reinterpret_cast<Prop*>(obj), text);
  };
  return true;
}
inline bool prop_info_of_prop_bound = bind_prop_info_of_prop_to_json();
} // namespace detail

using PropLoadFallback = std::function<bool(Prop&, const std::vector<uint8_t>&)>;

inline bool prop_dump_binary(const Prop& prop, std::vector<uint8_t>& out) {
  const auto& fields  = prop.fields();
  const uint8_t* base = prop.raw_data();

  // 1. 型収集(重複排除): 値ごとに繰り返さず型ごとに1回だけSchemaSectionへ記録する。
  std::vector<const PropInfo*> schemas;
  auto schema_index_of = [&](const PropInfo* t) -> uint32_t {
    for(size_t i = 0; i < schemas.size(); i++)
      if(schemas[i] == t) return static_cast<uint32_t>(i);
    schemas.push_back(t);
    return static_cast<uint32_t>(schemas.size() - 1);
  };
  std::vector<uint32_t> field_schema_index(fields.size());
  for(size_t i = 0; i < fields.size(); i++) field_schema_index[i] = schema_index_of(fields[i].type);

  // 2. DataBlock/BlobBlock構築
  std::vector<uint8_t> data_block, blob_block;
  std::vector<uint32_t> data_offsets(fields.size());
  for(size_t i = 0; i < fields.size(); i++) {
    const auto& f       = fields[i];
    const uint8_t* fptr = base + f.offset;
    data_offsets[i]     = static_cast<uint32_t>(data_block.size());
    if(f.type->klass == PropClass::Trivial) {
      detail::append_bytes(data_block, fptr, f.type->size);
    } else {
      uint32_t blob_offset = static_cast<uint32_t>(blob_block.size());
      detail::write_value_binary(f.type, fptr, blob_block);
      uint32_t blob_size = static_cast<uint32_t>(blob_block.size()) - blob_offset;
      detail::append_u32(data_block, blob_offset);
      detail::append_u32(data_block, blob_size);
    }
  }

  // 3. ヘッダ+SchemaSection+EntryTable+DataBlock+BlobBlockを連結
  PropFileHeader header;
  header.schema_count = static_cast<uint32_t>(schemas.size());
  header.entry_count  = static_cast<uint32_t>(fields.size());

  out.clear();
  detail::append_bytes(out, &header, sizeof(header));

  for(const auto* s : schemas) {
    PropSchemaEntry se;
    std::strncpy(se.type_id, s->id, sizeof(se.type_id) - 1);
    se.version     = s->version;
    se.size        = static_cast<uint32_t>(s->size);
    se.field_count = static_cast<uint32_t>(s->fields.size());
    detail::append_bytes(out, &se, sizeof(se));
    for(const auto& f : s->fields) {
      PropSchemaFieldDesc fd;
      std::strncpy(fd.name, f.name, sizeof(fd.name) - 1);
      std::strncpy(fd.type_id, f.type->id, sizeof(fd.type_id) - 1);
      fd.offset = static_cast<uint32_t>(f.offset);
      fd.size   = static_cast<uint32_t>(f.type->size);
      detail::append_bytes(out, &fd, sizeof(fd));
    }
  }

  for(size_t i = 0; i < fields.size(); i++) {
    PropValueEntry ve;
    std::strncpy(ve.name, fields[i].name, sizeof(ve.name) - 1);
    ve.schema_index = field_schema_index[i];
    ve.data_offset  = data_offsets[i];
    ve.data_size    = fields[i].type->klass == PropClass::Trivial ? static_cast<uint32_t>(fields[i].type->size) : 8u; // Indirect/Dynamicは{blob_offset,blob_size}固定8バイト
    detail::append_bytes(out, &ve, sizeof(ve));
  }

  detail::append_bytes(out, data_block.data(), data_block.size());
  detail::append_bytes(out, blob_block.data(), blob_block.size());
  return true;
}

inline bool prop_load_binary(Prop& prop, const std::vector<uint8_t>& bytes, const PropLoadFallback& fallback = nullptr) {
  auto do_fallback = [&]() -> bool { return fallback ? fallback(prop, bytes) : false; };

  if(bytes.size() < sizeof(PropFileHeader)) return do_fallback();
  PropFileHeader header;
  std::memcpy(&header, bytes.data(), sizeof(header));
  if(std::memcmp(header.magic, "CPR2", 4) != 0) return do_fallback();
  if(header.endianness_tag != PROP_BINARY_ENDIANNESS_TAG) return do_fallback();
  if(header.format_version != PROP_BINARY_FORMAT_VERSION) return do_fallback();

  size_t cursor = sizeof(PropFileHeader);

  // SchemaSection読み込み
  struct LoadedSchema {
    PropSchemaEntry entry;
    std::vector<PropSchemaFieldDesc> fields;
  };
  // schema_count等は壊れたファイルだと巨大値になり得るためvector(count)で先に確保せずpush_backする。
  std::vector<LoadedSchema> schemas;
  schemas.reserve(std::min<size_t>(header.schema_count, bytes.size()));
  for(uint32_t i = 0; i < header.schema_count; i++) {
    if(cursor + sizeof(PropSchemaEntry) > bytes.size()) return do_fallback();
    LoadedSchema ls;
    std::memcpy(&ls.entry, bytes.data() + cursor, sizeof(PropSchemaEntry));
    cursor += sizeof(PropSchemaEntry);
    if(static_cast<uint64_t>(ls.entry.field_count) * sizeof(PropSchemaFieldDesc) > bytes.size() - cursor) return do_fallback();
    ls.fields.resize(ls.entry.field_count);
    for(uint32_t j = 0; j < ls.entry.field_count; j++) {
      std::memcpy(&ls.fields[j], bytes.data() + cursor, sizeof(PropSchemaFieldDesc));
      cursor += sizeof(PropSchemaFieldDesc);
    }
    schemas.push_back(std::move(ls));
  }

  // EntryTable読み込み
  std::vector<PropValueEntry> entries;
  entries.reserve(std::min<size_t>(header.entry_count, bytes.size()));
  for(uint32_t i = 0; i < header.entry_count; i++) {
    if(cursor + sizeof(PropValueEntry) > bytes.size()) return do_fallback();
    PropValueEntry ve;
    std::memcpy(&ve, bytes.data() + cursor, sizeof(PropValueEntry));
    cursor += sizeof(PropValueEntry);
    entries.push_back(ve);
  }

  size_t data_block_offset = cursor;
  size_t data_block_size   = 0;
  for(const auto& e : entries) data_block_size = std::max(data_block_size, static_cast<size_t>(e.data_offset) + e.data_size);
  size_t blob_block_offset = data_block_offset + data_block_size;
  if(blob_block_offset > bytes.size()) return do_fallback();
  const uint8_t* blob = bytes.data() + blob_block_offset;
  size_t blob_size    = bytes.size() - blob_block_offset;

  try {
    for(uint32_t i = 0; i < header.entry_count; i++) {
      const auto& ve = entries[i];
      if(ve.schema_index >= schemas.size()) return do_fallback(); // 破損したschema_indexで範囲外参照しない
      const auto& file_schema   = schemas[ve.schema_index];
      const PropInfo* live_type = PropInfoRegistry::instance().find(file_schema.entry.type_id);

      size_t entry_data_offset = data_block_offset + ve.data_offset;
      if(entry_data_offset + ve.data_size > bytes.size()) return do_fallback();

      if(!live_type) continue; // 現行コードに存在しない型はこのフィールドのみスキップする

      if(live_type->version == file_schema.entry.version) {
        // fast path: 現行スキーマとバイト完全互換なのでそのまま読み込む
        std::vector<uint8_t> storage(live_type->size);
        if(live_type->klass == PropClass::Trivial) {
          std::memcpy(storage.data(), bytes.data() + entry_data_offset, live_type->size);
        } else {
          uint32_t blob_offset = 0, blob_len = 0;
          std::memcpy(&blob_offset, bytes.data() + entry_data_offset, 4);
          std::memcpy(&blob_len, bytes.data() + entry_data_offset + 4, 4);
          if(blob_offset + blob_len > blob_size) return do_fallback();
          detail::read_value_binary(live_type, storage.data(), blob, blob_offset, blob_size);
        }
        if(live_type->klass == PropClass::Trivial) {
          prop.set_raw_pod_by_info(ve.name, live_type, storage.data());
        } else {
          prop.adopt_raw_by_info(ve.name, live_type, storage.data());
        }
      } else if(live_type->klass == PropClass::Trivial) {
        // Trivial型はversion不一致でもバイナリレイアウト(size)が変わっていなければそのままmemcpyで復元できる。
        if(file_schema.entry.size != live_type->size) continue;
        std::vector<uint8_t> storage(live_type->size);
        std::memcpy(storage.data(), bytes.data() + entry_data_offset, live_type->size);
        prop.set_raw_pod_by_info(ve.name, live_type, storage.data());
      } else {
        // slow path(型/エントリ単位のフォールバック): 旧schemaのTrivialフィールドのみ名前一致で復元、Indirect/Dynamicの変更は次段階として復元不能でスキップする。
        if(file_schema.fields.empty() || live_type->fields.empty()) continue;

        std::vector<uint8_t> storage(live_type->size);
        if(live_type->default_ctor)
          live_type->default_ctor(storage.data());
        else
          std::memset(storage.data(), 0, live_type->size);

        uint32_t blob_offset = 0, blob_len = 0;
        std::memcpy(&blob_offset, bytes.data() + entry_data_offset, 4);
        std::memcpy(&blob_len, bytes.data() + entry_data_offset + 4, 4);
        if(blob_offset + blob_len > blob_size) continue;
        const uint8_t* old_blob = blob + blob_offset;

        for(const auto& old_f : file_schema.fields) {
          const PropInfo::Field* cur_f = live_type->find_field(old_f.name);
          if(!cur_f || cur_f->type->klass != PropClass::Trivial) continue;
          if(std::strncmp(cur_f->type->id, old_f.type_id, sizeof(old_f.type_id)) != 0) continue;
          if(old_f.offset + old_f.size > blob_len) continue;
          std::memcpy(storage.data() + cur_f->offset, old_blob + old_f.offset, old_f.size);
        }
        prop.adopt_raw_by_info(ve.name, live_type, storage.data());
      }
    }
  } catch(const detail::BinaryTruncated&) {
    return do_fallback(); // blob内の長さ/要素数フィールドが壊れており安全に読み切れなかった
  } catch(const std::exception&) {
    return do_fallback(); // 同名フィールドが型違いで重複するなど壊れたEntryTable特有の状態
  }

  return true;
}

} // namespace cutil
