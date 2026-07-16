#pragma once

#include <memory>
#include <string>

// cutil::json: nlohmann/json への薄いラッパー
// STBスタイル: 通常include時は<nlohmann/json.hpp>をincludeせず宣言のみ公開する。
// CUTIL_IMPLEMENTATION を定義した唯一のTUでのみ<nlohmann/json.hpp>をincludeし実装する。
// (cutil::json::Value は不透明型 = pimpl経由で実装を隠蔽する)
//
// 注意: operator[]のような参照を返すAPIはpimpl越しに安全に提供するのが難しいため、
// あえて提供しない。オブジェクト/配列へのアクセスは常に値のコピーで行う
// (get/set)。JSONフォールバック用途(頻度が低い)を想定しているため
// このコストは許容する。

namespace cutil::json {

class Value {
public:
  Value();
  Value(const Value& other);
  Value(Value&& other) noexcept;
  Value& operator=(const Value& other);
  Value& operator=(Value&& other) noexcept;
  ~Value();

  static Value parse(const std::string& text, bool* ok = nullptr);
  [[nodiscard]] std::string dump(int indent = -1) const;

  [[nodiscard]] bool is_null() const;
  [[nodiscard]] bool is_object() const;
  [[nodiscard]] bool is_array() const;

  // オブジェクトのフィールドアクセス(値はコピーで取得/格納する)
  [[nodiscard]] bool contains(const std::string& key) const;
  [[nodiscard]] Value get(const std::string& key) const;
  void set(const std::string& key, const Value& v);

  // 配列アクセス
  [[nodiscard]] size_t size() const;
  [[nodiscard]] Value get(size_t index) const;
  void push_back(const Value& v);

  // 基本型からの構築/取得
  static Value make_bool(bool v);
  static Value make_int(long long v);
  static Value make_double(double v);
  static Value make_string(const std::string& v);
  static Value make_object();
  static Value make_array();

  [[nodiscard]] bool as_bool() const;
  [[nodiscard]] long long as_int() const;
  [[nodiscard]] double as_double() const;
  [[nodiscard]] std::string as_string() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  explicit Value(std::unique_ptr<Impl> impl);
};

} // namespace cutil::json

#ifdef CUTIL_IMPLEMENTATION
#include <nlohmann/json.hpp>

namespace cutil::json {

struct Value::Impl {
  nlohmann::json j;
};

Value::Value() : impl_(std::make_unique<Impl>()) {}
Value::Value(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Value::Value(const Value& other) : impl_(std::make_unique<Impl>(*other.impl_)) {}
Value::Value(Value&& other) noexcept = default;
Value::~Value()                      = default;

Value& Value::operator=(const Value& other) {
  if(this != &other) impl_ = std::make_unique<Impl>(*other.impl_);
  return *this;
}
Value& Value::operator=(Value&& other) noexcept = default;

Value Value::parse(const std::string& text, bool* ok) {
  auto impl = std::make_unique<Impl>();
  try {
    impl->j = nlohmann::json::parse(text);
    if(ok) *ok = true;
  } catch(const nlohmann::json::parse_error&) {
    impl->j = nlohmann::json();
    if(ok) *ok = false;
  }
  return Value(std::move(impl));
}

std::string Value::dump(int indent) const { return impl_->j.dump(indent); }

bool Value::is_null() const { return impl_->j.is_null(); }
bool Value::is_object() const { return impl_->j.is_object(); }
bool Value::is_array() const { return impl_->j.is_array(); }

bool Value::contains(const std::string& key) const { return impl_->j.is_object() && impl_->j.contains(key); }

Value Value::get(const std::string& key) const {
  auto impl = std::make_unique<Impl>();
  impl->j   = impl_->j.at(key);
  return Value(std::move(impl));
}

void Value::set(const std::string& key, const Value& v) { impl_->j[key] = v.impl_->j; }

size_t Value::size() const { return impl_->j.size(); }

Value Value::get(size_t index) const {
  auto impl = std::make_unique<Impl>();
  impl->j   = impl_->j.at(index);
  return Value(std::move(impl));
}

void Value::push_back(const Value& v) { impl_->j.push_back(v.impl_->j); }

Value Value::make_bool(bool v) {
  auto impl = std::make_unique<Impl>();
  impl->j   = v;
  return Value(std::move(impl));
}
Value Value::make_int(long long v) {
  auto impl = std::make_unique<Impl>();
  impl->j   = v;
  return Value(std::move(impl));
}
Value Value::make_double(double v) {
  auto impl = std::make_unique<Impl>();
  impl->j   = v;
  return Value(std::move(impl));
}
Value Value::make_string(const std::string& v) {
  auto impl = std::make_unique<Impl>();
  impl->j   = v;
  return Value(std::move(impl));
}
Value Value::make_object() {
  auto impl = std::make_unique<Impl>();
  impl->j   = nlohmann::json::object();
  return Value(std::move(impl));
}
Value Value::make_array() {
  auto impl = std::make_unique<Impl>();
  impl->j   = nlohmann::json::array();
  return Value(std::move(impl));
}

bool Value::as_bool() const { return impl_->j.get<bool>(); }
long long Value::as_int() const { return impl_->j.get<long long>(); }
double Value::as_double() const { return impl_->j.get<double>(); }
std::string Value::as_string() const { return impl_->j.get<std::string>(); }

} // namespace cutil::json
#endif
