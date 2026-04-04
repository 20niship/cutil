#pragma once

#include "string.hpp"
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nanobind {
namespace detail {

// Type caster for cutil::Str - converts Python str <-> cutil::Str
template <> struct type_caster<cutil::Str> {
  NB_TYPE_CASTER(cutil::Str, const_name("str"));

  // Python から C++ へ: Python str -> cutil::Str
  bool from_python(handle src, uint8_t flags, cleanup_list* list) noexcept {
    // std::string へ変換
    type_caster<std::string> caster;
    if (!caster.from_python(src, flags, list)) {
      return false;
    }
    // std::string から cutil::Str へ変換
    value = cutil::Str(caster.value);
    return true;
  }

  // C++ から Python へ: cutil::Str -> Python str
  static handle from_cpp(const cutil::Str& src, rv_policy, cleanup_list*) {
    // cutil::Str を std::string に変換してから Python str に
    std::string str_value = src.to_std_string();
    return PyUnicode_FromStringAndSize(str_value.data(), (Py_ssize_t)str_value.size());
  }
};

} // namespace detail
} // namespace nanobind

namespace cutil {

// nanobind統合時の補助関数

/// 任意の Python オブジェクトから Str へ変換
/// nanobind の lambda 内で str(obj) を使用する場合
inline Str from_python_str(const std::string& s) { return Str(s); }

/// Str から std::string へ変換
/// nanobind では自動的に行われるが、明示的な変換も可能
inline std::string to_std_string(const Str& s) { return s.to_std_string(); }

} // namespace cutil
