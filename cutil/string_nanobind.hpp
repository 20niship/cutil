#pragma once

#include "string.hpp"

// nanobind での Str の使用パターン
//
// nanobind では std::string がネイティブにサポートされているので、
// 以下のパターンで Str と Python str を自動変換できます：
//
// 1. 入力変換パターン：
//    nb::class_<MyClass>(m, "MyClass")
//      .def("process", [](const cutil::Str& s) {
//        // ... s を使用
//      });
//    これは以下のように呼び出し：
//    obj.process("hello")  # Python str が自動的に cutil::Str に変換される
//
// 2. 出力変換パターン：
//    cutil::Str を返す関数は、自動的に Python str に変換されます
//
// 3. メンババインディング：
//    nb::class_<StringHolder>(m, "StringHolder")
//      .def_rw("my_str", &StringHolder::my_str);
//    これも自動変換されます
//
// 詳細な仕組みは以下に実装：

namespace cutil {

// Str <-> std::string 相互変換（nanobind が活用）

/// std::string から Str への暗黙変換コンストラクタ（既に実装済み）
/// Str(const std::string& s) { ... }

/// Str から std::string への暗黙変換（std::string_view経由）
/// operator std::string_view() const { ... }

// nanobind統合時の補助関数

/// 任意の Python オブジェクトから Str へ変換
/// nanobind の lambda 内で str(obj) を使用する場合
inline Str from_python_str(const std::string& s) {
  return Str(s);
}

/// Str から std::string へ変換
/// nanobind では自動的に行われるが、明示的な変換も可能
inline std::string to_std_string(const Str& s) {
  return s.to_std_string();
}

}  // namespace cutil

// ============================================================================
// How nanobind integrates Str automatically
// ============================================================================
//
// When you include <nanobind/stl/string.h>, nanobind's type system can:
//
// 1. Python str -> std::string: PyUnicode_AsUTF8 + std::string constructor
// 2. std::string -> Python str: PyUnicode_FromStringAndSize
// 3. Since cutil::Str(const std::string&) and cutil::Str::to_std_string() exist,
//    the chain works automatically:
//
//    Python str <==> std::string <==> cutil::Str
//
// Example binding (in mylib.cpp):
//
//   m.def("process_str", [](const cutil::Str& s) {
//     return s.to_upper();  // returns cutil::Str
//   });
//
//   # In Python:
//   result = mylib.process_str("hello")  # "hello" is auto-converted to Str
//   print(result)  # result is automatically converted back to Python str
//
