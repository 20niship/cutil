#pragma once

#include <cutil/string.hpp>

// cutil::Path: std::filesystem::path への薄いラッパー
// STBスタイル: 通常include時は<filesystem>をincludeせず宣言のみ公開する。
// CUTIL_IMPLEMENTATION を定義した唯一のTUでのみ<filesystem>をincludeし実装する。

namespace cutil {

class Path {
public:
  Path() = default;
  explicit Path(const Str& s) : path_(s) {}
  explicit Path(const char* s) : path_(s) {}

  const Str& str() const { return path_; }
  bool empty() const { return path_.empty(); }

  bool is_absolute() const;
  Path absolute() const;
  Path relative_to(const Path& base) const;
  Path parent() const;
  Str filename() const;
  Str extension() const;
  Path operator/(const Str& child) const;

  bool operator==(const Path& other) const { return path_ == other.path_; }
  bool operator!=(const Path& other) const { return !(*this == other); }

private:
  Str path_; // std::filesystem::pathはメンバに持たない(宣言側を<filesystem>非依存にするため)
};

} // namespace cutil

#ifdef CUTIL_IMPLEMENTATION
#include <filesystem>

// 注意: ここでの定義は意図的に`inline`を付けない。
// `inline`(vague linkage)な関数は、そのTU内で実際に呼ばれない場合コンパイラが
// 定義自体を出力しないことがあるため(未使用のinline関数は他TUで使われる前提でも
// 省略されうる)、非inlineの通常のexternal linkageな定義にしてこの1TUで確実に
// シンボルが生成されるようにする。

namespace cutil {

bool Path::is_absolute() const { return std::filesystem::path(path_.c_str()).is_absolute(); }

Path Path::absolute() const { return Path(Str(std::filesystem::absolute(std::filesystem::path(path_.c_str())).string().c_str())); }

Path Path::relative_to(const Path& base) const {
  return Path(Str(std::filesystem::relative(std::filesystem::path(path_.c_str()), std::filesystem::path(base.path_.c_str())).string().c_str()));
}

Path Path::parent() const { return Path(Str(std::filesystem::path(path_.c_str()).parent_path().string().c_str())); }

Str Path::filename() const { return Str(std::filesystem::path(path_.c_str()).filename().string().c_str()); }

Str Path::extension() const { return Str(std::filesystem::path(path_.c_str()).extension().string().c_str()); }

Path Path::operator/(const Str& child) const { return Path(Str((std::filesystem::path(path_.c_str()) / child.c_str()).string().c_str())); }

} // namespace cutil
#endif
