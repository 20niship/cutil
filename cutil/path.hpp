#pragma once

#include <cutil/string.hpp>

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
  const char* c_str() const { return path_.c_str(); }

  Path operator/(const Str& child) const;
  bool operator==(const Path& other) const { return path_ == other.path_; }
  bool operator!=(const Path& other) const { return !(*this == other); }
  Str operator+(const Str& other) const { return path_ + other; }

private:
  Str path_;
};

} // namespace cutil

#ifdef CUTIL_IMPLEMENTATION
#include <filesystem>

namespace cutil {

bool Path::is_absolute() const { return std::filesystem::path(path_.c_str()).is_absolute(); }
Path Path::absolute() const { return Path(Str(std::filesystem::absolute(std::filesystem::path(path_.c_str())).string().c_str())); }
Path Path::relative_to(const Path& base) const { return Path(Str(std::filesystem::relative(std::filesystem::path(path_.c_str()), std::filesystem::path(base.path_.c_str())).string().c_str())); }
Path Path::parent() const { return Path(Str(std::filesystem::path(path_.c_str()).parent_path().string().c_str())); }
Str Path::filename() const { return Str(std::filesystem::path(path_.c_str()).filename().string().c_str()); }
Str Path::extension() const { return Str(std::filesystem::path(path_.c_str()).extension().string().c_str()); }
Path Path::operator/(const Str& child) const { return Path(Str((std::filesystem::path(path_.c_str()) / child.c_str()).string().c_str())); }

} // namespace cutil
#endif
