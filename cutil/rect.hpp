#pragma once
#include <cutil/vector.hpp>
#include <limits>
#include <string>

namespace Cutil {
struct Range {
  float min, max;
  Range() { clear(); }
  Range(float min_, float max_) {
    min = min_;
    max = max_;
  }
  [[nodiscard]] float length() const { return max - min; }
  [[nodiscard]] float center() const { return (max + min) / 2.0f; }
  void clear() {
    min = std::numeric_limits<float>::max();
    max = std::numeric_limits<float>::lowest();
  }
  [[nodiscard]] bool contains(float t) const { return min <= t && t <= max; }
  [[nodiscard]] bool intersects(const Range& t) const { return !(t.max <= min || t.min >= max); }
  [[nodiscard]] bool contains(const Range& t) const { return min <= t.min && t.max <= max; }
  void merge(const Range& t) {
    min = std::min(min, t.min);
    max = std::max(max, t.max);
  }
  void expand(const float& t) {
    min = std::min(min, t);
    max = std::max(max, t);
  }
  [[nodiscard]] auto scale(const float t) const { return Range(center() - length() * t / 2.0f, center() + length() * t / 2.0f); }
  [[nodiscard]] Range margin(const float t) const { return {min - t, max + t}; }
  [[nodiscard]] Range shift(const float t) const { return {min + t, max + t}; }
  [[nodiscard]] bool valid() const { return max >= min; }
  bool operator==(const Range& o) const { return o.min == min && o.max == max; }
  static Range Inf() { return {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max()}; }
  Range operator&(const Range& o) const {
    Range r;
    r.min = std::max(min, o.min);
    r.max = std::min(max, o.max);
    return r;
  }
  Range operator|(const Range& o) const {
    Range r;
    r.min = std::min(min, o.min);
    r.max = std::max(max, o.max);
    return r;
  }
  [[nodiscard]] std::string str() const { return "Range(" + std::to_string(min) + ", " + std::to_string(max) + ")"; }
};

struct Rect {
  Range x, y;
  Rect() { x.min = y.min = x.max = y.max = 0; }
  Rect(const Range& xr, const Range& yr) {
    x = xr;
    y = yr;
  }
  Rect(float xmin, float xmax, float ymin, float ymax) {
    x.min = xmin;
    x.max = xmax;
    y.min = ymin;
    y.max = ymax;
  }
  template <typename T> Rect(const _Vec<T, 2>& pos, const _Vec<T, 2>& size) {
    x.min = pos[0];
    x.max = pos[0] + size[0];
    y.min = pos[1];
    y.max = pos[1] + size[1];
  }
  [[nodiscard]] Vector2f center() const { return {x.center(), y.center()}; }
  [[nodiscard]] Vector2f size() const { return {x.length(), y.length()}; }
  [[nodiscard]] Vector2f pos() const { return {x.min, y.min}; }
  [[nodiscard]] float top() const { return y.min; }
  [[nodiscard]] float bottom() const { return y.max; }
  [[nodiscard]] float left() const { return x.min; }
  [[nodiscard]] float right() const { return x.max; }
  float& top() { return y.min; }
  float& bottom() { return y.max; }
  float& left() { return x.min; }
  float& right() { return x.max; }

  [[nodiscard]] auto w() const { return x.length(); } /// width  ( x length  )
  [[nodiscard]] auto h() const { return y.length(); } /// height ( y lngth   )
  [[nodiscard]] bool contains(const Rect& other) const { return x.contains(other.x) && y.contains(other.y); }
  [[nodiscard]] bool intersects(const Rect& other) const { return x.contains(other.x) && y.contains(other.y); }
  template <typename T> bool contains(const _Vec<T, 2>& p) const { return contains(p[0], p[1]); }
  template <typename T> bool contains(const T x_, const T y_) const { return x.contains(x_) && y.contains(y_); }
  [[nodiscard]] float area() const { return x.length() * y.length(); }
  [[nodiscard]] Rect scale(float t) const { return {x.scale(t), y.scale(t)}; }
  template <typename T> Rect scale(const _Vec<T, 2> t) { return {x.scale(t[0]), y.scale(t[1])}; }
  [[nodiscard]] Rect margin(float t) const { return {x.margin(t), y.margin(t)}; }
  [[nodiscard]] Rect shift(const Vector2f dx) const { return {x.shift(dx[0]), y.shift(dx[1])}; }
  template <typename T> void expand(const T x_, const T y_) { expand({x_, y_}); }
  template <typename T> void expand(const _Vec<T, 2>& p) {
    x.expand(p[0]);
    y.expand(p[1]);
  }
  [[nodiscard]] bool valid() const { return x.valid() && y.valid(); }
  void merge(const Rect& other) {
    x.merge(other.x);
    y.merge(other.y);
  }
  void clear() {
    x.clear();
    y.clear();
  }
  [[nodiscard]] std::string str() const { return "Rect3D( [x,y] = " + x.str() + ", " + y.str() + ", " + ")"; }
  static auto Inf() { return Rect(Range::Inf(), Range::Inf()); }
  [[nodiscard]] auto operator|(const Rect& o) const { return Rect(x | o.x, y | o.y); }
  [[nodiscard]] auto operator&(const Rect& o) const { return Rect(x & o.x, y & o.y); }
  bool operator==(const Rect& o) const { return x == o.x && y == o.y; }
  static Rect PosSize(const Vector2f& p, const Vector2f& s) { return {p, s}; }
  static Rect PosSize(const Vector2f& p, const float& s) { return {p, Vector2f(s, s)}; }
  static Rect PosSize(float x, float y, float width, float height) { return {Vector2f(x, y), Vector2f(width, height)}; }
  static Rect CenterSize(const Vector2f& c, const Vector2f& s) { return {c - s / 2, c + s / 2}; }
};
inline void operator|=(Rect& r1, const Rect& r2) {
  r1 = r1 | r2;
}
inline void operator&=(Rect& r1, const Rect& r2) {
  r1 = r1 & r2;
}
} // namespace Cutil
