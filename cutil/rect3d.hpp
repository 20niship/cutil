#pragma once
#include <cutil/rect.hpp>
#include <cutil/vec.hpp>

namespace cutil {

// Rect3D: cutil::Rect(2D)の3D版。3軸(x,y,z)のRangeを持つAABB(軸並行境界ボックス)。
struct Rect3D {
  Range x, y, z;
  Rect3D() { x.clear(); y.clear(); z.clear(); }
  Rect3D(const Range& xr, const Range& yr, const Range& zr) : x(xr), y(yr), z(zr) {}
  Rect3D(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax) {
    x.min = xmin; x.max = xmax;
    y.min = ymin; y.max = ymax;
    z.min = zmin; z.max = zmax;
  }
  Rect3D(const Vec3f& pos, const Vec3f& size) {
    x.min = pos.data[0]; x.max = pos.data[0] + size.data[0];
    y.min = pos.data[1]; y.max = pos.data[1] + size.data[1];
    z.min = pos.data[2]; z.max = pos.data[2] + size.data[2];
  }

  [[nodiscard]] Vec3f center() const { return Vec3f(x.center(), y.center(), z.center()); }
  [[nodiscard]] Vec3f size() const { return Vec3f(x.length(), y.length(), z.length()); }
  [[nodiscard]] Vec3f pos() const { return Vec3f(x.min, y.min, z.min); }

  [[nodiscard]] bool contains(const Rect3D& other) const { return x.contains(other.x) && y.contains(other.y) && z.contains(other.z); }
  [[nodiscard]] bool intersects(const Rect3D& other) const { return x.intersects(other.x) && y.intersects(other.y) && z.intersects(other.z); }
  [[nodiscard]] bool contains(const Vec3f& p) const { return x.contains(p.data[0]) && y.contains(p.data[1]) && z.contains(p.data[2]); }

  [[nodiscard]] float volume() const { return x.length() * y.length() * z.length(); }
  [[nodiscard]] Rect3D scale(float t) const { return {x.scale(t), y.scale(t), z.scale(t)}; }
  [[nodiscard]] Rect3D margin(float t) const { return {x.margin(t), y.margin(t), z.margin(t)}; }
  [[nodiscard]] Rect3D shift(const Vec3f& d) const { return {x.shift(d.data[0]), y.shift(d.data[1]), z.shift(d.data[2])}; }

  void expand(const Vec3f& p) {
    x.expand(p.data[0]);
    y.expand(p.data[1]);
    z.expand(p.data[2]);
  }
  [[nodiscard]] bool valid() const { return x.valid() && y.valid() && z.valid(); }
  void merge(const Rect3D& other) {
    x.merge(other.x);
    y.merge(other.y);
    z.merge(other.z);
  }
  void clear() {
    x.clear();
    y.clear();
    z.clear();
  }

  [[nodiscard]] std::string str() const { return "Rect3D( [x,y,z] = " + x.str() + ", " + y.str() + ", " + z.str() + ")"; }
  static auto Inf() { return Rect3D(Range::Inf(), Range::Inf(), Range::Inf()); }

  [[nodiscard]] Rect3D operator|(const Rect3D& o) const { return {x | o.x, y | o.y, z | o.z}; }
  [[nodiscard]] Rect3D operator&(const Rect3D& o) const { return {x & o.x, y & o.y, z & o.z}; }
  bool operator==(const Rect3D& o) const { return x == o.x && y == o.y && z == o.z; }

  static Rect3D PosSize(const Vec3f& p, const Vec3f& s) { return {p, s}; }
  static Rect3D CenterSize(const Vec3f& c, const Vec3f& s) {
    Vec3f half = s / 2.0f;
    return {Range(c.data[0] - half.data[0], c.data[0] + half.data[0]), Range(c.data[1] - half.data[1], c.data[1] + half.data[1]),
            Range(c.data[2] - half.data[2], c.data[2] + half.data[2])};
  }
};
inline void operator|=(Rect3D& r1, const Rect3D& r2) { r1 = r1 | r2; }
inline void operator&=(Rect3D& r1, const Rect3D& r2) { r1 = r1 & r2; }

} // namespace cutil
