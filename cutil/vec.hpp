#pragma once

#include <cassert>
#include <cmath>
#include <ostream>
#include <string>
#include <type_traits>

// NVec<N, T>: N-dimensional vector with GLSL/GLM/Godot-style API
// Namespace: Cutil

namespace Cutil {

template <unsigned int N, typename T> struct NVec {
  static_assert(N > 0, "NVec: dimension N must be > 0");
  static_assert(std::is_arithmetic<T>::value, "NVec: T must be an arithmetic type");

  T data[N];

  // ---- Constructors --------------------------------------------------------

  constexpr NVec() {
    for(unsigned int i = 0; i < N; i++) data[i] = T(0);
  }

  explicit constexpr NVec(T scalar) {
    for(unsigned int i = 0; i < N; i++) data[i] = scalar;
  }

  // 2D
  constexpr NVec(T x_, T y_) {
    static_assert(N == 2, "NVec(x, y) requires N == 2");
    data[0] = x_;
    data[1] = y_;
  }

  // 3D
  constexpr NVec(T x_, T y_, T z_) {
    static_assert(N == 3, "NVec(x, y, z) requires N == 3");
    data[0] = x_;
    data[1] = y_;
    data[2] = z_;
  }

  // 4D
  constexpr NVec(T x_, T y_, T z_, T w_) {
    static_assert(N == 4, "NVec(x, y, z, w) requires N == 4");
    data[0] = x_;
    data[1] = y_;
    data[2] = z_;
    data[3] = w_;
  }

  // Extend a lower-dimensional vector: NVec<3>(vec2, z)
  constexpr NVec(const NVec<N - 1, T>& v, T last) {
    static_assert(N >= 2, "extension constructor requires N >= 2");
    for(unsigned int i = 0; i < N - 1; i++) data[i] = v.data[i];
    data[N - 1] = last;
  }

  // Convert from a different element type
  template <typename U> explicit constexpr NVec(const NVec<N, U>& o) {
    for(unsigned int i = 0; i < N; i++) data[i] = static_cast<T>(o.data[i]);
  }

  // ---- Named accessors (static_assert ensures only valid at proper N) -------

  constexpr T& x() {
    static_assert(N >= 1, "x() requires N >= 1");
    return data[0];
  }
  constexpr const T& x() const {
    static_assert(N >= 1, "x() requires N >= 1");
    return data[0];
  }

  constexpr T& y() {
    static_assert(N >= 2, "y() requires N >= 2");
    return data[1];
  }
  constexpr const T& y() const {
    static_assert(N >= 2, "y() requires N >= 2");
    return data[1];
  }

  constexpr T& z() {
    static_assert(N >= 3, "z() requires N >= 3");
    return data[2];
  }
  constexpr const T& z() const {
    static_assert(N >= 3, "z() requires N >= 3");
    return data[2];
  }

  constexpr T& w() {
    static_assert(N >= 4, "w() requires N >= 4");
    return data[3];
  }
  constexpr const T& w() const {
    static_assert(N >= 4, "w() requires N >= 4");
    return data[3];
  }

  // Swizzle helpers: xy(), xyz()
  constexpr NVec<2, T> xy() const {
    static_assert(N >= 2);
    return {data[0], data[1]};
  }
  constexpr NVec<3, T> xyz() const {
    static_assert(N >= 3);
    return {data[0], data[1], data[2]};
  }

  // ---- Indexed access -------------------------------------------------------

  constexpr T& operator[](unsigned int i) {
    assert(i < N);
    return data[i];
  }
  constexpr const T& operator[](unsigned int i) const {
    assert(i < N);
    return data[i];
  }

  // ---- Arithmetic operators -------------------------------------------------

  constexpr NVec operator+(const NVec& o) const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = data[i] + o.data[i];
    return r;
  }
  constexpr NVec operator-(const NVec& o) const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = data[i] - o.data[i];
    return r;
  }
  // Component-wise multiply (Hadamard product)
  constexpr NVec operator*(const NVec& o) const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = data[i] * o.data[i];
    return r;
  }
  constexpr NVec operator/(const NVec& o) const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = data[i] / o.data[i];
    return r;
  }

  constexpr NVec operator*(T s) const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = data[i] * s;
    return r;
  }
  constexpr NVec operator/(T s) const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = data[i] / s;
    return r;
  }
  constexpr NVec operator+(T s) const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = data[i] + s;
    return r;
  }
  constexpr NVec operator-(T s) const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = data[i] - s;
    return r;
  }
  constexpr NVec operator-() const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = -data[i];
    return r;
  }

  constexpr NVec& operator+=(const NVec& o) {
    for(unsigned int i = 0; i < N; i++) data[i] += o.data[i];
    return *this;
  }
  constexpr NVec& operator-=(const NVec& o) {
    for(unsigned int i = 0; i < N; i++) data[i] -= o.data[i];
    return *this;
  }
  constexpr NVec& operator*=(const NVec& o) {
    for(unsigned int i = 0; i < N; i++) data[i] *= o.data[i];
    return *this;
  }
  constexpr NVec& operator*=(T s) {
    for(unsigned int i = 0; i < N; i++) data[i] *= s;
    return *this;
  }
  constexpr NVec& operator/=(T s) {
    for(unsigned int i = 0; i < N; i++) data[i] /= s;
    return *this;
  }

  constexpr bool operator==(const NVec& o) const {
    for(unsigned int i = 0; i < N; i++)
      if(data[i] != o.data[i]) return false;
    return true;
  }
  constexpr bool operator!=(const NVec& o) const { return !(*this == o); }

  // ---- Math functions ------------------------------------------------------

  constexpr T dot(const NVec& o) const {
    T s = T(0);
    for(unsigned int i = 0; i < N; i++) s += data[i] * o.data[i];
    return s;
  }

  constexpr T length_squared() const { return dot(*this); }

  double length() const { return std::sqrt(static_cast<double>(length_squared())); }

  // Returns normalized copy (asserts non-zero length)
  NVec normalized() const {
    double len = length();
    assert(len > 0.0 && "normalized() called on zero-length vector");
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = static_cast<T>(data[i] / len);
    return r;
  }

  // Safe normalize: returns zero vector if length < eps
  NVec safe_normalized(double eps = 1e-10) const {
    double len = length();
    if(len < eps) return NVec();
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = static_cast<T>(data[i] / len);
    return r;
  }

  // Cross product (N == 3 only)
  constexpr NVec cross(const NVec& o) const {
    static_assert(N == 3, "cross product requires N == 3");
    return {data[1] * o.data[2] - data[2] * o.data[1], data[2] * o.data[0] - data[0] * o.data[2], data[0] * o.data[1] - data[1] * o.data[0]};
  }

  // Angle between two vectors in radians [0, pi] (Godot: angle_to)
  double angle_to(const NVec& o) const {
    double l = length() * o.length();
    assert(l > 0.0 && "angle_to() called with zero-length vector");
    double cos_a = static_cast<double>(dot(o)) / l;
    return std::acos(std::max(-1.0, std::min(1.0, cos_a)));
  }

  double distance_to(const NVec& o) const { return (o - *this).length(); }
  constexpr T distance_squared_to(const NVec& o) const { return (o - *this).length_squared(); }

  // Linear interpolation (Godot: lerp / GLSL: mix)
  NVec lerp(const NVec& o, double t) const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = static_cast<T>(data[i] * (1.0 - t) + o.data[i] * t);
    return r;
  }

  // Spherical linear interpolation (unit vectors) (Godot: slerp)
  NVec slerp(const NVec& o, double t) const {
    static_assert(N == 3 || N == 4, "slerp requires N == 3 or N == 4");
    double dot_v = std::max(-1.0, std::min(1.0, static_cast<double>(dot(o))));
    double theta = std::acos(dot_v) * t;
    NVec rel     = (o - (*this) * static_cast<T>(dot_v)).safe_normalized();
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = static_cast<T>(data[i] * cos_t + rel.data[i] * sin_t);
    return r;
  }

  // Reflect incident vector around normal (Godot: reflect)
  // result = v - 2 * dot(v, n) * n
  NVec reflect(const NVec& normal) const { return *this - normal * (T(2) * dot(normal)); }

  // Remove component along normal (Godot: slide / GLSL: tangent component)
  NVec slide(const NVec& normal) const { return *this - normal * dot(normal); }

  // Project onto b (Godot: project)
  NVec project(const NVec& b) const {
    T b_sq = b.length_squared();
    assert(b_sq > T(0) && "project() called with zero-length vector");
    return b * (dot(b) / b_sq);
  }

  // Per-element operations
  NVec abs() const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = data[i] < T(0) ? -data[i] : data[i];
    return r;
  }

  NVec clamp(T lo, T hi) const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = data[i] < lo ? lo : (data[i] > hi ? hi : data[i]);
    return r;
  }

  NVec min_with(const NVec& o) const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = data[i] < o.data[i] ? data[i] : o.data[i];
    return r;
  }

  NVec max_with(const NVec& o) const {
    NVec r;
    for(unsigned int i = 0; i < N; i++) r.data[i] = data[i] > o.data[i] ? data[i] : o.data[i];
    return r;
  }

  constexpr T sum() const {
    T s = T(0);
    for(unsigned int i = 0; i < N; i++) s += data[i];
    return s;
  }

  T max_component() const {
    T m = data[0];
    for(unsigned int i = 1; i < N; i++) m = data[i] > m ? data[i] : m;
    return m;
  }

  T min_component() const {
    T m = data[0];
    for(unsigned int i = 1; i < N; i++) m = data[i] < m ? data[i] : m;
    return m;
  }

  bool is_zero_approx(double eps = 1e-6) const {
    for(unsigned int i = 0; i < N; i++)
      if(static_cast<double>(data[i] < T(0) ? -data[i] : data[i]) > eps) return false;
    return true;
  }

  bool is_normalized(double eps = 1e-5) const { return std::abs(length() - 1.0) < eps; }

  // ---- Static factories ----------------------------------------------------

  static constexpr NVec zero() { return NVec(); }

  static constexpr NVec one() { return NVec(T(1)); }

  // Unit vector along axis i (e.g. axis(0) = X axis)
  static constexpr NVec axis(unsigned int i) {
    assert(i < N);
    NVec r;
    r.data[i] = T(1);
    return r;
  }

  // ---- Iterators / size ----------------------------------------------------

  constexpr static unsigned int size() { return N; }
  constexpr T* begin() { return data; }
  constexpr T* end() { return data + N; }
  constexpr const T* begin() const { return data; }
  constexpr const T* end() const { return data + N; }

  // ---- String / ostream ----------------------------------------------------

  std::string str() const {
    std::string s = "NVec<" + std::to_string(N) + ">(";
    for(unsigned int i = 0; i < N; i++) {
      s += std::to_string(data[i]);
      if(i < N - 1) s += ", ";
    }
    return s + ")";
  }
};

// ---- Non-member operators --------------------------------------------------

template <unsigned int N, typename T> constexpr NVec<N, T> operator*(T s, const NVec<N, T>& v) { return v * s; }

template <unsigned int N, typename T> constexpr NVec<N, T> operator+(T s, const NVec<N, T>& v) { return v + s; }

template <unsigned int N, typename T> std::ostream& operator<<(std::ostream& os, const NVec<N, T>& v) {
  os << "NVec<" << N << ">(";
  for(unsigned int i = 0; i < N; i++) {
    os << v.data[i];
    if(i < N - 1) os << ", ";
  }
  return os << ")";
}

// ---- GLSL-style free functions --------------------------------------------

template <unsigned int N, typename T> constexpr T dot(const NVec<N, T>& a, const NVec<N, T>& b) { return a.dot(b); }

template <unsigned int N, typename T> double length(const NVec<N, T>& v) { return v.length(); }

template <unsigned int N, typename T> NVec<N, T> normalize(const NVec<N, T>& v) { return v.normalized(); }

template <typename T> constexpr NVec<3, T> cross(const NVec<3, T>& a, const NVec<3, T>& b) { return a.cross(b); }

// GLSL: mix (same as lerp)
template <unsigned int N, typename T> NVec<N, T> mix(const NVec<N, T>& a, const NVec<N, T>& b, double t) { return a.lerp(b, t); }

template <unsigned int N, typename T> NVec<N, T> lerp(const NVec<N, T>& a, const NVec<N, T>& b, double t) { return a.lerp(b, t); }

template <unsigned int N, typename T> NVec<N, T> reflect(const NVec<N, T>& incident, const NVec<N, T>& normal) { return incident.reflect(normal); }

template <unsigned int N, typename T> NVec<N, T> abs(const NVec<N, T>& v) { return v.abs(); }

template <unsigned int N, typename T> NVec<N, T> clamp(const NVec<N, T>& v, T lo, T hi) { return v.clamp(lo, hi); }

template <unsigned int N, typename T> NVec<N, T> min(const NVec<N, T>& a, const NVec<N, T>& b) { return a.min_with(b); }

template <unsigned int N, typename T> NVec<N, T> max(const NVec<N, T>& a, const NVec<N, T>& b) { return a.max_with(b); }

template <unsigned int N, typename T> double distance(const NVec<N, T>& a, const NVec<N, T>& b) { return a.distance_to(b); }

// ---- Common type aliases (matching GLSL / GLM conventions) ----------------

using Vec2f = NVec<2, float>;
using Vec3f = NVec<3, float>;
using Vec4f = NVec<4, float>;

using Vec2d = NVec<2, double>;
using Vec3d = NVec<3, double>;
using Vec4d = NVec<4, double>;

using Vec2i = NVec<2, int>;
using Vec3i = NVec<3, int>;
using Vec4i = NVec<4, int>;

using Vec2u = NVec<2, unsigned int>;
using Vec3u = NVec<3, unsigned int>;
using Vec4u = NVec<4, unsigned int>;

} // namespace Cutil
