#pragma once

#include <cassert>
#include <cmath>
#include <cutil/mat.hpp>
#include <cutil/vec.hpp>
#include <ostream>
#include <string>
#include <type_traits>

// Quat<T>: Quaternion for 3D rotations, GLM/Godot-style API
// Storage order: x, y, z, w (w last, matching Godot::Quaternion)
// Namespace: Cutil

namespace cutil {

template <typename T = float> struct Quat {
  static_assert(std::is_floating_point<T>::value, "Quat: T must be a floating-point type");

  T x, y, z, w;

  // ---- Constructors ----------------------------------------------------

  constexpr Quat() : x(T(0)), y(T(0)), z(T(0)), w(T(1)) {}
  constexpr Quat(T x_, T y_, T z_, T w_) : x(x_), y(y_), z(z_), w(w_) {}

  // Axis-angle (angle in radians). axis need not be normalized.
  Quat(const NVec<3, T>& axis, double angle) {
    NVec<3, T> n = axis.normalized();
    double half  = angle * 0.5;
    T s          = static_cast<T>(std::sin(half));
    x            = n[0] * s;
    y            = n[1] * s;
    z            = n[2] * s;
    w            = static_cast<T>(std::cos(half));
  }

  // From a 3x3 rotation matrix
  explicit Quat(const Mat<3, 3, T>& basis) { *this = from_mat3(basis); }

  // ---- Static factories --------------------------------------------------

  static constexpr Quat identity() { return Quat(); }

  static Quat from_axis_angle(const NVec<3, T>& axis, double angle) { return Quat(axis, angle); }

  // euler = (roll, pitch, yaw), applied in ZYX intrinsic order (aerospace convention).
  // The inverse of get_euler().
  static Quat from_euler(const NVec<3, T>& euler) {
    const double hr = static_cast<double>(euler[0]) * 0.5;
    const double hp = static_cast<double>(euler[1]) * 0.5;
    const double hy = static_cast<double>(euler[2]) * 0.5;
    const double cr = std::cos(hr), sr = std::sin(hr);
    const double cp = std::cos(hp), sp = std::sin(hp);
    const double cy = std::cos(hy), sy = std::sin(hy);
    return Quat(static_cast<T>(sr * cp * cy - cr * sp * sy), static_cast<T>(cr * sp * cy + sr * cp * sy), static_cast<T>(cr * cp * sy - sr * sp * cy),
                static_cast<T>(cr * cp * cy + sr * sp * sy));
  }

  static Quat from_mat3(const Mat<3, 3, T>& m) {
    const T tr = m(0, 0) + m(1, 1) + m(2, 2);
    T qx, qy, qz, qw;
    if(tr > T(0)) {
      T s = T(0.5) / static_cast<T>(std::sqrt(static_cast<double>(tr) + 1.0));
      qw  = T(0.25) / s;
      qx  = (m(2, 1) - m(1, 2)) * s;
      qy  = (m(0, 2) - m(2, 0)) * s;
      qz  = (m(1, 0) - m(0, 1)) * s;
    } else if(m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
      T s = T(2) * static_cast<T>(std::sqrt(1.0 + static_cast<double>(m(0, 0) - m(1, 1) - m(2, 2))));
      qw  = (m(2, 1) - m(1, 2)) / s;
      qx  = T(0.25) * s;
      qy  = (m(0, 1) + m(1, 0)) / s;
      qz  = (m(0, 2) + m(2, 0)) / s;
    } else if(m(1, 1) > m(2, 2)) {
      T s = T(2) * static_cast<T>(std::sqrt(1.0 + static_cast<double>(m(1, 1) - m(0, 0) - m(2, 2))));
      qw  = (m(0, 2) - m(2, 0)) / s;
      qx  = (m(0, 1) + m(1, 0)) / s;
      qy  = T(0.25) * s;
      qz  = (m(1, 2) + m(2, 1)) / s;
    } else {
      T s = T(2) * static_cast<T>(std::sqrt(1.0 + static_cast<double>(m(2, 2) - m(0, 0) - m(1, 1))));
      qw  = (m(1, 0) - m(0, 1)) / s;
      qx  = (m(0, 2) + m(2, 0)) / s;
      qy  = (m(1, 2) + m(2, 1)) / s;
      qz  = T(0.25) * s;
    }
    return Quat(qx, qy, qz, qw);
  }

  // ---- Arithmetic operators ----------------------------------------------

  constexpr Quat operator+(const Quat& o) const { return Quat(x + o.x, y + o.y, z + o.z, w + o.w); }
  constexpr Quat operator-(const Quat& o) const { return Quat(x - o.x, y - o.y, z - o.z, w - o.w); }
  constexpr Quat operator-() const { return Quat(-x, -y, -z, -w); }
  constexpr Quat operator*(T s) const { return Quat(x * s, y * s, z * s, w * s); }
  constexpr Quat operator/(T s) const { return Quat(x / s, y / s, z / s, w / s); }

  // Hamilton product: composes rotations. (a * b) applied to a vector rotates by b first, then a.
  constexpr Quat operator*(const Quat& o) const {
    return Quat(w * o.x + x * o.w + y * o.z - z * o.y, w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w,
                w * o.w - x * o.x - y * o.y - z * o.z);
  }

  // Rotate a vector by this quaternion (assumed to be unit-length)
  NVec<3, T> operator*(const NVec<3, T>& v) const {
    const NVec<3, T> qv(x, y, z);
    const NVec<3, T> t = qv.cross(v) * T(2);
    return v + t * w + qv.cross(t);
  }

  constexpr Quat& operator+=(const Quat& o) {
    x += o.x;
    y += o.y;
    z += o.z;
    w += o.w;
    return *this;
  }
  constexpr Quat& operator-=(const Quat& o) {
    x -= o.x;
    y -= o.y;
    z -= o.z;
    w -= o.w;
    return *this;
  }
  constexpr Quat& operator*=(T s) {
    x *= s;
    y *= s;
    z *= s;
    w *= s;
    return *this;
  }
  Quat& operator*=(const Quat& o) {
    *this = *this * o;
    return *this;
  }

  constexpr bool operator==(const Quat& o) const { return x == o.x && y == o.y && z == o.z && w == o.w; }
  constexpr bool operator!=(const Quat& o) const { return !(*this == o); }

  // ---- Math functions ------------------------------------------------------

  constexpr T dot(const Quat& o) const { return x * o.x + y * o.y + z * o.z + w * o.w; }
  constexpr T length_squared() const { return dot(*this); }
  double length() const { return std::sqrt(static_cast<double>(length_squared())); }

  Quat normalized() const {
    double len = length();
    assert(len > 0.0 && "normalized() called on zero-length quaternion");
    return Quat(static_cast<T>(x / len), static_cast<T>(y / len), static_cast<T>(z / len), static_cast<T>(w / len));
  }

  constexpr Quat conjugate() const { return Quat(-x, -y, -z, w); }

  Quat inverse() const {
    const T n2 = length_squared();
    assert(n2 > T(0) && "inverse() called on zero-length quaternion");
    return conjugate() / n2;
  }

  // Rotation angle in radians, in [0, 2*pi]
  double get_angle() const { return 2.0 * std::acos(std::max(-1.0, std::min(1.0, static_cast<double>(w)))); }

  // Normalized rotation axis. Falls back to the X axis for a near-zero rotation.
  NVec<3, T> get_axis() const {
    const double s = std::sqrt(std::max(0.0, 1.0 - static_cast<double>(w) * static_cast<double>(w)));
    if(s < 1e-8) return NVec<3, T>::axis(0);
    return NVec<3, T>(static_cast<T>(x / s), static_cast<T>(y / s), static_cast<T>(z / s));
  }

  // Returns (roll, pitch, yaw), the inverse of from_euler()
  NVec<3, T> get_euler() const {
    const double sinr_cosp = 2.0 * (static_cast<double>(w) * x + static_cast<double>(y) * z);
    const double cosr_cosp = 1.0 - 2.0 * (static_cast<double>(x) * x + static_cast<double>(y) * y);
    const double roll      = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp   = 2.0 * (static_cast<double>(w) * y - static_cast<double>(z) * x);
    const double pitch  = std::abs(sinp) >= 1.0 ? std::copysign(1.5707963267948966, sinp) : std::asin(sinp);

    const double siny_cosp = 2.0 * (static_cast<double>(w) * z + static_cast<double>(x) * y);
    const double cosy_cosp = 1.0 - 2.0 * (static_cast<double>(y) * y + static_cast<double>(z) * z);
    const double yaw        = std::atan2(siny_cosp, cosy_cosp);

    return NVec<3, T>(static_cast<T>(roll), static_cast<T>(pitch), static_cast<T>(yaw));
  }

  bool is_normalized(double eps = 1e-5) const { return std::abs(length() - 1.0) < eps; }

  Mat<3, 3, T> to_mat3() const {
    Mat<3, 3, T> m;
    m(0, 0) = T(1) - T(2) * (y * y + z * z);
    m(0, 1) = T(2) * (x * y - w * z);
    m(0, 2) = T(2) * (x * z + w * y);
    m(1, 0) = T(2) * (x * y + w * z);
    m(1, 1) = T(1) - T(2) * (x * x + z * z);
    m(1, 2) = T(2) * (y * z - w * x);
    m(2, 0) = T(2) * (x * z - w * y);
    m(2, 1) = T(2) * (y * z + w * x);
    m(2, 2) = T(1) - T(2) * (x * x + y * y);
    return m;
  }

  Mat<4, 4, T> to_mat4() const {
    const Mat<3, 3, T> r = to_mat3();
    Mat<4, 4, T> m       = Mat<4, 4, T>::identity();
    for(unsigned int row = 0; row < 3; row++)
      for(unsigned int col = 0; col < 3; col++) m(row, col) = r(row, col);
    return m;
  }

  // Spherical linear interpolation to `to`, taking the shorter arc
  Quat slerp(const Quat& to, double t) const {
    double cosom       = static_cast<double>(dot(to));
    Quat to_corrected = to;
    if(cosom < 0.0) {
      cosom        = -cosom;
      to_corrected = -to;
    }
    double scale0, scale1;
    if(1.0 - cosom > 1e-6) {
      const double omega = std::acos(cosom);
      const double sinom = std::sin(omega);
      scale0             = std::sin((1.0 - t) * omega) / sinom;
      scale1             = std::sin(t * omega) / sinom;
    } else {
      scale0 = 1.0 - t;
      scale1 = t;
    }
    return Quat(static_cast<T>(scale0 * x + scale1 * to_corrected.x), static_cast<T>(scale0 * y + scale1 * to_corrected.y),
                static_cast<T>(scale0 * z + scale1 * to_corrected.z), static_cast<T>(scale0 * w + scale1 * to_corrected.w));
  }

  std::string str() const {
    return "Quat(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ", " + std::to_string(w) + ")";
  }
};

// ---- Non-member operators ---------------------------------------------------

template <typename T> constexpr Quat<T> operator*(T s, const Quat<T>& q) { return q * s; }

template <typename T> std::ostream& operator<<(std::ostream& os, const Quat<T>& q) {
  os << "Quat(" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << ")";
  return os;
}

// ---- GLSL-style free functions --------------------------------------------

template <typename T> constexpr T dot(const Quat<T>& a, const Quat<T>& b) { return a.dot(b); }

template <typename T> Quat<T> normalize(const Quat<T>& q) { return q.normalized(); }

template <typename T> Quat<T> inverse(const Quat<T>& q) { return q.inverse(); }

template <typename T> Quat<T> slerp(const Quat<T>& a, const Quat<T>& b, double t) { return a.slerp(b, t); }

// Rotation matrix built from a quaternion (mirrors mat4_rotation(axis, angle) in mat.hpp)
template <typename T> Mat<4, 4, T> mat4_rotation(const Quat<T>& q) { return q.to_mat4(); }

// ---- Common type aliases ----------------------------------------------------

using Quatf = Quat<float>;
using Quatd = Quat<double>;

} // namespace cutil
