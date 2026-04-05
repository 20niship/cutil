#pragma once

#include <cassert>
#include <cmath>
#include <cutil/vec.hpp>
#include <ostream>
#include <string>
#include <type_traits>

// Mat<Rows, Cols, T>: Matrix with column-major storage (OpenGL/GLSL convention)
// Element access: (row, col) -> data[col * Rows + row]
// SIMD-optimized for Mat<4, 4, float> with SSE2

namespace Cutil {

// Forward declaration for matrix multiplication helpers
namespace detail {
template <unsigned int Rows, unsigned int K, unsigned int Cols, typename T> void mat_mul_impl(const T* A, const T* B, T* C);
}

template <unsigned int Rows, unsigned int Cols, typename T> struct Mat {
  static_assert(Rows > 0, "Mat: Rows must be > 0");
  static_assert(Cols > 0, "Mat: Cols must be > 0");
  static_assert(std::is_arithmetic<T>::value, "Mat: T must be arithmetic");

  // Column-major storage: data[col * Rows + row]
  // Alignment for SIMD: 16 bytes for SSE compatibility
  alignas(16) T data[Rows * Cols] = {};

  // ---- Constructors --------------------------------------------------------

  constexpr Mat() {
    for(unsigned int i = 0; i < Rows * Cols; i++) data[i] = T(0);
  }

  // From array
  explicit constexpr Mat(const T* ptr) {
    for(unsigned int i = 0; i < Rows * Cols; i++) data[i] = ptr[i];
  }

  // From another matrix of same dimensions
  constexpr Mat(const Mat& o) {
    for(unsigned int i = 0; i < Rows * Cols; i++) data[i] = o.data[i];
  }

  Mat& operator=(const Mat& o) {
    if(this != &o) {
      for(unsigned int i = 0; i < Rows * Cols; i++) data[i] = o.data[i];
    }
    return *this;
  }

  // ---- Element access (column-major) ----------------------------------------

  constexpr T& operator()(unsigned int row, unsigned int col) {
    assert(row < Rows && col < Cols);
    return data[col * Rows + row];
  }

  constexpr const T& operator()(unsigned int row, unsigned int col) const {
    assert(row < Rows && col < Cols);
    return data[col * Rows + row];
  }

  // Flat array access
  constexpr T& operator[](unsigned int i) {
    assert(i < Rows * Cols);
    return data[i];
  }
  constexpr const T& operator[](unsigned int i) const {
    assert(i < Rows * Cols);
    return data[i];
  }

  // ---- Column/row access ---------------------------------------------------

  NVec<Rows, T> col(unsigned int c) const {
    assert(c < Cols);
    NVec<Rows, T> result;
    for(unsigned int i = 0; i < Rows; i++) result[i] = data[c * Rows + i];
    return result;
  }

  void set_col(unsigned int c, const NVec<Rows, T>& v) {
    assert(c < Cols);
    for(unsigned int i = 0; i < Rows; i++) data[c * Rows + i] = v[i];
  }

  NVec<Cols, T> row(unsigned int r) const {
    assert(r < Rows);
    NVec<Cols, T> result;
    for(unsigned int c = 0; c < Cols; c++) result[c] = data[c * Rows + r];
    return result;
  }

  void set_row(unsigned int r, const NVec<Cols, T>& v) {
    assert(r < Rows);
    for(unsigned int c = 0; c < Cols; c++) data[c * Rows + r] = v[c];
  }

  // ---- Arithmetic operators ------------------------------------------------

  Mat operator+(const Mat& o) const {
    Mat r;
    for(unsigned int i = 0; i < Rows * Cols; i++) r.data[i] = data[i] + o.data[i];
    return r;
  }

  Mat operator-(const Mat& o) const {
    Mat r;
    for(unsigned int i = 0; i < Rows * Cols; i++) r.data[i] = data[i] - o.data[i];
    return r;
  }

  Mat operator*(T s) const {
    Mat r;
    for(unsigned int i = 0; i < Rows * Cols; i++) r.data[i] = data[i] * s;
    return r;
  }

  Mat operator/(T s) const {
    Mat r;
    for(unsigned int i = 0; i < Rows * Cols; i++) r.data[i] = data[i] / s;
    return r;
  }

  Mat operator-() const {
    Mat r;
    for(unsigned int i = 0; i < Rows * Cols; i++) r.data[i] = -data[i];
    return r;
  }

  Mat& operator+=(const Mat& o) {
    for(unsigned int i = 0; i < Rows * Cols; i++) data[i] += o.data[i];
    return *this;
  }

  Mat& operator-=(const Mat& o) {
    for(unsigned int i = 0; i < Rows * Cols; i++) data[i] -= o.data[i];
    return *this;
  }

  Mat& operator*=(T s) {
    for(unsigned int i = 0; i < Rows * Cols; i++) data[i] *= s;
    return *this;
  }

  bool operator==(const Mat& o) const {
    for(unsigned int i = 0; i < Rows * Cols; i++)
      if(data[i] != o.data[i]) return false;
    return true;
  }

  bool operator!=(const Mat& o) const { return !(*this == o); }

  // ---- Matrix multiplication: Mat<Rows, K> * Mat<K, Cols> -> Mat<Rows, Cols> -------

  template <unsigned int K> Mat<Rows, K, T> operator*(const Mat<K, Cols, T>& B) const {
    Mat<Rows, K, T> result;
    detail::mat_mul_impl<Rows, Cols, K, T>(data, B.data, result.data);
    return result;
  }

  // ---- Matrix-vector multiplication: Mat<Rows, Cols> * Vec<Cols> -> Vec<Rows> ----

  NVec<Rows, T> operator*(const NVec<Cols, T>& v) const {
    NVec<Rows, T> result;
    for(unsigned int i = 0; i < Rows; i++) {
      T sum = T(0);
      for(unsigned int j = 0; j < Cols; j++) {
        sum += data[j * Rows + i] * v[j];
      }
      result[i] = sum;
    }
    return result;
  }

  // ---- Transpose -----------------------------------------------------------

  Mat<Cols, Rows, T> transposed() const {
    Mat<Cols, Rows, T> result;
    for(unsigned int c = 0; c < Cols; c++) {
      for(unsigned int r = 0; r < Rows; r++) {
        result(c, r) = data[c * Rows + r];
      }
    }
    return result;
  }

  // In-place transpose only for square matrices
  void transpose() {
    static_assert(Rows == Cols, "in-place transpose requires square matrix");
    for(unsigned int c = 0; c < Cols; c++) {
      for(unsigned int r = c + 1; r < Rows; r++) {
        std::swap(data[c * Rows + r], data[r * Rows + c]);
      }
    }
  }

  // ---- Square matrix functions (Rows == Cols) ------

  T trace() const {
    static_assert(Rows == Cols, "trace() requires square matrix");
    T sum = T(0);
    for(unsigned int i = 0; i < Rows; i++) sum += data[i * Rows + i];
    return sum;
  }

  // Determinant via Gaussian elimination
  T det() const {
    static_assert(Rows == Cols, "det() requires square matrix");
    Mat tmp = *this;
    T det   = T(1);
    for(unsigned int i = 0; i < Rows; i++) {
      // Find pivot
      int pivot = i;
      for(unsigned int j = i + 1; j < Rows; j++) {
        if(std::abs(tmp(j, i)) > std::abs(tmp(pivot, i))) pivot = j;
      }
      if(std::abs(tmp(pivot, i)) < T(1e-10)) return T(0); // Singular
      if(pivot != static_cast<int>(i)) {
        // Swap rows
        for(unsigned int k = 0; k < Cols; k++) {
          std::swap(tmp(i, k), tmp(pivot, k));
        }
        det = -det;
      }
      // Eliminate
      det *= tmp(i, i);
      T scale = T(1) / tmp(i, i);
      for(unsigned int k = i; k < Cols; k++) {
        tmp(i, k) *= scale;
      }
      for(unsigned int j = i + 1; j < Rows; j++) {
        T factor = tmp(j, i);
        for(unsigned int k = i; k < Cols; k++) {
          tmp(j, k) -= factor * tmp(i, k);
        }
      }
    }
    return det;
  }

  // Matrix inverse via Gaussian elimination
  Mat inverse() const {
    static_assert(Rows == Cols, "inverse() requires square matrix");
    const unsigned int N = Rows;
    Mat A                = *this;
    Mat I                = Mat::identity();

    for(unsigned int i = 0; i < N; i++) {
      // Find pivot
      int pivot = i;
      for(unsigned int j = i + 1; j < N; j++) {
        if(std::abs(A(j, i)) > std::abs(A(pivot, i))) pivot = j;
      }
      assert(std::abs(A(pivot, i)) > T(1e-10) && "Matrix is singular");
      if(pivot != static_cast<int>(i)) {
        for(unsigned int k = 0; k < N; k++) {
          std::swap(A(i, k), A(pivot, k));
          std::swap(I(i, k), I(pivot, k));
        }
      }
      // Scale pivot row
      T scale = T(1) / A(i, i);
      for(unsigned int k = 0; k < N; k++) {
        A(i, k) *= scale;
        I(i, k) *= scale;
      }
      // Eliminate
      for(unsigned int j = 0; j < N; j++) {
        if(i != j) {
          T factor = A(j, i);
          for(unsigned int k = 0; k < N; k++) {
            A(j, k) -= factor * A(i, k);
            I(j, k) -= factor * I(i, k);
          }
        }
      }
    }
    return I;
  }

  // ---- Norms ---------------------------------------------------------------

  T frobenius_norm_sq() const {
    T sum = T(0);
    for(unsigned int i = 0; i < Rows * Cols; i++) sum += data[i] * data[i];
    return sum;
  }

  double frobenius_norm() const { return std::sqrt(static_cast<double>(frobenius_norm_sq())); }

  // ---- Static factories ----------------------------------------------------

  static Mat identity() {
    static_assert(Rows == Cols, "identity requires square matrix");
    Mat m;
    for(unsigned int i = 0; i < Rows; i++) m(i, i) = T(1);
    return m;
  }

  static Mat zeros() {
    Mat m;
    for(unsigned int i = 0; i < Rows * Cols; i++) m.data[i] = T(0);
    return m;
  }

  static Mat ones() {
    Mat m;
    for(unsigned int i = 0; i < Rows * Cols; i++) m.data[i] = T(1);
    return m;
  }

  // ---- Information ---------------------------------------------------------

  constexpr static unsigned int rows() { return Rows; }
  constexpr static unsigned int cols() { return Cols; }
  constexpr static unsigned int size() { return Rows * Cols; }

  T* begin() { return data; }
  T* end() { return data + Rows * Cols; }
  const T* begin() const { return data; }
  const T* end() const { return data + Rows * Cols; }

  std::string str() const {
    std::string s = "Mat<" + std::to_string(Rows) + ", " + std::to_string(Cols) + ">(";
    for(unsigned int r = 0; r < Rows; r++) {
      s += "[";
      for(unsigned int c = 0; c < Cols; c++) {
        s += std::to_string(operator()(r, c));
        if(c < Cols - 1) s += ", ";
      }
      s += "]";
      if(r < Rows - 1) s += "; ";
    }
    s += ")";
    return s;
  }
};

// ---- Matrix multiplication helper implementation ---------------------------

namespace detail {

// General implementation (used for most cases)
template <unsigned int Rows, unsigned int K, unsigned int Cols, typename T> inline void mat_mul_impl(const T* A, const T* B, T* C) {
  // C = A * B, where A is Rows x K, B is K x Cols
  // Column-major layout
  for(unsigned int col = 0; col < Cols; col++) {
    for(unsigned int row = 0; row < Rows; row++) {
      T sum = T(0);
      for(unsigned int k = 0; k < K; k++) {
        sum += A[k * Rows + row] * B[col * K + k];
      }
      C[col * Rows + row] = sum;
    }
  }
}

// ---- SIMD specialization for Mat<4,4> * Mat<4,4> float (SSE2) --------
#ifdef __SSE2__
#include <immintrin.h>

template <> inline void mat_mul_impl<4, 4, 4, float>(const float* A, const float* B, float* C) {
  // Optimized 4x4 float matrix multiplication using SSE2
  // A and B are 4x4 in column-major
  const __m128* Acol = reinterpret_cast<const __m128*>(A);
  __m128* Ccol       = reinterpret_cast<__m128*>(C);

  for(int col = 0; col < 4; col++) {
    __m128 b0 = _mm_set1_ps(B[col * 4 + 0]);
    __m128 b1 = _mm_set1_ps(B[col * 4 + 1]);
    __m128 b2 = _mm_set1_ps(B[col * 4 + 2]);
    __m128 b3 = _mm_set1_ps(B[col * 4 + 3]);

    __m128 result = _mm_mul_ps(Acol[0], b0);
    result        = _mm_add_ps(result, _mm_mul_ps(Acol[1], b1));
    result        = _mm_add_ps(result, _mm_mul_ps(Acol[2], b2));
    result        = _mm_add_ps(result, _mm_mul_ps(Acol[3], b3));

    _mm_store_ps(&C[col * 4], result);
  }
}

// Specialization for Mat<4, 4, float> * Vec<4, float>
template <> inline void mat_mul_impl<4, 4, 1, float>(const float* A, const float* B, float* C) {
  const __m128* Acol = reinterpret_cast<const __m128*>(A);
  __m128 b0          = _mm_set1_ps(B[0]);
  __m128 b1          = _mm_set1_ps(B[1]);
  __m128 b2          = _mm_set1_ps(B[2]);
  __m128 b3          = _mm_set1_ps(B[3]);

  __m128 result = _mm_mul_ps(Acol[0], b0);
  result        = _mm_add_ps(result, _mm_mul_ps(Acol[1], b1));
  result        = _mm_add_ps(result, _mm_mul_ps(Acol[2], b2));
  result        = _mm_add_ps(result, _mm_mul_ps(Acol[3], b3));

  _mm_store_ps(C, result);
}
#endif

} // namespace detail

// ---- Non-member operators ---------------------------------------------------

template <unsigned int R, unsigned int C, typename T> Mat<R, C, T> operator*(T s, const Mat<R, C, T>& m) { return m * s; }

template <unsigned int R, unsigned int C, typename T> std::ostream& operator<<(std::ostream& os, const Mat<R, C, T>& m) {
  os << "Mat<" << R << ", " << C << ">(\n";
  for(unsigned int r = 0; r < R; r++) {
    os << "  [";
    for(unsigned int c = 0; c < C; c++) {
      os << m(r, c);
      if(c < C - 1) os << ", ";
    }
    os << "]\n";
  }
  os << ")";
  return os;
}

// ---- Godot-inspired 3D transform functions (4x4 matrices) ------------------

// Translation matrix
template <typename T> Mat<4, 4, T> mat4_translation(const NVec<3, T>& t) {
  Mat<4, 4, T> m = Mat<4, 4, T>::identity();
  m(0, 3)        = t[0];
  m(1, 3)        = t[1];
  m(2, 3)        = t[2];
  return m;
}

// Scale matrix
template <typename T> Mat<4, 4, T> mat4_scale(const NVec<3, T>& s) {
  Mat<4, 4, T> m = Mat<4, 4, T>::identity();
  m(0, 0)        = s[0];
  m(1, 1)        = s[1];
  m(2, 2)        = s[2];
  return m;
}

// Rotation matrices (around principal axes)
template <typename T> Mat<4, 4, T> mat4_rotation_x(double angle) {
  Mat<4, 4, T> m = Mat<4, 4, T>::identity();
  T c            = static_cast<T>(std::cos(angle));
  T s            = static_cast<T>(std::sin(angle));
  m(1, 1)        = c;
  m(1, 2)        = -s;
  m(2, 1)        = s;
  m(2, 2)        = c;
  return m;
}

template <typename T> Mat<4, 4, T> mat4_rotation_y(double angle) {
  Mat<4, 4, T> m = Mat<4, 4, T>::identity();
  T c            = static_cast<T>(std::cos(angle));
  T s            = static_cast<T>(std::sin(angle));
  m(0, 0)        = c;
  m(0, 2)        = s;
  m(2, 0)        = -s;
  m(2, 2)        = c;
  return m;
}

template <typename T> Mat<4, 4, T> mat4_rotation_z(double angle) {
  Mat<4, 4, T> m = Mat<4, 4, T>::identity();
  T c            = static_cast<T>(std::cos(angle));
  T s            = static_cast<T>(std::sin(angle));
  m(0, 0)        = c;
  m(0, 1)        = -s;
  m(1, 0)        = s;
  m(1, 1)        = c;
  return m;
}

// Rodrigues' rotation formula: rotation by angle around axis
template <typename T> Mat<4, 4, T> mat4_rotation(const NVec<3, T>& axis, double angle) {
  T c          = static_cast<T>(std::cos(angle));
  T s          = static_cast<T>(std::sin(angle));
  T t          = T(1) - c;
  NVec<3, T> n = axis.normalized();
  T x = n[0], y = n[1], z = n[2];

  Mat<4, 4, T> m = Mat<4, 4, T>::identity();
  m(0, 0)        = t * x * x + c;
  m(0, 1)        = t * x * y - z * s;
  m(0, 2)        = t * x * z + y * s;

  m(1, 0) = t * x * y + z * s;
  m(1, 1) = t * y * y + c;
  m(1, 2) = t * y * z - x * s;

  m(2, 0) = t * x * z - y * s;
  m(2, 1) = t * y * z + x * s;
  m(2, 2) = t * z * z + c;
  return m;
}

// Look-at matrix (Godot: Transform3D::looking_at)
template <typename T> Mat<4, 4, T> mat4_look_at(const NVec<3, T>& eye, const NVec<3, T>& center, const NVec<3, T>& up) {
  NVec<3, T> f = (center - eye).normalized();
  NVec<3, T> s = f.cross(up).normalized();
  NVec<3, T> u = s.cross(f);

  Mat<4, 4, T> m = Mat<4, 4, T>::identity();
  m(0, 0)        = s[0];
  m(1, 0)        = s[1];
  m(2, 0)        = s[2];

  m(0, 1) = u[0];
  m(1, 1) = u[1];
  m(2, 1) = u[2];

  m(0, 2) = -f[0];
  m(1, 2) = -f[1];
  m(2, 2) = -f[2];

  m(0, 3) = -s.dot(eye);
  m(1, 3) = -u.dot(eye);
  m(2, 3) = f.dot(eye);
  return m;
}

// Perspective projection (like Godot's Projection::perspective)
template <typename T> Mat<4, 4, T> mat4_perspective(double fov_y, double aspect, double z_near, double z_far) {
  T f = static_cast<T>(1.0 / std::tan(fov_y * 0.5));
  Mat<4, 4, T> m;
  m.data[0]  = f / static_cast<T>(aspect);
  m.data[5]  = f;
  m.data[10] = static_cast<T>((z_far + z_near) / (z_near - z_far));
  m.data[14] = static_cast<T>(2.0 * z_far * z_near / (z_near - z_far));
  m.data[11] = T(-1);
  return m;
}

// Orthographic projection
template <typename T> Mat<4, 4, T> mat4_ortho(double left, double right, double bottom, double top, double z_near, double z_far) {
  Mat<4, 4, T> m = Mat<4, 4, T>::identity();
  m(0, 0)        = T(2.0 / (right - left));
  m(1, 1)        = T(2.0 / (top - bottom));
  m(2, 2)        = T(-2.0 / (z_far - z_near));
  m(0, 3)        = -T((right + left) / (right - left));
  m(1, 3)        = -T((top + bottom) / (top - bottom));
  m(2, 3)        = -T((z_far + z_near) / (z_far - z_near));
  return m;
}

// ---- Common type aliases (matching GLSL / GLM) ----------------------------

using Mat2f = Mat<2, 2, float>;
using Mat3f = Mat<3, 3, float>;
using Mat4f = Mat<4, 4, float>;

using Mat2d = Mat<2, 2, double>;
using Mat3d = Mat<3, 3, double>;
using Mat4d = Mat<4, 4, double>;

// Rectangular matrices
using Mat3x4f = Mat<3, 4, float>;
using Mat4x3f = Mat<4, 3, float>;

} // namespace Cutil
