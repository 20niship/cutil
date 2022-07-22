#pragma once

#include <math.h>
#include <iostream>
#include <functional>
#include <assert.h>
#include <initializer_list>
#include <vector>
#include <cmath>
#include <limits>
#include <cassert>
#include <iostream>
#include <string>
#include <iostream>
#include <type_traits>
#include <cstring>

#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wsign-compare" // May be used uninitialized 'return {};'.

#define USE_DEFAULT_VECTORS_AND_MATRIX
#define MY_ASSERT(A) assert(A)

// #ifndef MAX
// #define MAX(a, b) ((a) > (b) ? (a) : (b))
// #endif
// #ifndef MIN
// #define MIN(a, b) ((a) < (b) ? (a) : (b))
// #endif

#define disp(A) (std::cout << #A << " = " << (A) << "\n")

#ifdef VKUI_USE_CONCEPTS
    #include <concepts>
    template <typename T> 
    concept RealNumberConcept = requires(T a) { (double)a; (int)a; }; // doubleとintに変換可能な型
#endif

#ifdef USE_CUSTOM_VECTOR3
template<typename T>
struct Vector3 {
	T x, y, z;
	Vector3() { reset(); }
	Vector3(float x_, float y_, float z_) { x = x_; y = y_; z = z_; }
	void reset() { x = 0; y = 0; z = 0; }
	T operator[](int i) {if (i == 0) { return x; }else if (i == 1) { return y; }else return z;}

	template<typename U> Vector3 operator+=(const Vector3<U>& other) { x += other.x; y += other.y; z+=other.z; return *this; }
	template<typename U> Vector3 operator-=(const Vector3<U>& other)    { x -= other.x; y -= other.y; return *this; }
	template<typename U> Vector3 operator*=(const U& mul) { x = x * mul; y = y * mul; return *this; }
	template<typename U> Vector3 operator/=(const U& div) { x = x / div; y = y / div; return *this; }

	template<typename U> Vector3 operator-(const Vector3<U>& other) { return Vector3<T>(x - other.x, y - other.y, z - other.z); }
	template<typename U> Vector3 operator+(const Vector3<U>& other) const { std::cout << "a" << std::endl; return Vector3<T>(x + other.x, y + other.y, z + other.z); }
	template<typename U> Vector3 operator*(const Vector3<U>& other) { return Vector3<T>(x * other.x, y * other.y, z * other.z); }
	template<typename U> inline Vector3 operator+(const U& value)const{ return Vector3<T>(x + value,   y + value,   z + value  ); }
	template<typename U> Vector3 operator-(const U& value)          { return Vector3<T>(x - value,   y - value,   z - value  ); }
	template<typename U> Vector3 operator*(const U& value)          { return Vector3<T>(x * value,   y * value,   z * value  ); }
	template<typename U> Vector3 operator/(const U& value)          { return Vector3<T>(x / value,   y / value,   z / value  ); }

	template<typename U> double dot(const Vector3<U>& value) { return (x *value.x  +  y*value.y + z*value.z); }

  	// T operator[](int i)            { MY_ASSERT(i < 3); return (i == 0)? x : (i==1 ? y : z); }
  	const T& operator[](int i)const{ MY_ASSERT(i < 3); return (i == 0)? x : (i==1 ? y : z); }
    template<typename U> 
	bool operator==(const Vector3<U>& other) const { return x==other.x && y==other.y && z == other.z; }
    template<typename U> 
	bool operator!=(const Vector3<U>& other) const { return !(*this == other); }
    double norm() const {return std::sqrt(x*x + y*y + z*z);}
};
#endif

template<typename T, unsigned int LEN>
class _Vec {
public:
	T value[LEN];
	_Vec() { reset(); }
	_Vec(std::initializer_list<T> init) { MY_ASSERT(init.size() >= LEN);int idx=0; for(auto i=init.begin(); i<init.end(); i++){ value[idx] = *i; idx++; } }

    // template<class... Args>
    // _Vec(Args... args){
    //     constexpr std::size_t size = sizeof...(Args);
    //     static_assert(size == LEN);
    //     std::vector<T> tt{ args... };
    //     for(int i=0; i<LEN; i++){value[i] = tt[i];}
    // }
	_Vec(const T x_, const T y_)                                              { static_assert(LEN >= 2); value[0] = x_; value[1] = y_; }
	_Vec(const T x_, const T y_, const T z_)                                  { static_assert(LEN >= 3); value[0] = x_; value[1] = y_; value[2]= z_; }
	_Vec(const T x_, const T y_, const T z_, const T a)                       { static_assert(LEN >= 4); value[0] = x_; value[1] = y_; value[2]= z_; value[3]= a; }
	_Vec(const T x_, const T y_, const T z_, const T a, const T b)            { static_assert(LEN >= 5); value[0] = x_; value[1] = y_; value[2]= z_; value[3]= a; value[4] = b;}
	_Vec(const T x_, const T y_, const T z_, const T a, const T b, const T c) { static_assert(LEN >= 6); value[0] = x_; value[1] = y_; value[2]= z_; value[3]= a; value[4] = b; value[5] = c;}

	inline void reset() { for(int i=0; i<LEN; i++){ value[i] = 0; } }

	template<typename U> inline _Vec operator+=(const _Vec<U, LEN>& other) { for(int i=0; i<LEN; i++){value[i] += other[i];}return *this; }
	template<typename U> inline _Vec operator-=(const _Vec<U, LEN>& other) { for(int i=0; i<LEN; i++){value[i] -= other[i];}return *this; }
	template<typename U> inline _Vec operator+=(const U& other)          { for(int i=0; i<LEN; i++){value[i] += other; }return *this; }
	template<typename U> inline _Vec operator-=(const U& other)          { for(int i=0; i<LEN; i++){value[i] -= other; }return *this; }
	template<typename U> inline _Vec operator*=(const U& other)          { for(int i=0; i<LEN; i++){value[i] *= other; }return *this; }
	template<typename U> inline _Vec operator/=(const U& other)          { for(int i=0; i<LEN; i++){value[i] /= other; }return *this; }

	template<typename U> inline _Vec operator+(const _Vec<U, LEN>& other) const { _Vec<T, LEN> t; for(int i=0; i<LEN; i++){t[i] = value[i] + other[i];} return t; }
	template<typename U> inline _Vec operator-(const _Vec<U, LEN>& other) const { _Vec<T, LEN> t; for(int i=0; i<LEN; i++){t[i] = value[i] - other[i];} return t; }
	template<typename U> inline _Vec operator*(const _Vec<U, LEN>& other) const { _Vec<T, LEN> t; for(int i=0; i<LEN; i++){t[i] = value[i] * other[i];} return t; }
	template<typename U> inline _Vec operator/(const _Vec<U, LEN>& other) const { _Vec<T, LEN> t; for(int i=0; i<LEN; i++){t[i] = value[i] / other[i];} return t; }

#ifdef VKUI_USE_CONCEPTS
	template<RealNumberConcept U> inline _Vec operator+(const U& other)   const { _Vec<T, LEN> t; for(int i=0; i<LEN; i++){t[i] = value[i] + other;} return t;}
	template<RealNumberConcept U> inline _Vec operator-(const U& other)   const { _Vec<T, LEN> t; for(int i=0; i<LEN; i++){t[i] = value[i] - other;} return t;}
	template<RealNumberConcept U> inline _Vec operator*(const U& other)   const { _Vec<T, LEN> t; for(int i=0; i<LEN; i++){t[i] = value[i] * other;} return t;}
	template<RealNumberConcept U> inline _Vec operator/(const U& other)   const { _Vec<T, LEN> t; for(int i=0; i<LEN; i++){t[i] = value[i] / other;} return t;}
#else
	inline _Vec operator+(const double& other)                            const { _Vec<T, LEN> t; for(int i=0; i<LEN; i++){t[i] = value[i] + other;} return t;}
	inline _Vec operator-(const double& other)                            const { _Vec<T, LEN> t; for(int i=0; i<LEN; i++){t[i] = value[i] - other;} return t;}
	inline _Vec operator*(const double& other)                            const { _Vec<T, LEN> t; for(int i=0; i<LEN; i++){t[i] = value[i] * other;} return t;}
	inline _Vec operator/(const double& other)                            const { _Vec<T, LEN> t; for(int i=0; i<LEN; i++){t[i] = value[i] / other;} return t;}
#endif

    template<typename U> inline double dot(const _Vec<U, LEN>& other)const { double s{0}; for(int i=0; i<LEN; i++){ s += value[i] * other[i];    } return s; }
	                     inline _Vec operator-()                          { _Vec<T, LEN> t; for(int i=0; i<LEN; i++){t[i] = -value[i];       } return t; }
	
    template<typename U> bool operator< (const _Vec<U, LEN>& other) const {for(int i=0; i<LEN; i++){ if(value[i] >= other[i])return false; } return true; }
    template<typename U> bool operator<=(const _Vec<U, LEN>& other) const {for(int i=0; i<LEN; i++){ if(value[i] >  other[i])return false; } return true; }
    template<typename U> bool operator> (const _Vec<U, LEN>& other) const {for(int i=0; i<LEN; i++){ if(value[i] <= other[i])return false; } return true; }
    template<typename U> bool operator>=(const _Vec<U, LEN>& other) const {for(int i=0; i<LEN; i++){ if(value[i] <  other[i])return false; } return true; }
    template<typename U> bool operator==(const _Vec<U, LEN>& other) const {for(int i=0; i<LEN; i++){ if(value[i] != other[i])return false; } return true; }
    template<typename U> bool operator!=(const _Vec<U, LEN>& other) const {for(int i=0; i<LEN; i++){ if(value[i] == other[i])return false; } return true; }
    // template<typename U, int LEN2> inline _Mat outer_product(const _Vec<U, LEN2>& other)     {  
    //     _Mat<T, LEN1, LEN2> s; for(int i=0; i<LEN; i++){ for(int j=0; j<LEN2; j++){ s(i, j) = value[i] * other[j]; } }
    //     return s;
    // }

  	inline const T& operator[](int i)const{ MY_ASSERT(i < LEN); return value[i]; }
  	inline T& operator[](int i)           { MY_ASSERT(i < LEN); return value[i]; }
    inline double norm() const { double s = 0;  for(int i=0; i<LEN; i++){s += value[i] * value[i]; } return sqrtf(s); }
    inline double norm_sq() const { double s = 0;  for(int i=0; i<LEN; i++){s += value[i] * value[i]; } return s; }
	inline _Vec normalize() const { return (*this) / norm(); }
    template<typename U> inline _Vec<T,3> cross(const _Vec<U,3>& other) const { static_assert(LEN == 3); _Vec<T, 3> res; 
        res[0] = value[1]*other[2] - value[2]*other[1];
        res[1] = value[2]*other[0] - value[0]*other[2];
        res[2] = value[0]*other[1] - value[1]*other[0];
        return res;
    }

	inline void display() {  std::cout << "value = ["; for(int i = 0; i < LEN; i++) { std::cout << value[i] << ", "; } std::cout << "\n"; }
    inline void all(const T &v){for(int i=0; i<LEN; i++){value[i] = v;}}
    inline void setMax(){ all(std::numeric_limits<T>::max()); }
    inline void setMin(){ all(std::numeric_limits<T>::min()); }
    inline void zero(){all(0);}
    inline void ones(){all(1);}
};


template<typename T,unsigned int LEN>
bool operator>(_Vec<T, LEN>& t1, _Vec<T, LEN>& t2)  { 
    bool r = true;
    for(int i=0; i<LEN; i++){ r&= t1[i] > t2[i]; }
    return r;
}

template<typename T, unsigned int LEN>
bool operator<( _Vec<T, LEN>& t1, _Vec<T, LEN>& t2) { 
    bool r = true;
    for(int i=0; i<LEN; i++){ r&= t1[i] < t2[i]; }
    return r;
}

template<typename T, unsigned int LEN>
std::ostream& operator<<(std::ostream& os, const _Vec<T, LEN>& t){
    os << " _Vector<" << std::string(typeid(T).name()) << ", " << LEN << "> [ ";
    for(int i=0; i<LEN; i++){os << t[i]; if(i != LEN-1){os << ", "; }}
    os << " ] ";
    return os;
}

using Vector2 = _Vec<double, 2>;
#ifndef USE_CUSTOM_VECTOR3
using Vector3 = _Vec<double, 3>;
#endif
using Vector6 = _Vec<double, 6>;
using Vector4 = _Vec<double, 4>;

using Vector2b = _Vec<unsigned char, 2>;
using Vector3b = _Vec<unsigned char, 3>;
using Vector4b = _Vec<unsigned char, 4>;
using Vector5b = _Vec<unsigned char, 5>;
using Vector6b = _Vec<unsigned char, 6>;

using Vector2s = _Vec<uint16_t, 2>;
using Vector3s = _Vec<uint16_t, 3>;
using Vector4s = _Vec<uint16_t, 4>;
using Vector5s = _Vec<uint16_t, 5>;
using Vector6s = _Vec<uint16_t, 6>;

using Vector2f = _Vec<float, 2>;
using Vector3f = _Vec<float, 3>;
using Vector4f = _Vec<float, 4>;
using Vector5f = _Vec<float, 5>;
using Vector6f = _Vec<float, 6>;

using Vector2d = _Vec<int64_t, 2>;
using Vector3d = _Vec<int64_t, 3>;
using Vector4d = _Vec<int64_t, 4>;
using Vector5d = _Vec<int64_t, 5>;
using Vector6d = _Vec<int64_t, 6>;

using Vector3S16 = _Vec<int16_t, 3>;
using Vector3U16 = _Vec<uint16_t, 3>;
using Vector3S32 = _Vec<int32_t, 3>;
using Vector3U32 = _Vec<uint32_t, 3>;


template<typename T, unsigned int W, unsigned int H>
struct _Mat{
    class CommaInput{
      private:
        _Mat *m;
        int index;
      public:
        CommaInput(_Mat *m_, int i){ m = m_; index = i;  }
        CommaInput &operator ,(T v){ m->value[index] = v;index++; return *this;}
    };

	T value[W*H];
	_Mat() { reset(); }
	_Mat(const _Mat<T, W, H>&t) { for(int i=0; i<W*H; i++){ value[i] = t[i]; }  }
	_Mat(const std::initializer_list<T> init) { MY_ASSERT(init.size() >= W*H);int idx=0; for(auto i=init.begin(); i<init.end(); i++){ value[idx] = *i; idx++; } }
  _Mat<T, W, H> &operator=(const _Mat<T, W, H> &o) = default;
	inline void reset() { for(int i=0; i<W*H; i++){ value[i] = 0; } }
	template<typename U> inline _Mat operator+=(const _Mat<U, W, H>& other) { for(int i=0; i<W*H; i++){value[i] += other[i];}return *this; }
	template<typename U> inline _Mat operator-=(const _Mat<U, W, H>& other) { for(int i=0; i<W*H; i++){value[i] -= other[i];}return *this; }
	template<typename U> inline _Mat operator+(const _Mat<U, W, H>& other) { _Mat<T, W,H> t; for(int i=0; i<W*H; i++){t[i] = value[i] + other[i];} return t; }
	template<typename U> inline _Mat operator-(const _Mat<U, W, H>& other) { _Mat<T, W,H> t; for(int i=0; i<W*H; i++){t[i] = value[i] - other[i];} return t; }

#ifdef VKUI_USE_CONCEPTS
	template<RealNumberConcept  U> inline _Mat operator+=(const U& other)  { for(int i=0; i<W*H; i++){value[i] += other; }return *this; }
	template<RealNumberConcept  U> inline _Mat operator-=(const U& other)  { for(int i=0; i<W*H; i++){value[i] -= other; }return *this; }
	template<RealNumberConcept  U> inline _Mat operator*=(const U& other)  { for(int i=0; i<W*H; i++){value[i] *= other; }return *this; }
	template<RealNumberConcept  U> inline _Mat operator/=(const U& other)  { for(int i=0; i<W*H; i++){value[i] /= other; }return *this; }
	template<RealNumberConcept  U> inline _Mat operator+(const U& other)   { _Mat<T, W,H> t; for(int i=0; i<W*H; i++){t[i] = value[i] + other;} return t;}
	template<RealNumberConcept  U> inline _Mat operator-(const U& other)   { _Mat<T, W,H> t; for(int i=0; i<W*H; i++){t[i] = value[i] - other;} return t;}
    template<RealNumberConcept  U> inline _Mat operator*(const U& other)   { _Mat<T, W,H> t; for(int i=0; i<W*H; i++){t[i] = value[i] * other;} return t;}
	template<RealNumberConcept  U> inline _Mat operator/(const U& other)   { _Mat<T, W,H> t; for(int i=0; i<W*H; i++){t[i] = value[i] / other;} return t;}
#else
	inline _Mat operator+=(const double other)  { for(int i=0; i<W*H; i++){value[i] += other; }return *this; }
	inline _Mat operator-=(const double other)  { for(int i=0; i<W*H; i++){value[i] -= other; }return *this; }
	inline _Mat operator*=(const double other)  { for(int i=0; i<W*H; i++){value[i] *= other; }return *this; }
	inline _Mat operator/=(const double other)  { for(int i=0; i<W*H; i++){value[i] /= other; }return *this; }
	inline _Mat operator+(const double other)   { _Mat<T, W,H> t; for(int i=0; i<W*H; i++){t[i] = value[i] + other;} return t;}
	inline _Mat operator-(const double other)   { _Mat<T, W,H> t; for(int i=0; i<W*H; i++){t[i] = value[i] - other;} return t;}
    inline _Mat operator*(const double other)   { _Mat<T, W,H> t; for(int i=0; i<W*H; i++){t[i] = value[i] * other;} return t;}
	inline _Mat operator/(const double other)   { _Mat<T, W,H> t; for(int i=0; i<W*H; i++){t[i] = value[i] / other;} return t;}
#endif


  	// T operator[](int i)            { MY_ASSERT(i < 3); return (i == 0)? x : (i==1 ? y : z); }
  	inline const T& operator[](int i)const{ MY_ASSERT(i < W*H); return value[i]; }
	inline T&       operator[](int i)     { MY_ASSERT(i < W*H); return value[i]; }
    inline const T& operator()(int x, int y) const { MY_ASSERT(x*y < W*H); return value[y*W + x];}
    inline T&       operator()(int x, int y)       { MY_ASSERT(x*y < W*H); return value[y*W + x];}
    CommaInput operator << (int v)        { value[0] = v; return CommaInput(this, 1); }

    template<typename U> inline bool operator==(const _Mat<U, W,H>& other) const { bool s = true; for(int i=0; i<W*H; i++){s &= value[i] == other[i]; } return s; }
    template<typename U> inline bool operator!=(const _Mat<U, W,H>& other) const { return !(*this == other); }

    inline double norm() const { double s = 0;  for(int i=0; i<W*H; i++){s += value[i] * value[i]; } return s; }
    inline void  all(T v)      { for(int i=0; i<W*H; i++){value[i] = v; }; }
    inline void  zeros()       { all(0); }
    inline void  ones()        { all(1); }
    inline void  identify()    { for(int i=0; i<W*H; i++){ value[i] = (i%(W+1) == 0) ? 1 : 0; } }
    inline double  trace()     { double s = 0; for(int i=0; i<W*H; i++){ if(i%(W+1) == 0) {s += value[i]; } } return s; }
	inline int   size() const  { return W*H; }
    inline int   width() const { return W; }
    inline int   height() const{ return H; }
    inline T*    begin() const { return &value; }
    inline T*    end()   const { return &value + W*H; }
    [[deprecated]] void  transpose()    const {  _Mat<T, H, W> tmp(*this); int x,y; for(int i=0; i< W*H; i++){ x = i%W; y = i/W; value[y*W + x] = tmp[x*H+y]; } }
	template<typename U, unsigned int W2> _Mat operator* (const _Mat<U, W2, W>& other) const { _Mat<T, W2, H> out; for(int x=0; x<H; x++){ for(int y=0; y < W2; y++){ T A = 0; for(int i=0; i<W; i++){ A += other(i, x) * (*this)(y,i); }  out(y,x) = A;  } } return out; } 
	template<typename U> _Vec<T, H> operator* (const _Vec<U, W>& other) const { _Vec<T, H> out; for(int x=0; x<H; x++){ T A = 0;  for(int i=0; i<W; i++){ A += value[x*H + i] * other[i]; } out[x] = A; } return out; }
	_Mat<T, H, W> Trans() const { _Mat<T, H, W> out; for(int x=0; x<W; x++){ for(int y=0; y<H; y++){ out(y, x) = value[y*W + x]; } } return out; }

    // T& operator= (const _Mat& other){
    //     if(other.width()*other.height() > W*H){
    //         _grow_capacity(other.width()*other.height());
    //         W = other.width(); H = other.height();
    //     }
    //     std::memcpy(Data, other.begin(), W*H*sizeof(T));
    // }

    float det() const {
        MY_ASSERT(W == H); 
        _Mat<T, W, H> tmp = (*this);
        T buf;
        int i,j,k;
        //三角行列を作成
        for(i=0;i<W;i++){ for(j=0;j<W;j++){ if(i<j){ buf=tmp[j*W + i]/tmp[i*W + i]; for(k=0;k<W;k++){ tmp[j*W + k]-=tmp[i*W+k]*buf;} } } }
        //対角部分の積
        double det = 1.0f;
        for(i=0;i<W;i++){ det*=tmp[i*W + i]; }
        return det;
    }

    void display() const {
		std::cout << "data = [ " << std::endl;
        for(int i=0; i < W; i++){
			std::cout << "    [ ";
            for(int j=0;j<H;j++){
				std::cout << value[j*W+i] << ", ";
            }
			std::cout << " ]" << std::endl ;
        }
		std::cout << "]" << std::endl;
    }

    _Mat inv() const {
        MY_ASSERT(W == H); 
        _Mat<T, W, W> inverse;
        _Mat<T, W, H> tmp(*this);
        float buf; //一時的なデータを蓄える
        int i,j,k; //カウンタ
        for(int i=0; i< W; i++) inverse[i*(W+1)] = 1.0;  
        //掃き出し法
        for(i=0; i < W; i++){
            buf=1.0f / tmp[i*W+i];
            for(j=0; j < W; j++){
                tmp[i*W+j]*=buf;
                inverse[i*W+j]*=buf;
            }
            for(j=0;j<W;j++){
            if(i != j){
                buf=tmp[j*W+i];
                for(k=0;k<W;k++){
                    tmp[j*W+k]-=tmp[i*W+k]*buf;
                    inverse[j*W+k]-=inverse[i*W+k]*buf;
                }
            }
            }
        }
        return inverse;
    }
};

template<typename T, unsigned int W, unsigned int H>
std::ostream& operator<<(std::ostream& os, const _Mat<T, W, H>& t){
    os << " _Mat<" << std::string(typeid(T).name()) << ", " << W  << "," << H << "> [ ";
    for(int i=0; i<H; i++){ for(int j=0; j<W;j++){ os << t[i*H+j] << ", "; } os << "\n"; }
    os << " ] " << std::endl;
    return os;
}

namespace MathUtils{
    template<typename T, typename U, unsigned int LEN, unsigned int LEN2> 
    inline _Mat<T, LEN2, LEN> outer_product(const _Vec<T, LEN> &A, const _Vec<U, LEN2> &B)     {  
        _Mat<T, LEN2, LEN> s; 
        for(int i=0; i<LEN; i++){ for(int j=0; j<LEN2; j++){ s(j, i) = A[i] * B[j]; } }
        return s;
    }
};


using Mat2x2 = _Mat<double, 2, 2>;
using Mat3x3 = _Mat<double, 3, 3>;
using Mat4x4 = _Mat<double, 4, 4>;
using Mat5x5 = _Mat<double, 5, 5>;
// using Mat3x6 = _Mat<double, 3, 6>;
using Mat3x6 = _Mat<double, 6, 3>;
using Mat6x6 = _Mat<double, 6, 6>;

using Matrix3x3 = _Mat<double, 3, 3>;
using Matrix6x6 = _Mat<double, 6, 6>;



template<typename T>
struct uiVector
{    int                 Size;
    int                 Capacity;
    T*                  Data;

    // Provide standard typedefs but we don't use them ourselves.
    typedef T                   value_type;
    typedef value_type*         iterator;
    typedef const value_type*   const_iterator;

    // Constructors, destructor
    inline uiVector()                                       { Size = Capacity = 0; Data = NULL; }
    inline uiVector(const uiVector<T>& src)                 { Size = Capacity = 0; Data = NULL; operator=(src); }
    inline uiVector<T>& operator=(const uiVector<T>& src)   { clear(); resize(src.Size); std::memcpy(Data, src.Data, (size_t)Size * sizeof(T)); return *this; }
    inline ~uiVector()                                      { if (Data) free(Data); }

    inline bool         empty() const                       { return Size == 0; }
    inline int          size() const                        { return Size; }
    inline int          size_in_bytes() const               { return Size * (int)sizeof(T); }
    inline int          capacity() const                    { return Capacity; }
    inline T&           operator[](int i)                   { MY_ASSERT(i < Size); return Data[i]; }
    inline const T&     operator[](int i) const             { MY_ASSERT(i < Size); return Data[i]; }

    inline void         clear()                             { if (Data) { Size = Capacity = 0; free(Data); Data = NULL; } }
    inline T*           begin()                             { return Data; }
    inline T*           data()                              { return Data; }
    inline const T*     begin() const                       { return Data; }
    inline T*           end()                               { return Data + Size; }
    inline const T*     end() const                         { return Data + Size; }
    inline T&           front()                             { MY_ASSERT(Size > 0); return Data[0]; }
    inline const T&     front() const                       { MY_ASSERT(Size > 0); return Data[0]; }
    inline T&           back()                              { MY_ASSERT(Size > 0); return Data[Size - 1]; }
    inline const T&     back() const                        { MY_ASSERT(Size > 0); return Data[Size - 1]; }
    inline void         swap(uiVector<T>& rhs)              { int rhs_size = rhs.Size; rhs.Size = Size; Size = rhs_size; int rhs_cap = rhs.Capacity; rhs.Capacity = Capacity; Capacity = rhs_cap; T* rhs_data = rhs.Data; rhs.Data = Data; Data = rhs_data; }

    inline int          _grow_capacity(int sz) const        { int new_capacity = Capacity ? (Capacity + Capacity/2) : 8; return new_capacity > sz ? new_capacity : sz; }
    inline void         resize(int new_size)                { if (new_size > Capacity) reserve(_grow_capacity(new_size)); Size = new_size; }
    inline void         resize(int new_size, const T& v)    { if (new_size > Capacity) reserve(_grow_capacity(new_size)); if (new_size > Size) for (int n = Size; n < new_size; n++) std::memcpy(&Data[n], &v, sizeof(v)); Size = new_size; }
    inline void         shrink(int new_size)                { MY_ASSERT(new_size <= Size); Size = new_size; } // Resize a vector to a smaller size, guaranteed not to cause a reallocation
    inline void         reserve(int new_capacity)           { if (new_capacity <= Capacity) return; T* new_data = (T*)malloc((size_t)new_capacity * sizeof(T)); if (Data) { std::memcpy(new_data, Data, (size_t)Size * sizeof(T)); free(Data); } Data = new_data; Capacity = new_capacity; }

    // NB: It is illegal to call push_back/push_front/insert with a reference pointing inside the uiVector data itself! e.g. v.push_back(v[10]) is forbidden.
    inline void         push_back(const T& v)               { if (Size == Capacity) reserve(_grow_capacity(Size + Size/2 + 100)); std::memcpy(&Data[Size], &v, sizeof(v)); Size++; }
    inline void         push_back(const T&& v)              { if (Size == Capacity) reserve(_grow_capacity(Size + Size/2 + 100)); std::memcpy(&Data[Size], &v, sizeof(v)); Size++; }
    inline void         push_back_ptr(const T *v)              { if (Size == Capacity) reserve(_grow_capacity(Size + Size/2 + 100)); std::memcpy(&Data[Size], v, sizeof(T)); Size++; }
    inline void         push_back(std::initializer_list<T> v){ for (auto it = v.begin(); it != v.end(); ++it) { push_back(*it); }}
    inline void         push_back(std::initializer_list<T> v, int n){ T *it; for(int i=0; i<n; i++){ for ( it = v.begin(); it != v.end(); ++it) { push_back(*it); }}}
    inline void         push_back(const T& v, int n)        { T *it; for(int i=0; i<n; i++){ push_back(v); }}
    inline void         pop_back()                          { MY_ASSERT(Size > 0); Size--; }
    inline void         push_front(const T& v)              { if (Size == 0) push_back(v); else insert(Data, v); }
    inline T*           erase(const T* it)                  { MY_ASSERT(it >= Data && it < Data+Size); const ptrdiff_t off = it - Data; memmove(Data + off, Data + off + 1, ((size_t)Size - (size_t)off - 1) * sizeof(T)); Size--; return Data + off; }
    inline T*           erase(const T* it, const T* it_last){ MY_ASSERT(it >= Data && it < Data+Size && it_last > it && it_last <= Data+Size); const ptrdiff_t count = it_last - it; const ptrdiff_t off = it - Data; memmove(Data + off, Data + off + count, ((size_t)Size - (size_t)off - count) * sizeof(T)); Size -= (int)count; return Data + off; }
    inline T*           erase_unsorted(const T* it)         { MY_ASSERT(it >= Data && it < Data+Size);  const ptrdiff_t off = it - Data; if (it < Data+Size-1) std::memcpy(Data + off, Data + Size - 1, sizeof(T)); Size--; return Data + off; }
    inline T*           insert(const T* it, const T& v)     { MY_ASSERT(it >= Data && it <= Data+Size); const ptrdiff_t off = it - Data; if (Size == Capacity) reserve(_grow_capacity(Size + 1)); if (off < (int)Size) memmove(Data + off + 1, Data + off, ((size_t)Size - (size_t)off) * sizeof(T)); std::memcpy(&Data[off], &v, sizeof(v)); Size++; return Data + off; }
    inline bool         contains(const T& v) const          { const T* data = Data;  const T* data_end = Data + Size; while (data < data_end) if (*data++ == v) return true; return false; }
    inline T*           find(const T& v)                    { T* data = Data;  const T* data_end = Data + Size; while (data < data_end) if (*data == v) break; else ++data; return data; }
    inline const T*     find(const T& v) const              { const T* data = Data;  const T* data_end = Data + Size; while (data < data_end) if (*data == v) break; else ++data; return data; }
    inline bool         find_erase(const T& v)              { const T* it = find(v); if (it < Data + Size) { erase(it); return true; } return false; }
    inline bool         find_erase_unsorted(const T& v)     { const T* it = find(v); if (it < Data + Size) { erase_unsorted(it); return true; } return false; }
    inline int          index_from_ptr(const T* it) const   { MY_ASSERT(it >= Data && it < Data + Size); const ptrdiff_t off = it - Data; return (int)off; }
    inline void         fill(const T& v)const               { for(int i=0; i<Size; i++){ Data[i] = v; } }
};



