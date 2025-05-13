#pragma once

#include <cstddef>
#include <cstdint>

// --- Architecture Detection & Native Types ---

// --- x86/x64 (SSE/AVX) ---
// __AVX__ implies __SSE__, __SSE2__, __SSE3__, __SSSE3__, __SSE4_1__, __SSE4_2__
#if defined(__AVX__) || defined(__SSE__)
#include <immintrin.h>
#undef ARCH_SCALAR
#define ARCH_X86_SSE 1  // General flag for 128-bit SSE/AVX capabilities
using Native4f = __m128;
using Native4i = __m128i;  // Requires SSE2 for the type itself

// --- ARM (NEON) ---
#elif defined(__ARM_NEON) || defined(__aarch64__)
#include <arm_neon.h>
#undef ARCH_SCALAR
#define ARCH_ARM_NEON 1
using Native4f = float32x4_t;
using Native4i = int32x4_t;

#else
#define ARCH_SCALAR 1
#endif

//-----------------------------------------------------------------------
// Float vector (4 x floats)
//-----------------------------------------------------------------------
struct alignas(16) Vector4f {
  // Conditionally include the native SIMD type

  union alignas(16) {
#if defined(ARCH_X86_SSE) || defined(ARCH_ARM_NEON)
    Native4f m;
#endif
    float v[4];
    uint8_t u[16];
  } data;

  // Default constructor (optional, good practice)
  Vector4f() = default;

  Vector4f(const Vector4f& other) = default;
  Vector4f& operator=(const Vector4f& other) = default;
  Vector4f(Vector4f&& other) = default;
  Vector4f& operator=(Vector4f&& other) = default;

#if defined(ARCH_X86_SSE) || defined(ARCH_ARM_NEON)
  Vector4f(const Native4f& native) : data{native} {}
#endif

  Vector4f(float x, float y, float z, float w) {
#if defined(ARCH_X86_SSE)
    data.m = _mm_set_ps(w, z, y, x);
#elif defined(ARCH_ARM_NEON)
    data.m = vsetq_lane_f32(w, vsetq_lane_f32(z, vsetq_lane_f32(y, vdupq_n_f32(x), 2), 1), 3);
#else  // Scalar
    data.v[0] = x;
    data.v[1] = y;
    data.v[2] = z;
    data.v[3] = w;
#endif
  }

  void setZero() {
#if defined(ARCH_X86_SSE)
    data.m = _mm_setzero_ps();
#elif defined(ARCH_ARM_NEON)
    data.m = vdupq_n_f32(0.0f);
#else  // Scalar
    data.v[0] = 0.0f;
    data.v[1] = 0.0f;
    data.v[2] = 0.0f;
    data.v[3] = 0.0f;
#endif
  }

  // --- Size in bytes ---

  static size_t byte_size() {
    return 16;
  }

  Vector4f operator+(const Vector4f& other) const {
#if defined(ARCH_X86_SSE)
    return Vector4f(_mm_add_ps(data.m, other.data.m));
#elif defined(ARCH_ARM_NEON)
    return Vector4f(vaddq_f32(data.m, other.data.m));
#else  // Scalar
    return Vector4f(
        data.v[0] + other.data.v[0],  //
        data.v[1] + other.data.v[1],  //
        data.v[2] + other.data.v[2],  //
        data.v[3] + other.data.v[3]);
#endif
  }

  Vector4f operator-(const Vector4f& other) const {
#if defined(ARCH_X86_SSE)
    return Vector4f(_mm_sub_ps(data.m, other.data.m));
#elif defined(ARCH_ARM_NEON)
    return Vector4f(vsubq_f32(data.m, other.data.m));
#else  // Scalar
    return Vector4f(
        data.v[0] - other.data.v[0],  //
        data.v[1] - other.data.v[1],  //
        data.v[2] - other.data.v[2],  //
        data.v[3] - other.data.v[3]);
#endif
  }

  Vector4f operator*(const Vector4f& other) const {
#if defined(ARCH_X86_SSE)
    return Vector4f(_mm_mul_ps(data.m, other.data.m));
#elif defined(ARCH_ARM_NEON)
    return Vector4f(vmulq_f32(data.m, other.data.m));
#else  // Scalar
    return Vector4f(
        data.v[0] * other.data.v[0],  //
        data.v[1] * other.data.v[1],  //
        data.v[2] * other.data.v[2],  //
        data.v[3] * other.data.v[3]);
#endif
  }

  Vector4f operator/(const Vector4f& other) const {
#if defined(ARCH_X86_SSE)
    return Vector4f(_mm_div_ps(data.m, other.data.m));
#elif defined(ARCH_ARM_NEON)
    return Vector4f(vdivq_f32(data.m, other.data.m));
#else  // Scalar
    return Vector4f(
        data.v[0] / other.data.v[0],  //
        data.v[1] / other.data.v[1],  //
        data.v[2] / other.data.v[2],  //
        data.v[3] / other.data.v[3]);
#endif
  }

  // --- Accessors ---
  float& operator[](size_t index) {
    assert(index < 4);
    return data.v[index];
  }

  const float& operator[](size_t index) const {
    assert(index < 4);
    return data.v[index];
  }
};

//-----------------------------------------------------------------------
// Integers vector (4x int32_t)
//-----------------------------------------------------------------------

struct alignas(16) Vector4i {
  union alignas(16) {
#if defined(ARCH_X86_SSE) || defined(ARCH_ARM_NEON)
    Native4i m;  // Native 128-bit type
#endif
    int32_t v[4];   // Scalar representation
    uint8_t u[16];  // Byte representation
  } data;

  // --- Constructors ---
  Vector4i() = default;
  Vector4i(const Vector4i& other) = default;
  Vector4i& operator=(const Vector4i& other) = default;
  Vector4i(Vector4i&& other) = default;
  Vector4i& operator=(Vector4i&& other) = default;

  Vector4i(int32_t x, int32_t y, int32_t z, int32_t w) {
#if defined(ARCH_X86_SSE)
    data.m = _mm_set_epi32(w, z, y, x);
#else  // Scalar
    data.v[0] = x;
    data.v[1] = y;
    data.v[2] = z;
    data.v[3] = w;
#endif
  }

  void setZero() {
#if defined(ARCH_X86_SSE)
    data.m = _mm_setzero_si128();
#elif defined(ARCH_ARM_NEON)
    data.m = vdupq_n_s32(0);
#else  // Scalar
    data.v[0] = 0;
    data.v[1] = 0;
    data.v[2] = 0;
    data.v[3] = 0;
#endif
  }

  static size_t byte_size() {
    return 32;
  }

// Constructor from native SIMD type
#if defined(ARCH_X86_SSE) || defined(ARCH_ARM_NEON)
  Vector4i(const Native4i& native) : data{native} {}
#endif

  Vector4i operator+(const Vector4i& other) const {
#if defined(ARCH_X86_SSE)
    return Vector4i(_mm_add_epi32(data.m, other.data.m));
#elif defined(ARCH_ARM_NEON)
    return Vector4i(vaddq_s32(data.m, other.data.m));
#else  // Scalar fallback
    return Vector4i(
        data.v[0] + other.data.v[0],  //
        data.v[1] + other.data.v[1],  //
        data.v[2] + other.data.v[2],  //
        data.v[3] + other.data.v[3]);
#endif
  }

  Vector4i operator-(const Vector4i& other) const {
#if defined(ARCH_X86_SSE)
    return Vector4i(_mm_sub_epi32(data.m, other.data.m));
#elif defined(ARCH_ARM_NEON)
    return Vector4i(vsubq_s32(data.m, other.data.m))
#else  // Scalar fallback
    return Vector4i(
        data.v[0] - other.data.v[0],  //
        data.v[1] - other.data.v[1],  //
        data.v[2] - other.data.v[2],  //
        data.v[3] - other.data.v[3]);
#endif
  }

  Vector4i operator*(const Vector4i& other) const {
#if defined(__SSE4_1__)                                      // Requires SSE4.1
    return Vector4i(_mm_mullo_epi32(data.m, other.data.m));  // SSE4.1 intrinsic
#elif defined(ARCH_ARM_NEON)
    return Vector4i(vmulq_s32(data.m, other.data.m));
#else  // Scalar
    return Vector4i(
        data.v[0] * other.data.v[0],  //
        data.v[1] * other.data.v[1],  //
        data.v[2] * other.data.v[2],  //
        data.v[3] * other.data.v[3]);
#endif
  }

  Vector4i operator/(const Vector4i& other) const {
    return Vector4i(
        data.v[0] / other.data.v[0],  //
        data.v[1] / other.data.v[1],  //
        data.v[2] / other.data.v[2],  //
        data.v[3] / other.data.v[3]);
  }

  // --- Accessors ---
  int32_t& operator[](size_t index) {
    assert(index < 4);
    return data.v[index];
  }

  const int32_t& operator[](size_t index) const {
    assert(index < 4);
    return data.v[index];
  }
};

//-----------------------------------------------------------------------
// Casting
//-----------------------------------------------------------------------

// Cast Vector4f (float) to Vector4i (int32_t) using truncation
inline Vector4i cast_vector4f_to_Vector4i(const Vector4f& float_vec) {
#if defined(ARCH_X86_SSE)  // Requires SSE2
  // Converts four floats (__m128) to four 32-bit signed integers (__m128i)
  // using truncation (round towards zero).
  return Vector4i(_mm_cvtps_epi32(float_vec.data.m));
#elif defined(ARCH_ARM_NEON)
  // Converts four floats (float32x4_t) to four 32-bit signed integers (int32x4_t)
  // using truncation (round towards zero).
  return Vector4i(vcvtq_s32_f32(float_vec.data.m));
#else  // Scalar fallback
  return Vector4i(
      static_cast<int32_t>(float_vec.data.v[0]),  //
      static_cast<int32_t>(float_vec.data.v[1]),  //
      static_cast<int32_t>(float_vec.data.v[2]),  //
      static_cast<int32_t>(float_vec.data.v[3]));
#endif
}