#pragma once

#include <cstddef>
#include <cstdint>

// --- x86/x64 (SSE/AVX) ---
#if defined(__SSE__) || defined(__AVX__)  // Check for SSE or AVX support
#include <immintrin.h>
#define ARCH_X86_SSE 1
using Native4f = __m128;

// --- ARM (NEON) ---
#elif defined(__ARM_NEON) || defined(__aarch64__)  // Check for ARM NEON (32-bit or 64-bit)
#include <arm_neon.h>
#define ARCH_ARM_NEON 1
using Native4f = float32x4_t;
// --- Fallback (Scalar) ---
#else
#define ARCH_SCALAR 1
#endif

#if defined(__AVX2__)
#define ARCH_X86_AVX2 1
using Native4l = __m256i;
#endif

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
    data.v[0] = x;
    data.v[1] = y;
    data.v[2] = z;
    data.v[3] = w;
  }

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
    return data.v[index];
  }

  const float& operator[](size_t index) const {
    return data.v[index];
  }
};

// --- Zero Vector ---
inline Vector4f ZeroVector4f() {
#if defined(ARCH_X86_SSE)
  return Vector4f(_mm_setzero_ps());
#elif defined(ARCH_ARM_NEON)
  return Vector4f(vdupq_n_f32(0.0f));
#else  // Scalar
  return Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
#endif
}

static const Vector4f kVectorZero = ZeroVector4f();

//-----------------------------------------------------------------------
struct alignas(32) Vector4l {
  union alignas(32) {
#if defined(ARCH_X86_AVX2)
    Native4l m;  // Native 256-bit type (AVX2 only)
#endif
    int64_t v[4];   // Scalar representation
    uint8_t u[32];  // Byte representation
  } data;

  // --- Constructors ---
  Vector4l() = default;
  Vector4l(const Vector4l& other) = default;
  Vector4l& operator=(const Vector4l& other) = default;
  Vector4l(Vector4l&& other) = default;
  Vector4l& operator=(Vector4l&& other) = default;

  Vector4l(int64_t x, int64_t y, int64_t z, int64_t w) {
    data.v[0] = x;
    data.v[1] = y;
    data.v[2] = z;
    data.v[3] = w;
  }

  static size_t byte_size() {
    return 32;
  }

#if defined(ARCH_X86_AVX2)
  Vector4l(const Native4f& native) : data{native} {}
#endif

  Vector4l operator+(const Vector4l& other) const {
#if defined(ARCH_X86_AVX2)
    return Vector4l(_mm256_add_epi64(data.m, other.data.m));
#elif defined(ARCH_ARM_NEON)
    return Vector4l(vaddq_s64(data.m, other.data.m));
#else  // Scalar
    return Vector4l(
        data.v[0] + other.data.v[0],  //
        data.v[1] + other.data.v[1],  //
        data.v[2] + other.data.v[2],  //
        data.v[3] + other.data.v[3]);
#endif
  }

  Vector4l operator-(const Vector4l& other) const {
#if defined(ARCH_X86_AVX2)
    return Vector4l(_mm256_sub_epi64(data.m, other.data.m));
#elif defined(ARCH_ARM_NEON)
    return Vector4l(vsubq_s64(data.m, other.data.m));
#else  // Scalar
    return Vector4l(
        data.v[0] - other.data.v[0],  //
        data.v[1] - other.data.v[1],  //
        data.v[2] - other.data.v[2],  //
        data.v[3] - other.data.v[3]);
#endif
  }

  Vector4l operator*(const Vector4l& other) const {
#if defined(ARCH_X86_AVX2)
    return Vector4l(_mm256_mullo_epi64(data.m, other.data.m));
#elif defined(ARCH_ARM_NEON)
    return Vector4l(vmulq_s64(data.m, other.data.m));
#else  // Scalar
    return Vector4l(
        data.v[0] * other.data.v[0],  //
        data.v[1] * other.data.v[1],  //
        data.v[2] * other.data.v[2],  //
        data.v[3] * other.data.v[3]);
#endif
  }

  Vector4l operator/(const Vector4l& other) const {
#if defined(ARCH_X86_AVX2)
    return Vector4l(_mm256_div_epi64(data.m, other.data.m));
#elif defined(ARCH_ARM_NEON)
    return Vector4l(vdivq_s64(data.m, other.data.m));
#else  // Scalar
    return Vector4l(
        data.v[0] / other.data.v[0],  //
        data.v[1] / other.data.v[1],  //
        data.v[2] / other.data.v[2],  //
        data.v[3] / other.data.v[3]);
#endif
  }

  // --- Accessors ---
  int64_t& operator[](size_t index) {
    return data.v[index];
  }

  const int64_t& operator[](size_t index) const {
    return data.v[index];
  }
};

inline Vector4l cast_vector4f_to_Vector4l(const Vector4f& vec) {
#if defined(ARCH_X86_AVX2)
  // Convert four floats (__m128) to four 32-bit integers (__m128i).
  // Uses truncation. Use _mm_cvtroundps_epi32 for other rounding modes.
  __m128i temp_si128 = _mm_cvtps_epi32(vec.data.m);  // SSE2

  // Place the 128-bit integer data (containing 4x int32_t) into the lower lane,
  //  zeroing the upper 128 bits.
  __m256i result = _mm256_castsi128_si256(ints_si128);  // Requires AVX
  return Vector4l(result);
#else  // Scalar
  return Vector4l(
      static_cast<int64_t>(vec.data.v[0]),  //
      static_cast<int64_t>(vec.data.v[1]),  //
      static_cast<int64_t>(vec.data.v[2]),  //
      static_cast<int64_t>(vec.data.v[3]));
#endif
}