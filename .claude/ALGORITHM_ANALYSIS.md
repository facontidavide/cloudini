# Cloudini Algorithm Analysis

## Overview

Cloudini implements a two-stage compression algorithm specifically designed for pointcloud data. The algorithm focuses on speed while achieving excellent compression ratios by exploiting the spatial coherence and data patterns typical in pointclouds.

## Two-Stage Compression Pipeline

### Stage 1: Custom Field Encoding
The first stage applies specialized encoding to different field types based on their characteristics:

### Stage 2: General-Purpose Compression
The encoded data from stage 1 is then compressed using standard algorithms (LZ4 or ZSTD).

## Field-Specific Encoding Strategies

### 1. Integer Fields (Delta + Varint Encoding)
**Applied to:** INT8, INT16, INT32, INT64, UINT8, UINT16, UINT32, UINT64

**Algorithm:**
- Computes delta between consecutive values: `delta = current_value - previous_value`
- Encodes delta using variable-length integer encoding (varint) with zigzag encoding
- Zigzag encoding maps signed integers to unsigned: `(value << 1) ^ (value >> 63)`
- Reserves value 0 for special cases (e.g., NaN markers)

**Code Example (from field_encoder.hpp:78-84):**
```cpp
size_t encode(const ConstBufferView& point_view, BufferView& output) override {
  int64_t value = ToInt64<IntType>(point_view.data() + offset_);
  int64_t diff = value - prev_value_;
  prev_value_ = value;
  int64_t var_size = encodeVarint64(diff, output.data());
  output.trim_front(var_size);
  return var_size;
}
```

### 2. Floating Point - Lossy Compression (Quantization + Delta + Varint)
**Applied to:** FLOAT32, FLOAT64 with resolution parameter

**Algorithm:**
- Quantizes float to integer: `quantized = round(float_value * (1/resolution))`
- Applies delta encoding: `delta = quantized - previous_quantized`
- Encodes delta using varint encoding
- Special handling for NaN values (encoded as single byte 0)

**Key Insight:** Quantization resolution controls precision vs compression tradeoff. For LiDAR data:
- 0.001m (1mm) resolution is typically sufficient for "raw" data
- 0.01m (1cm) resolution for visualization purposes

**Code Example (from field_encoder.hpp:168-182):**
```cpp
size_t encode(const ConstBufferView& point_view, BufferView& output) override {
  FloatType value_real = *(reinterpret_cast<const FloatType*>(point_view.data() + offset_));
  if (std::isnan(value_real)) {
    output.data()[0] = 0;  // value 0 is reserved for NaN
    prev_value_ = 0;
    output.trim_front(1);
    return 1;
  }
  const int64_t value = static_cast<int64_t>(std::round(value_real * multiplier_));
  const int64_t delta = value - prev_value_;
  prev_value_ = value;
  auto count = encodeVarint64(delta, output.data());
  output.trim_front(count);
  return count;
}
```

### 3. Floating Point - Lossless Compression (XOR Encoding)
**Applied to:** FLOAT32, FLOAT64 without resolution parameter

**Algorithm:**
- Treats float as bit pattern (uint32_t/uint64_t)
- Computes XOR with previous value: `residual = current_bits ^ previous_bits`
- Stores raw residual (fixed-size encoding)
- Exploits floating-point representation patterns for compression

**Code Example (from field_encoder.hpp:185-195):**
```cpp
size_t encode(const ConstBufferView& point_view, BufferView& output) override {
  IntType current_val_uint;
  memcpy(&current_val_uint, point_view.data() + offset_, sizeof(IntType));

  const IntType residual = current_val_uint ^ prev_bits_;
  prev_bits_ = current_val_uint;

  memcpy(output.data(), &residual, sizeof(IntType));
  output.trim_front(sizeof(IntType));
  return sizeof(IntType);
}
```

### 4. SIMD-Optimized Vector Encoding (FloatN_Lossy)
**Applied to:** 3-4 consecutive FLOAT32 fields (typically XYZ or XYZI coordinates)

**Algorithm:**
- Processes 4 floats simultaneously using SIMD instructions (SSE/AVX)
- Applies quantization using vector multiplication
- Converts to integers using SIMD rounding (`_mm_cvtps_epi32`)
- Computes vector delta and encodes each component with varint
- Optimized NaN detection using SIMD compare operations

**SIMD Optimizations:**
- Uses `__m128` (Vector4f) and `__m128i` (Vector4i) for vectorized operations
- NaN detection: `_mm_cmpneq_ps(vect, vect)` (NaN != NaN)
- Branch prediction hints with `__builtin_expect` for common no-NaN case

**Code Example (from field_encoder.cpp:42-91):**
```cpp
size_t encode(const ConstBufferView& point_view, BufferView& output) {
  const Vector4f vect_real(...); // Load 4 floats
  const Vector4f normalized_vect = vect_real * multiplier_;
  const Vector4i vect_int = cast_vector4f_to_vector4i(normalized_vect);
  const Vector4i delta = vect_int - prev_vect_;
  prev_vect_ = vect_int;

#if defined(ARCH_X86_SSE)
  // SIMD NaN detection
  const __m128 nan_mask = _mm_cmpneq_ps(vect_real.data.m, vect_real.data.m);
  const int nan_bits = _mm_movemask_ps(nan_mask);

  if (__builtin_expect(nan_bits == 0, 1)) {
    // Fast path: no NaNs
    ptr_out += encodeVarint64(delta[0], ptr_out);
    ptr_out += encodeVarint64(delta[1], ptr_out);
    // ... encode remaining components
  }
#endif
  // Fallback path handles NaNs
}
```

### 5. Raw Copy (No Encoding)
**Applied to:** Small integer types (INT8, UINT8) or when EncodingOptions::NONE is used

## Decoding Process

Decoding reverses the encoding process:

1. **Header Parsing:** Extract field metadata, compression settings
2. **Stage 2 Decompression:** LZ4/ZSTD decompression if used
3. **Stage 1 Decoding:** Field-specific decoding using appropriate decoders
4. **Point Reconstruction:** Assemble decoded fields into output pointcloud

## Key Design Decisions

### Compression Effectiveness
- **Delta encoding** exploits spatial coherence in pointclouds (nearby points have similar coordinates)
- **Varint encoding** efficiently handles small deltas (common in coherent data)
- **Quantization** removes precision beyond sensor accuracy (1-10mm for typical LiDAR)
- **SIMD processing** accelerates the most common case (XYZ coordinates)

### Performance Optimizations
- **Vectorized operations** using SSE4.1 intrinsics for x86_64
- **Branch prediction hints** for common cases (no NaNs, valid data)
- **Memory layout awareness** with aligned data structures
- **Single-pass processing** with streaming interface

### Robustness
- **Magic header** with version identifier ("CLOUDINI_V02")
- **Self-describing format** with embedded field metadata
- **NaN handling** with special encoding (value 0 reserved)
- **Error checking** for compression/decompression failures

## Data Format Structure

```
[MAGIC_HEADER] [WIDTH] [HEIGHT] [POINT_STEP] [ENCODING_OPT] [COMPRESSION_OPT]
[FIELD_COUNT] [FIELD_1_METADATA] ... [FIELD_N_METADATA] [COMPRESSED_DATA]
```

## Algorithm Characteristics

- **Compression Ratio:** Typically 40-85% size reduction vs raw data
- **Speed:** Faster than LZ4/ZSTD alone due to preprocessing effectiveness
- **Precision:** Configurable quantization with recommended 1mm resolution
- **Compatibility:** Works with standard pointcloud formats (PCL, ROS)
- **Scalability:** Processes millions of points efficiently

This algorithm represents a domain-specific compression approach that leverages pointcloud data characteristics for superior performance compared to general-purpose compression alone.
