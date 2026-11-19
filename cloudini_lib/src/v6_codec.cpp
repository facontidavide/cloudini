/*
 * Copyright 2025 Davide Faconti
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "v6_codec.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "cloudini_lib/encoding_utils.hpp"
#include "cloudini_lib/field_encoder.hpp"
#include "codec_common.hpp"
#include "v5_codec.hpp"

// For the per-value helpers of the hot loops.
#if defined(__GNUC__)
#define CLOUDINI_V6_INLINE inline __attribute__((always_inline))
#else
#define CLOUDINI_V6_INLINE inline
#endif

namespace Cloudini::detail {

//==========================================================================================
// V6: geometry predicted from the best neighbour, invalid-point mask, one stream per field.
//
// Stage 1 of a chunk of n points:
//   geometry section   u8 mode (V6GeometryMode):
//                        Raw: the x, y and z FLOAT32 columns, n values each.
//                        Predicted: u8 predictor (V6Predictor), uvarint lag K, u8 mask kind (V6MaskKind),
//                          [ceil(n / 8) validity bytes, LSB first, when there is a mask],
//                          uvarint size of the x stream, uvarint size of the y stream,
//                          then the x, y and z streams: one varint per valid point (encodeVarint64,
//                          0 = NaN), the residual of the quantized value against the prediction.
//   float columns      every other FLOAT32 field with a resolution: u8 mode, then the raw values or
//                        one residual stream with the Previous predictor.
//   regular columns    every other non-integer field: its V4 field encoder run over the points.
//   integer sections   the V5 adaptive sections.
//
// Quantized value q = round(v / resolution): in float precision (as V4) while |q| < 2^22, in double
// precision beyond. The decoder reconstructs float(double(q) * resolution).
namespace {

//------------------------------------------------------------------------------------------
// Format

enum class V6GeometryMode : uint8_t { Raw = 0, Predicted = 1 };
enum class V6Predictor : uint8_t { Previous = 0, LagK = 1, Median = 2, SecondOrder = 3 };
enum class V6MaskKind : uint8_t { None = 0, NaN = 1, Zero = 2 };

constexpr size_t kV6GeometryFields = 3;
constexpr size_t kV6MaxLag = 2100;
// The predictor of a chunk is chosen on its first kV6ProbePoints points.
constexpr size_t kV6ProbePoints = 4096;
// The lag is detected on the first points of the first chunk.
constexpr size_t kV6LagDetectPoints = 8192;
// Quantized values beyond this magnitude make the chunk store its geometry raw.
constexpr double kV6MaxQuantized = 1125899906842624.0;  // 2^50
// Values that quantize within this magnitude are quantized in float precision, as the V4 FloatN
// encoder does (error below one resolution step); larger ones in double precision.
constexpr float kV6FloatQuantized = 4194304.0f;  // 2^22
// Fields with a larger resolution (or a NaN one) are not V6-coded, so that no decoded value, even from a
// corrupted stream, leaves the float range.
constexpr float kV6MaxResolution = 1e18f;

bool isV6Resolution(const std::optional<float>& resolution) {
  return resolution && *resolution > 0.0f && *resolution < kV6MaxResolution;
}

// FLOAT32 fields with a resolution after x, y, z (e.g. intensity): coded like one geometry axis, with the
// Previous predictor, in a column of their own.
bool isV6FloatColumn(const EncodingInfo& info, size_t index) {
  const auto& field = info.fields[index];
  return index >= kV6GeometryFields && field.type == FieldType::FLOAT32 && isV6Resolution(field.resolution);
}

// The fields after x, y, z that are not V5 adaptive integer sections: float columns and V4-coded fields.
bool isV6RegularField(const EncodingInfo& info, size_t index) {
  return index >= kV6GeometryFields && !IsAdaptiveIntType(info.fields[index].type);
}

// Offsets and resolutions of x, y, z.
struct V6Geometry {
  std::array<uint32_t, 3> offset{};
  std::array<float, 3> resolution{};
  std::array<double, 3> inv_resolution{};
  std::array<float, 3> inv_resolution_f{};
  bool contiguous = false;  // x, y, z at consecutive offsets, 16 bytes readable from x within the point
};

V6Geometry makeV6Geometry(const EncodingInfo& info) {
  V6Geometry geometry;
  geometry.contiguous = info.fields[1].offset == info.fields[0].offset + 4 &&
                        info.fields[2].offset == info.fields[0].offset + 8 &&
                        info.fields[0].offset + 16 <= info.point_step;
  for (size_t axis = 0; axis < kV6GeometryFields; ++axis) {
    geometry.offset[axis] = info.fields[axis].offset;
    geometry.resolution[axis] = *info.fields[axis].resolution;
    geometry.inv_resolution[axis] = 1.0 / static_cast<double>(geometry.resolution[axis]);
    geometry.inv_resolution_f[axis] = 1.0f / geometry.resolution[axis];
  }
  return geometry;
}

float readF32(const uint8_t* ptr) {
  float value;
  std::memcpy(&value, ptr, sizeof(value));
  return value;
}

// Header of the geometry section (see the layout above). For a predicted chunk, `valid_bits` holds the
// validity bits when there is a mask, and `stream_bytes` the sizes of the x and y streams.
struct V6GeometryHeader {
  V6GeometryMode mode = V6GeometryMode::Raw;
  V6Predictor predictor = V6Predictor::Previous;
  size_t lag = 0;
  V6MaskKind mask = V6MaskKind::None;
  std::span<const uint8_t> valid_bits;
  std::array<size_t, 2> stream_bytes{};
};

void appendBytes(BufferView& out, const uint8_t* data, size_t size) {
  if (out.size() < size) {
    throw std::runtime_error("V6: output buffer full");
  }
  std::memcpy(out.data(), data, size);
  out.trim_front(size);
}

void appendV6GeometryHeader(const V6GeometryHeader& header, BufferView& out) {
  appendByte(out, static_cast<uint8_t>(header.mode));
  if (header.mode == V6GeometryMode::Raw) {
    return;
  }
  appendByte(out, static_cast<uint8_t>(header.predictor));
  appendUVarint(header.lag, out);
  appendByte(out, static_cast<uint8_t>(header.mask));
  if (header.mask != V6MaskKind::None) {
    appendBytes(out, header.valid_bits.data(), header.valid_bits.size());
  }
  appendUVarint(header.stream_bytes[0], out);
  appendUVarint(header.stream_bytes[1], out);
}

// Reads the header of the geometry section of a chunk of n points, and checks it.
V6GeometryHeader readV6GeometryHeader(ConstBufferView& input, size_t n) {
  V6GeometryHeader header;
  uint8_t byte = 0;
  decode(input, byte);
  if (byte > static_cast<uint8_t>(V6GeometryMode::Predicted)) {
    throw std::runtime_error("V6: unknown geometry mode");
  }
  header.mode = static_cast<V6GeometryMode>(byte);
  if (header.mode == V6GeometryMode::Raw) {
    return header;
  }
  decode(input, byte);
  if (byte > static_cast<uint8_t>(V6Predictor::SecondOrder)) {
    throw std::runtime_error("V6: unknown predictor");
  }
  header.predictor = static_cast<V6Predictor>(byte);
  // a lag beyond the chunk is the same as n + 1: no point has a neighbour that far back
  header.lag = static_cast<size_t>(std::min<uint64_t>(readUVarint(input), n + 1));
  decode(input, byte);
  if (byte > static_cast<uint8_t>(V6MaskKind::Zero)) {
    throw std::runtime_error("V6: unknown mask kind");
  }
  header.mask = static_cast<V6MaskKind>(byte);
  if (header.mask != V6MaskKind::None) {
    const size_t mask_bytes = (n + 7) / 8;
    if (input.size() < mask_bytes) {
      throw std::runtime_error("V6: truncated validity mask");
    }
    header.valid_bits = {input.data(), mask_bytes};
    input.trim_front(mask_bytes);
  }
  const uint64_t x_bytes = readUVarint(input);
  const uint64_t y_bytes = readUVarint(input);
  if (x_bytes > input.size() || y_bytes > input.size() - x_bytes) {
    throw std::runtime_error("V6: geometry stream sizes exceed the chunk");
  }
  header.stream_bytes = {static_cast<size_t>(x_bytes), static_cast<size_t>(y_bytes)};
  return header;
}

// n FLOAT32 values of a field, copied bit-exact.
void appendRawFloatColumn(BufferView& out, const uint8_t* points, size_t point_step, size_t offset, size_t n) {
  if (out.size() < n * sizeof(float)) {
    throw std::runtime_error("V6: output buffer full");
  }
  for (size_t i = 0; i < n; ++i) {
    std::memcpy(out.data() + i * sizeof(float), points + i * point_step + offset, sizeof(float));
  }
  out.trim_front(n * sizeof(float));
}

void readRawFloatColumn(ConstBufferView& input, uint8_t* points, size_t point_step, uint32_t offset, size_t n) {
  if (input.size() < n * sizeof(float)) {
    throw std::runtime_error("V6: truncated raw column");
  }
  if (offset != kDecodeButSkipStore) {
    for (size_t i = 0; i < n; ++i) {
      std::memcpy(points + i * point_step + offset, input.data() + i * sizeof(float), sizeof(float));
    }
  }
  input.trim_front(n * sizeof(float));
}

//------------------------------------------------------------------------------------------
// Prediction, shared by the encoder and the decoder

inline int64_t wrapV6(uint64_t value) {
  return static_cast<int64_t>(value);
}

// Prediction of value i of one axis from the values already reconstructed (`history`, same axis only). The
// arithmetic wraps around: values decoded from a corrupted stream can be anywhere in the int64 range (the
// encoder's values stay below 2^50, where wrapping never happens). Interior: i >= lag + 2 is guaranteed,
// so the boundary tests fold away.
template <V6Predictor P, bool Interior = false>
CLOUDINI_V6_INLINE int64_t v6Predict(const int64_t* history, size_t i, size_t lag) {
  const int64_t previous = (Interior || i >= 1) ? history[i - 1] : 0;
  if constexpr (P == V6Predictor::Previous) {
    return previous;
  } else if constexpr (P == V6Predictor::LagK) {
    return (Interior || i >= lag) ? history[i - lag] : previous;
  } else if constexpr (P == V6Predictor::Median) {
    if (!Interior && i <= lag) {
      return previous;
    }
    // LOCO-I median edge detector
    const int64_t left = previous, up = history[i - lag], up_left = history[i - lag - 1];
    const int64_t high = std::max(left, up), low = std::min(left, up);
    return up_left >= high ? low : (up_left <= low ? high : wrapV6(uint64_t(left) + uint64_t(up) - uint64_t(up_left)));
  } else {
    return (Interior || i >= 2) ? wrapV6(2 * uint64_t(previous) - uint64_t(history[i - 2])) : previous;
  }
}

// Calls f(std::integral_constant<V6Predictor, P>{}) with the predictor known at run time.
template <typename F>
decltype(auto) withV6Predictor(V6Predictor predictor, F&& f) {
  switch (predictor) {
    case V6Predictor::Previous:
      return f(std::integral_constant<V6Predictor, V6Predictor::Previous>{});
    case V6Predictor::LagK:
      return f(std::integral_constant<V6Predictor, V6Predictor::LagK>{});
    case V6Predictor::Median:
      return f(std::integral_constant<V6Predictor, V6Predictor::Median>{});
    default:
      return f(std::integral_constant<V6Predictor, V6Predictor::SecondOrder>{});
  }
}

// Value of a quantized step. While q is exact in a float, float(q) * resolution gives the same bits (the
// double product of two floats is exact, and both round it once), so the double product costs nothing more.
// Defined for every q: V6 resolutions are below kV6MaxResolution, so |q * res| < 2^63 * 1e18 < FLT_MAX.
inline float v6Reconstruct(int64_t q, double resolution) {
  return static_cast<float>(static_cast<double>(q) * resolution);
}

//------------------------------------------------------------------------------------------
// Encoder: quantization

// x, y, z of one point quantized in float precision (the same values as quantizeV6Chunk). Bits 0-2 of
// `nan` / `zero` flag the NaN / 0.0f axes.
struct V6PointQuantized {
  alignas(16) int32_t q[4];
  int nan;
  int zero;
};

#if defined(__SSE4_1__)
// x, y, z of a point in lanes 0-2, 0 in lane 3.
CLOUDINI_V6_INLINE __m128 loadV6Xyz(const V6Geometry& geometry, const uint8_t* point) {
  if (geometry.contiguous) {
    const __m128 xyzw = _mm_loadu_ps(reinterpret_cast<const float*>(point + geometry.offset[0]));
    return _mm_blend_ps(xyzw, _mm_setzero_ps(), 0x8);
  }
  return _mm_setr_ps(
      readF32(point + geometry.offset[0]), readF32(point + geometry.offset[1]), readF32(point + geometry.offset[2]),
      0.0f);
}
#endif

// false when a value is infinite or quantizes beyond kV6FloatQuantized.
CLOUDINI_V6_INLINE bool quantizeV6Point(const V6Geometry& geometry, const uint8_t* point, V6PointQuantized& out) {
#if defined(__SSE4_1__)
  const __m128 v = loadV6Xyz(geometry, point);
  const __m128 inv =
      _mm_setr_ps(geometry.inv_resolution_f[0], geometry.inv_resolution_f[1], geometry.inv_resolution_f[2], 0.0f);
  const __m128 scaled = _mm_round_ps(_mm_mul_ps(v, inv), _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC);
  const __m128 magnitude = _mm_andnot_ps(_mm_set1_ps(-0.0f), scaled);
  if (_mm_movemask_ps(_mm_cmpge_ps(magnitude, _mm_set1_ps(kV6FloatQuantized))) != 0) {
    return false;  // NaN lanes compare false
  }
  const __m128 is_nan = _mm_cmpunord_ps(v, v);
  out.nan = _mm_movemask_ps(is_nan) & 7;
  out.zero = _mm_movemask_ps(_mm_cmpeq_ps(v, _mm_setzero_ps())) & 7;
  _mm_store_si128(reinterpret_cast<__m128i*>(out.q), _mm_cvtps_epi32(_mm_andnot_ps(is_nan, scaled)));
  return true;
#else
  out.nan = 0;
  out.zero = 0;
  for (size_t axis = 0; axis < 3; ++axis) {
    const float v = readF32(point + geometry.offset[axis]);
    const float scaled = std::nearbyint(v * geometry.inv_resolution_f[axis]);
    if (std::isnan(v)) {
      out.nan |= 1 << axis;
      out.q[axis] = 0;
      continue;
    }
    if (!(std::fabs(scaled) < kV6FloatQuantized)) {
      return false;
    }
    out.zero |= (v == 0.0f) << axis;
    out.q[axis] = static_cast<int32_t>(scaled);
  }
  return true;
#endif
}

// Mask kind of a chunk: the more frequent of all-NaN and all-zero points, None when there are neither.
V6MaskKind scanV6MaskKind(const V6Geometry& geometry, const uint8_t* points, size_t point_step, size_t n) {
  size_t nan_points = 0, zero_points = 0;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t* point = points + i * point_step;
#if defined(__SSE4_1__)
    const __m128 v = loadV6Xyz(geometry, point);
    nan_points += (_mm_movemask_ps(_mm_cmpunord_ps(v, v)) & 7) == 7;
    zero_points += (_mm_movemask_ps(_mm_cmpeq_ps(v, _mm_setzero_ps())) & 7) == 7;
#else
    int nan_axes = 0, zero_axes = 0;
    for (size_t axis = 0; axis < 3; ++axis) {
      const float v = readF32(point + geometry.offset[axis]);
      nan_axes += std::isnan(v);
      zero_axes += (v == 0.0f);
    }
    nan_points += (nan_axes == 3);
    zero_points += (zero_axes == 3);
#endif
  }
  if (nan_points == 0 && zero_points == 0) {
    return V6MaskKind::None;
  }
  return nan_points >= zero_points ? V6MaskKind::NaN : V6MaskKind::Zero;
}

// Quantized value of v, 0 for NaN: in float precision while it stays below kV6FloatQuantized, in double
// precision beyond.
inline double quantizeV6Value(float v, float inv_resolution_f, double inv_resolution) {
  if (std::isnan(v)) {
    return 0.0;
  }
  const float scaled_f = std::nearbyint(v * inv_resolution_f);
  return std::fabs(scaled_f) < kV6FloatQuantized ? static_cast<double>(scaled_f)
                                                 : std::nearbyint(static_cast<double>(v) * inv_resolution);
}

// Quantized geometry of the first points of a chunk, axis-major (value i of axis a at a * points + i).
struct V6ChunkGeometry {
  size_t points = 0;
  std::vector<int64_t> quantized;  // NaN axes as 0
  std::vector<uint8_t> nan;
  std::vector<uint8_t> valid;  // 1 per point: coded in the streams
  V6MaskKind mask = V6MaskKind::None;
  bool raw = false;  // a value the integer streams cannot hold: the chunk stores its geometry raw
};

// Quantizes the first n points of a chunk. The mask kind is the one of these n points, unless `mask` gives
// the kind of the whole chunk (scanV6MaskKind), when only a prefix of the chunk is quantized for probing.
// Stops at the first value the integer streams cannot hold (infinite or too large): the chunk is raw.
void quantizeV6Chunk(
    const V6Geometry& geometry, const uint8_t* points, size_t point_step, size_t n, V6ChunkGeometry& out,
    std::optional<V6MaskKind> mask = std::nullopt) {
  out.points = n;
  out.quantized.resize(n * 3);
  out.nan.resize(n * 3);
  out.valid.assign(n, 1);
  out.raw = false;
  out.mask = V6MaskKind::None;
  size_t nan_points = 0, zero_points = 0;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t* point = points + i * point_step;
    V6PointQuantized quantized;
    if (quantizeV6Point(geometry, point, quantized)) {  // the same values as the per-axis code below
      for (size_t axis = 0; axis < 3; ++axis) {
        out.nan[axis * n + i] = (quantized.nan >> axis) & 1;
        out.quantized[axis * n + i] = quantized.q[axis];
      }
      nan_points += (quantized.nan == 7);
      zero_points += (quantized.zero == 7);
      continue;
    }
    int nan_axes = 0, zero_axes = 0;
    for (size_t axis = 0; axis < 3; ++axis) {
      const float v = readF32(point + geometry.offset[axis]);
      const bool is_nan = std::isnan(v);
      out.nan[axis * n + i] = is_nan;
      nan_axes += is_nan;
      zero_axes += (v == 0.0f);
      const double scaled = quantizeV6Value(v, geometry.inv_resolution_f[axis], geometry.inv_resolution[axis]);
      if (!(std::fabs(scaled) < kV6MaxQuantized)) {
        out.raw = true;
        return;
      }
      out.quantized[axis * n + i] = static_cast<int64_t>(scaled);
    }
    nan_points += (nan_axes == 3);
    zero_points += (zero_axes == 3);
  }
  if (mask) {
    out.mask = *mask;
  } else if (nan_points > 0 || zero_points > 0) {
    out.mask = nan_points >= zero_points ? V6MaskKind::NaN : V6MaskKind::Zero;
  }
  if (out.mask == V6MaskKind::None) {
    return;
  }
  for (size_t i = 0; i < n; ++i) {
    bool invalid = true;
    for (size_t axis = 0; axis < 3; ++axis) {
      const bool is_nan = out.nan[axis * n + i];
      invalid &= out.mask == V6MaskKind::NaN ? is_nan
                                             : (!is_nan && out.quantized[axis * n + i] == 0 &&
                                                readF32(points + i * point_step + geometry.offset[axis]) == 0.0f);
    }
    out.valid[i] = !invalid;
  }
}

//------------------------------------------------------------------------------------------
// Encoder: residual streams

// Scratch array that grows without zero-filling: its contents are always written before they are read,
// and are not kept when it grows.
template <typename T>
class ScratchArray {
 public:
  T* data() {
    return data_.get();
  }
  void ensure(size_t count) {
    if (count > size_) {
      data_ = std::make_unique_for_overwrite<T[]>(count);
      size_ = count;
    }
  }

 private:
  std::unique_ptr<T[]> data_;
  size_t size_ = 0;
};

// The three residual streams of a chunk, and the values the decoder will reconstruct (`history`, what the
// predictors read), axis-major.
struct V6Streams {
  std::array<ScratchArray<uint8_t>, 3> data;
  std::array<size_t, 3> size{0, 0, 0};
  ScratchArray<int64_t> history;
  // block coder (encodeV6GeometryBlocks): quantized values, axis-major, and the validity of each point
  ScratchArray<int32_t> q32;
  ScratchArray<uint8_t> valid8;
};

// Buffers for `count` points (grow only).
void reserveV6Streams(V6Streams& streams, size_t count) {
  streams.history.ensure(3 * count);
  for (size_t axis = 0; axis < 3; ++axis) {
    streams.data[axis].ensure(count * kMaxVarintBytes);
  }
}

// Residual stream of one axis for predictor P, written at `out`; returns its size. `history` receives the
// values the decoder reconstructs: an invalid point repeats the previous value, a NaN the prediction.
template <V6Predictor P>
size_t encodeV6Axis(
    const int64_t* quantized, const uint8_t* nan, const uint8_t* valid, size_t n, size_t lag, int64_t* history,
    uint8_t* out) {
  uint8_t* cursor = out;
  for (size_t i = 0; i < n; ++i) {
    if (valid && !valid[i]) {
      history[i] = i >= 1 ? history[i - 1] : 0;
      continue;
    }
    const int64_t prediction = v6Predict<P>(history, i, lag);
    if (nan[i]) {
      history[i] = prediction;
      *cursor++ = 0;  // NaN marker
      continue;
    }
    history[i] = quantized[i];
    cursor += encodeVarint64(quantized[i] - prediction, cursor);
  }
  return static_cast<size_t>(cursor - out);
}

#if defined(__SSE4_1__)
//------------------------------------------------------------------------------------------
// Block coder: the streams of encodeV6GeometryFused in phases over the whole chunk, with 4 values per step.
//
//   1. quantize 4 points at a time into int32 columns (float path only: |q| < 2^22),
//   2. fill invalid points with the previous value (the history the decoder keeps),
//   3. predict 4 values at a time from the filled columns,
//   4. write 4 varints at a time with one byte shuffle.
//
// The residuals stay below 2^25 (predictions are combinations of at most three values below 2^22), so every
// varint has at most 4 bytes. Points with some but not all axes NaN make the history depend on the
// prediction: such chunks use the per-point coder.

// Byte shuffle that packs 4 varints of 0 to 4 bytes (lane k at bytes 4k to 4k + 3) to the front, by
// l0 + 5 l1 + 25 l2 + 125 l3, and the packed length.
struct V6VarintPack {
  alignas(16) uint8_t shuffle[16];
  uint8_t total;
};
const std::array<V6VarintPack, 625>& v6VarintPackTable() {
  static const std::array<V6VarintPack, 625> table = [] {
    std::array<V6VarintPack, 625> t{};
    for (int index = 0; index < 625; ++index) {
      int rest = index, pos = 0;
      std::memset(t[index].shuffle, 0x80, 16);
      for (int lane = 0; lane < 4; ++lane) {
        const int len = rest % 5;
        rest /= 5;
        for (int b = 0; b < len; ++b) {
          t[index].shuffle[pos++] = static_cast<uint8_t>(lane * 4 + b);
        }
      }
      t[index].total = static_cast<uint8_t>(pos);
    }
    return t;
  }();
  return table;
}

// Writes the varints of 4 residuals (encodeVarint64 of r: zigzag + 1), or a NaN marker byte 0 where `marker`
// is set, or nothing where `skip` is set. Needs 16 writable bytes at `out`.
CLOUDINI_V6_INLINE uint8_t* writeV6Varints4(
    const V6VarintPack* table, __m128i residual, __m128i marker, __m128i skip, uint8_t* out) {
  const __m128i zigzag = _mm_xor_si128(_mm_slli_epi32(residual, 1), _mm_srai_epi32(residual, 31));
  const __m128i code = _mm_andnot_si128(marker, _mm_add_epi32(zigzag, _mm_set1_epi32(1)));
  // 7-bit groups into bytes
  const __m128i bytes = _mm_or_si128(
      _mm_or_si128(
          _mm_and_si128(code, _mm_set1_epi32(0x7F)), _mm_and_si128(_mm_slli_epi32(code, 1), _mm_set1_epi32(0x7F00))),
      _mm_or_si128(
          _mm_and_si128(_mm_slli_epi32(code, 2), _mm_set1_epi32(0x7F0000)),
          _mm_and_si128(_mm_slli_epi32(code, 3), _mm_set1_epi32(0x7F000000))));
  const __m128i ge2 = _mm_cmpgt_epi32(code, _mm_set1_epi32(0x7F));
  const __m128i ge3 = _mm_cmpgt_epi32(code, _mm_set1_epi32(0x3FFF));
  const __m128i ge4 = _mm_cmpgt_epi32(code, _mm_set1_epi32(0x1FFFFF));
  const __m128i continuation = _mm_or_si128(
      _mm_and_si128(ge2, _mm_set1_epi32(0x80)),
      _mm_or_si128(_mm_and_si128(ge3, _mm_set1_epi32(0x8000)), _mm_and_si128(ge4, _mm_set1_epi32(0x800000))));
  // length 1 to 4 (compare masks are -1), 0 where skipped
  __m128i len = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_set1_epi32(1), ge2), ge3), ge4);
  len = _mm_andnot_si128(skip, len);
  const __m128i weighted = _mm_mullo_epi32(len, _mm_setr_epi32(1, 5, 25, 125));
  const __m128i sum2 = _mm_add_epi32(weighted, _mm_shuffle_epi32(weighted, _MM_SHUFFLE(1, 0, 3, 2)));
  const int index = _mm_cvtsi128_si32(_mm_add_epi32(sum2, _mm_shuffle_epi32(sum2, _MM_SHUFFLE(2, 3, 0, 1))));
  const V6VarintPack& pack = table[static_cast<size_t>(index)];
  const __m128i packed = _mm_shuffle_epi8(
      _mm_or_si128(bytes, continuation), _mm_load_si128(reinterpret_cast<const __m128i*>(pack.shuffle)));
  _mm_storeu_si128(reinterpret_cast<__m128i*>(out), packed);
  return out + pack.total;
}

// Prediction of 4 values from the filled column `h` at i to i + 3 (interior points only).
template <V6Predictor P>
CLOUDINI_V6_INLINE __m128i predictV6x4(const int32_t* h, size_t i, size_t lag) {
  const __m128i left = _mm_loadu_si128(reinterpret_cast<const __m128i*>(h + i - 1));
  if constexpr (P == V6Predictor::Previous) {
    return left;
  } else if constexpr (P == V6Predictor::LagK) {
    return _mm_loadu_si128(reinterpret_cast<const __m128i*>(h + i - lag));
  } else if constexpr (P == V6Predictor::Median) {
    const __m128i up = _mm_loadu_si128(reinterpret_cast<const __m128i*>(h + i - lag));
    const __m128i up_left = _mm_loadu_si128(reinterpret_cast<const __m128i*>(h + i - lag - 1));
    const __m128i high = _mm_max_epi32(left, up), low = _mm_min_epi32(left, up);
    const __m128i gradient = _mm_sub_epi32(_mm_add_epi32(left, up), up_left);
    const __m128i ul_ge_high = _mm_xor_si128(_mm_cmpgt_epi32(high, up_left), _mm_set1_epi32(-1));
    const __m128i ul_le_low = _mm_xor_si128(_mm_cmpgt_epi32(up_left, low), _mm_set1_epi32(-1));
    const __m128i t = _mm_blendv_epi8(gradient, high, ul_le_low);
    return _mm_blendv_epi8(t, low, ul_ge_high);
  } else {
    const __m128i left2 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(h + i - 2));
    return _mm_sub_epi32(_mm_slli_epi32(left, 1), left2);
  }
}

// Scalar prediction from the filled column, the same as v6Predict on the history.
template <V6Predictor P>
CLOUDINI_V6_INLINE int32_t predictV6x1(const int32_t* h, size_t i, size_t lag) {
  const int32_t previous = i >= 1 ? h[i - 1] : 0;
  if constexpr (P == V6Predictor::Previous) {
    return previous;
  } else if constexpr (P == V6Predictor::LagK) {
    return i >= lag ? h[i - lag] : previous;
  } else if constexpr (P == V6Predictor::Median) {
    if (i <= lag) {
      return previous;
    }
    const int32_t left = previous, up = h[i - lag], up_left = h[i - lag - 1];
    const int32_t high = std::max(left, up), low = std::min(left, up);
    return up_left >= high ? low : (up_left <= low ? high : left + up - up_left);
  } else {
    return i >= 2 ? 2 * previous - h[i - 2] : previous;
  }
}

// The residual stream of one filled column; points with valid[i] == 0 are skipped (valid may be null).
template <V6Predictor P>
uint8_t* encodeV6ColumnBlocks(const int32_t* h, const uint8_t* valid, size_t n, size_t lag, uint8_t* out) {
  size_t first_interior = 1;
  if constexpr (P == V6Predictor::LagK) {
    first_interior = std::max<size_t>(lag, 1);
  } else if constexpr (P == V6Predictor::Median) {
    first_interior = lag + 1;
  } else if constexpr (P == V6Predictor::SecondOrder) {
    first_interior = 2;
  }
  first_interior = std::min(first_interior, n);
  size_t i = 0;
  for (; i < first_interior; ++i) {
    if (valid && !valid[i]) {
      continue;
    }
    out += encodeVarint64(int64_t{h[i]} - predictV6x1<P>(h, i, lag), out);
  }
  const __m128i none = _mm_setzero_si128();
  const V6VarintPack* table = v6VarintPackTable().data();
  for (; i + 4 <= n; i += 4) {
    const __m128i value = _mm_loadu_si128(reinterpret_cast<const __m128i*>(h + i));
    const __m128i residual = _mm_sub_epi32(value, predictV6x4<P>(h, i, lag));
    __m128i skip = none;
    if (valid) {
      int32_t v4;
      std::memcpy(&v4, valid + i, 4);
      if (v4 == 0) {
        continue;  // 4 invalid points: nothing to write
      }
      skip = _mm_cmpeq_epi32(_mm_cvtepu8_epi32(_mm_cvtsi32_si128(v4)), none);
    }
    out = writeV6Varints4(table, residual, none, skip, out);
  }
  for (; i < n; ++i) {
    if (valid && !valid[i]) {
      continue;
    }
    out += encodeVarint64(int64_t{h[i]} - predictV6x1<P>(h, i, lag), out);
  }
  return out;
}

// x, y, z of points i to i + 3 as three vectors.
CLOUDINI_V6_INLINE void loadV6Xyz4(
    const V6Geometry& geometry, const uint8_t* points, size_t point_step, size_t i, __m128& x, __m128& y, __m128& z) {
  const uint8_t* p = points + i * point_step;
  if (geometry.contiguous) {
    __m128 r0 = _mm_loadu_ps(reinterpret_cast<const float*>(p + geometry.offset[0]));
    __m128 r1 = _mm_loadu_ps(reinterpret_cast<const float*>(p + point_step + geometry.offset[0]));
    __m128 r2 = _mm_loadu_ps(reinterpret_cast<const float*>(p + 2 * point_step + geometry.offset[0]));
    __m128 r3 = _mm_loadu_ps(reinterpret_cast<const float*>(p + 3 * point_step + geometry.offset[0]));
    _MM_TRANSPOSE4_PS(r0, r1, r2, r3);
    x = r0;
    y = r1;
    z = r2;
    return;
  }
  auto column = [&](size_t axis) {
    const size_t o = geometry.offset[axis];
    return _mm_setr_ps(
        readF32(p + o), readF32(p + point_step + o), readF32(p + 2 * point_step + o), readF32(p + 3 * point_step + o));
  };
  x = column(0);
  y = column(1);
  z = column(2);
}

enum class V6BlockResult { Coded, NeedsDouble, PartialNaN };

// The streams of encodeV6GeometryFused<P, M>, or NeedsDouble / PartialNaN when the caller must use another
// coder. Sets the validity bits like encodeV6GeometryFused.
template <V6Predictor P, V6MaskKind M>
V6BlockResult encodeV6GeometryBlocks(
    const V6Geometry& geometry, const uint8_t* points, size_t point_step, size_t n, size_t lag, V6Streams& streams,
    uint8_t* valid_bits) {
  streams.q32.ensure(3 * n + 4);
  streams.valid8.ensure(n + 4);
  int32_t* q[3] = {streams.q32.data(), streams.q32.data() + n, streams.q32.data() + 2 * n};
  uint8_t* valid = streams.valid8.data();
  const __m128 inv[3] = {
      _mm_set1_ps(geometry.inv_resolution_f[0]), _mm_set1_ps(geometry.inv_resolution_f[1]),
      _mm_set1_ps(geometry.inv_resolution_f[2])};
  const __m128 sign = _mm_set1_ps(-0.0f), limit = _mm_set1_ps(kV6FloatQuantized);
  // 4 validity bits to 4 bytes of 0 / 1
  static constexpr std::array<uint32_t, 16> kNibbleBytes = [] {
    std::array<uint32_t, 16> t{};
    for (uint32_t bits = 0; bits < 16; ++bits) {
      for (uint32_t k = 0; k < 4; ++k) {
        t[bits] |= ((bits >> k) & 1u) << (8 * k);
      }
    }
    return t;
  }();
  bool any_invalid = false;
  int32_t previous[3] = {0, 0, 0};  // last valid value of each axis (the history of invalid points)

  // 1. quantize 4 points; 2. invalid points repeat the previous value. `count` < 4 only for the last group.
  auto quantize4 = [&](size_t i, const __m128(&v)[3], int count) -> V6BlockResult {
    __m128i qv[3];
    int nan_bits[3], zero_bits[3];
    for (int axis = 0; axis < 3; ++axis) {
      const __m128 scaled = _mm_round_ps(_mm_mul_ps(v[axis], inv[axis]), _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC);
      if (_mm_movemask_ps(_mm_cmpge_ps(_mm_andnot_ps(sign, scaled), limit)) != 0) {
        return V6BlockResult::NeedsDouble;  // NaN lanes compare false
      }
      const __m128 is_nan = _mm_cmpunord_ps(v[axis], v[axis]);
      nan_bits[axis] = _mm_movemask_ps(is_nan);
      if constexpr (M == V6MaskKind::Zero) {
        zero_bits[axis] = _mm_movemask_ps(_mm_cmpeq_ps(v[axis], _mm_setzero_ps()));
      }
      qv[axis] = _mm_cvtps_epi32(_mm_andnot_ps(is_nan, scaled));
    }
    const int lanes = (1 << count) - 1;
    const int all_nan = nan_bits[0] & nan_bits[1] & nan_bits[2];
    const int any_nan = nan_bits[0] | nan_bits[1] | nan_bits[2];
    int invalid = 0, partial = 0;
    if constexpr (M == V6MaskKind::None) {
      partial = any_nan;
    } else if constexpr (M == V6MaskKind::NaN) {
      invalid = all_nan;
      partial = any_nan & ~all_nan;
    } else {
      invalid = zero_bits[0] & zero_bits[1] & zero_bits[2];
      partial = any_nan;
    }
    invalid &= lanes;
    if ((partial & lanes) != 0) {
      return V6BlockResult::PartialNaN;
    }
    if constexpr (M != V6MaskKind::None) {
      const int valid_nibble = ~invalid & lanes;
      valid_bits[i / 8] |= static_cast<uint8_t>(valid_nibble << (i % 8));  // i is a multiple of 4
      const uint32_t bytes = kNibbleBytes[static_cast<size_t>(valid_nibble)];
      std::memcpy(valid + i, &bytes, 4);
    }
    for (int axis = 0; axis < 3; ++axis) {
      int32_t lane[4];
      _mm_storeu_si128(reinterpret_cast<__m128i*>(lane), qv[axis]);
      if (invalid == lanes) {  // all invalid: repeat the previous value
        any_invalid = true;
        for (int k = 0; k < 4; ++k) {
          lane[k] = previous[axis];
        }
      } else if (invalid != 0) {
        any_invalid = true;
        for (int k = 0; k < count; ++k) {
          lane[k] = ((invalid >> k) & 1) ? previous[axis] : lane[k];
          previous[axis] = lane[k];
        }
      } else {
        previous[axis] = lane[count - 1];
      }
      if (count == 4) {
        std::memcpy(q[axis] + i, lane, 4 * sizeof(int32_t));
      } else {
        for (int k = 0; k < count; ++k) {
          q[axis][i + static_cast<size_t>(k)] = lane[k];
        }
      }
    }
    return V6BlockResult::Coded;
  };
  size_t i = 0;
  for (; i + 4 <= n; i += 4) {
    __m128 v[3];
    loadV6Xyz4(geometry, points, point_step, i, v[0], v[1], v[2]);
    const V6BlockResult result = quantize4(i, v, 4);
    if (result != V6BlockResult::Coded) {
      return result;
    }
  }
  if (i < n) {
    float lane[3][4] = {};
    for (size_t k = 0; i + k < n; ++k) {
      for (size_t axis = 0; axis < 3; ++axis) {
        lane[axis][k] = readF32(points + (i + k) * point_step + geometry.offset[axis]);
      }
    }
    const __m128 v[3] = {_mm_loadu_ps(lane[0]), _mm_loadu_ps(lane[1]), _mm_loadu_ps(lane[2])};
    const V6BlockResult result = quantize4(i, v, static_cast<int>(n - i));
    if (result != V6BlockResult::Coded) {
      return result;
    }
  }

  // 3. and 4. predict and write
  for (size_t axis = 0; axis < 3; ++axis) {
    uint8_t* begin = streams.data[axis].data();
    uint8_t* end = encodeV6ColumnBlocks<P>(q[axis], any_invalid ? valid : nullptr, n, lag, begin);
    streams.size[axis] = static_cast<size_t>(end - begin);
  }
  return V6BlockResult::Coded;
}
#endif

// Quantizes and codes the chunk in one pass, for a predictor and a mask kind already known. With a mask,
// sets the validity bit of each valid point in `valid_bits` (zeroed by the caller). Returns false when a
// value needs the double-precision path (quantizeV6Chunk). Same streams as encodeV6Axis on each axis.
template <V6Predictor P, V6MaskKind M>
bool encodeV6GeometryFused(
    const V6Geometry& geometry, const uint8_t* points, size_t point_step, size_t n, size_t lag, V6Streams& streams,
    uint8_t* valid_bits) {
  int64_t* history[3] = {streams.history.data(), streams.history.data() + n, streams.history.data() + 2 * n};
  uint8_t* cursor[3] = {streams.data[0].data(), streams.data[1].data(), streams.data[2].data()};
  V6PointQuantized point;
  for (size_t i = 0; i < n; ++i) {
    if (!quantizeV6Point(geometry, points + i * point_step, point)) {
      return false;
    }
    if constexpr (M != V6MaskKind::None) {
      if ((M == V6MaskKind::NaN ? point.nan : point.zero) == 7) {  // invalid point: repeats the previous value
        for (size_t axis = 0; axis < 3; ++axis) {
          history[axis][i] = i >= 1 ? history[axis][i - 1] : 0;
        }
        continue;
      }
      valid_bits[i / 8] |= static_cast<uint8_t>(1u << (i % 8));
    }
    for (size_t axis = 0; axis < 3; ++axis) {
      const int64_t prediction = v6Predict<P>(history[axis], i, lag);
      if (point.nan & (1 << axis)) {
        history[axis][i] = prediction;
        *cursor[axis]++ = 0;  // NaN marker
      } else {
        history[axis][i] = point.q[axis];
        cursor[axis] += encodeVarint64(point.q[axis] - prediction, cursor[axis]);
      }
    }
  }
  for (size_t axis = 0; axis < 3; ++axis) {
    streams.size[axis] = static_cast<size_t>(cursor[axis] - streams.data[axis].data());
  }
  return true;
}

// One-pass coding of a chunk (see encodeV6GeometryFused).
bool buildV6StreamsFused(
    const V6Geometry& geometry, const uint8_t* points, size_t point_step, size_t n, V6Predictor predictor, size_t lag,
    V6MaskKind mask, V6Streams& streams, uint8_t* valid_bits) {
  reserveV6Streams(streams, n);
  return withV6Predictor(predictor, [&](auto predictor_tag) {
    constexpr V6Predictor P = decltype(predictor_tag)::value;
#if defined(__SSE4_1__)
    {
      V6BlockResult result = V6BlockResult::PartialNaN;
      switch (mask) {
        case V6MaskKind::None:
          result =
              encodeV6GeometryBlocks<P, V6MaskKind::None>(geometry, points, point_step, n, lag, streams, valid_bits);
          break;
        case V6MaskKind::NaN:
          result =
              encodeV6GeometryBlocks<P, V6MaskKind::NaN>(geometry, points, point_step, n, lag, streams, valid_bits);
          break;
        default:
          result =
              encodeV6GeometryBlocks<P, V6MaskKind::Zero>(geometry, points, point_step, n, lag, streams, valid_bits);
          break;
      }
      if (result == V6BlockResult::Coded) {
        return true;
      }
      if (result == V6BlockResult::NeedsDouble) {
        return false;
      }
      if (mask != V6MaskKind::None) {  // partly NaN points: the per-point coder sets the bits again
        std::memset(valid_bits, 0, (n + 7) / 8);
      }
    }
#endif
    switch (mask) {
      case V6MaskKind::None:
        return encodeV6GeometryFused<P, V6MaskKind::None>(geometry, points, point_step, n, lag, streams, valid_bits);
      case V6MaskKind::NaN:
        return encodeV6GeometryFused<P, V6MaskKind::NaN>(geometry, points, point_step, n, lag, streams, valid_bits);
      default:
        return encodeV6GeometryFused<P, V6MaskKind::Zero>(geometry, points, point_step, n, lag, streams, valid_bits);
    }
  });
}

// Two-pass coding: the residual streams of the first `count` points of a quantized chunk, one axis at a time.
void buildV6Streams(const V6ChunkGeometry& chunk, size_t count, V6Predictor predictor, size_t lag, V6Streams& streams) {
  reserveV6Streams(streams, count);
  const uint8_t* valid = chunk.mask == V6MaskKind::None ? nullptr : chunk.valid.data();
  withV6Predictor(predictor, [&](auto predictor_tag) {
    for (size_t axis = 0; axis < 3; ++axis) {
      streams.size[axis] = encodeV6Axis<decltype(predictor_tag)::value>(
          chunk.quantized.data() + axis * chunk.points, chunk.nan.data() + axis * chunk.points, valid, count, lag,
          streams.history.data() + axis * count, streams.data[axis].data());
    }
  });
}

//------------------------------------------------------------------------------------------
// Encoder: choosing the lag and the predictor

// Lag between a point and "the same laser one firing earlier": the lag with the smallest mean distance
// between points i and i - lag, on a sample of the first points of the chunk.
size_t detectV6Lag(const V6ChunkGeometry& chunk) {
  const size_t n = std::min<size_t>(chunk.points, kV6LagDetectPoints);
  if (n < 64) {
    return 0;
  }
  // forward-filled copy of the first n points, point-major for the lag scan
  std::vector<int64_t> filled(n * 3, 0);
  for (size_t i = 0; i < n; ++i) {
    for (size_t axis = 0; axis < 3; ++axis) {
      const bool keep = chunk.valid[i] && !chunk.nan[axis * chunk.points + i];
      filled[i * 3 + axis] =
          keep ? chunk.quantized[axis * chunk.points + i] : (i >= 1 ? filled[(i - 1) * 3 + axis] : 0);
    }
  }
  const size_t max_lag = std::min(kV6MaxLag, n / 2);
  constexpr size_t kSamples = 64;
  const size_t span = n - max_lag;
  size_t best_lag = 0;
  int64_t best_cost = 0;
  for (size_t lag = 2; lag <= max_lag; ++lag) {
    int64_t cost = 0;
    for (size_t sample = 0; sample < kSamples; ++sample) {
      const size_t i = max_lag + (sample * span) / kSamples;
      for (size_t axis = 0; axis < 3; ++axis) {
        cost += std::llabs(filled[i * 3 + axis] - filled[(i - lag) * 3 + axis]);
      }
      if (best_lag != 0 && cost >= best_cost) {
        break;  // a lag wins only with a smaller cost
      }
    }
    if (best_lag == 0 || cost < best_cost) {
      best_lag = lag;
      best_cost = cost;
    }
  }
  return best_lag;
}

// Varint length of a value by its count of leading zero bits: ceil((64 - clz) / 7).
constexpr std::array<uint8_t, 64> kV6VarintBytes = [] {
  std::array<uint8_t, 64> table{};
  for (int clz = 0; clz < 64; ++clz) {
    table[clz] = static_cast<uint8_t>((64 - clz + 6) / 7);
  }
  return table;
}();

// Size of the residual stream of one axis for predictor P (as encodeV6Axis), without writing it. Stops
// early, returning at least `budget`, once the size reaches `budget`.
template <V6Predictor P>
size_t estimateV6Axis(
    const int64_t* quantized, const uint8_t* nan, const uint8_t* valid, size_t n, size_t lag, int64_t* history,
    size_t budget) {
  size_t bytes = 0;
  for (size_t i = 0; i < n; ++i) {
    if ((i & 511) == 0 && bytes >= budget) {
      return bytes;
    }
    if (valid && !valid[i]) {
      history[i] = i >= 1 ? history[i - 1] : 0;
      continue;
    }
    const int64_t prediction = v6Predict<P>(history, i, lag);
    if (nan[i]) {
      history[i] = prediction;
      ++bytes;
      continue;
    }
    history[i] = quantized[i];
    const int64_t residual = quantized[i] - prediction;
    const uint64_t zigzag = (static_cast<uint64_t>(residual) << 1) ^ static_cast<uint64_t>(residual >> 63);
    bytes += kV6VarintBytes[std::countl_zero(zigzag + 1)];  // encodeVarint64 codes zigzag + 1
  }
  return bytes;
}

// Stage-1 size of the first `count` points for a predictor; stops early once it reaches `budget` (the
// result is then at least `budget`).
size_t estimateV6Streams(
    const V6ChunkGeometry& chunk, size_t count, V6Predictor predictor, size_t lag, V6Streams& streams, size_t budget) {
  streams.history.ensure(count);
  const uint8_t* valid = chunk.mask == V6MaskKind::None ? nullptr : chunk.valid.data();
  size_t bytes = 0;
  for (size_t axis = 0; axis < 3 && bytes < budget; ++axis) {
    const int64_t* quantized = chunk.quantized.data() + axis * chunk.points;
    const uint8_t* nan = chunk.nan.data() + axis * chunk.points;
    bytes += withV6Predictor(predictor, [&](auto predictor_tag) {
      return estimateV6Axis<decltype(predictor_tag)::value>(
          quantized, nan, valid, count, lag, streams.history.data(), budget - bytes);
    });
  }
  return bytes;
}

// The predictor with the smallest stage-1 size on the first points of the chunk.
V6Predictor chooseV6Predictor(const V6ChunkGeometry& chunk, size_t lag, V6Streams& streams) {
  const size_t probe = std::min(chunk.points, kV6ProbePoints);
  const V6Predictor candidates[4] = {
      V6Predictor::Previous, V6Predictor::SecondOrder, V6Predictor::LagK, V6Predictor::Median};
  const size_t count = (lag > 1 && lag + 1 < probe) ? 4 : 2;  // LagK and Median need a lag within the probe
  V6Predictor best = V6Predictor::Previous;
  size_t best_bytes = std::numeric_limits<size_t>::max();
  for (size_t k = 0; k < count; ++k) {
    // a candidate wins only with a smaller size, so its estimate can stop at the best size so far
    const size_t bytes = estimateV6Streams(chunk, probe, candidates[k], lag, streams, best_bytes);
    if (bytes < best_bytes) {
      best_bytes = bytes;
      best = candidates[k];
    }
  }
  return best;
}

//------------------------------------------------------------------------------------------
// Encoder: float columns

// One float column: u8 mode (V6GeometryMode), then the raw FLOAT32 values or the residual stream with the
// Previous predictor. One pass in float precision; when a value needs double precision, the two-pass path.
void encodeV6FloatColumn(
    const uint8_t* points, size_t point_step, uint32_t offset, float resolution, size_t n, V6Streams& streams,
    std::vector<int64_t>& quantized, std::vector<uint8_t>& nan, BufferView& out) {
  streams.history.ensure(n);
  streams.data[0].ensure(n * kMaxVarintBytes);
  const float inv_resolution_f = 1.0f / resolution;
#if defined(__SSE4_1__)
  {
    // 4 values per step, the same stream as the loop below
    uint8_t* cursor = streams.data[0].data();
    const V6VarintPack* table = v6VarintPackTable().data();
    const __m128 inv = _mm_set1_ps(inv_resolution_f);
    const __m128 sign = _mm_set1_ps(-0.0f), limit = _mm_set1_ps(kV6FloatQuantized);
    const __m128i none = _mm_setzero_si128();
    int32_t previous = 0;
    size_t i = 0;
    bool coded = true;
    for (; i + 4 <= n; i += 4) {
      const uint8_t* p = points + i * point_step + offset;
      const __m128 v =
          _mm_setr_ps(readF32(p), readF32(p + point_step), readF32(p + 2 * point_step), readF32(p + 3 * point_step));
      const __m128 scaled = _mm_round_ps(_mm_mul_ps(v, inv), _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC);
      if (_mm_movemask_ps(_mm_cmpge_ps(_mm_andnot_ps(sign, scaled), limit)) != 0) {
        coded = false;  // NaN lanes compare false
        break;
      }
      const __m128 is_nan = _mm_cmpunord_ps(v, v);
      const __m128i q = _mm_cvtps_epi32(_mm_andnot_ps(is_nan, scaled));
      const int nan_bits = _mm_movemask_ps(is_nan);
      __m128i predicted;
      if (nan_bits == 0) {  // previous value of each lane: the lane before, the last value for lane 0
        predicted = _mm_alignr_epi8(q, _mm_set1_epi32(previous), 12);  // previous in lane 3
        previous = _mm_extract_epi32(q, 3);
      } else {  // a NaN keeps the prediction
        int32_t lane[4], pred[4];
        _mm_storeu_si128(reinterpret_cast<__m128i*>(lane), q);
        for (int k = 0; k < 4; ++k) {
          pred[k] = previous;
          previous = ((nan_bits >> k) & 1) ? previous : lane[k];
        }
        predicted = _mm_loadu_si128(reinterpret_cast<const __m128i*>(pred));
      }
      cursor = writeV6Varints4(table, _mm_sub_epi32(q, predicted), _mm_castps_si128(is_nan), none, cursor);
    }
    for (; coded && i < n; ++i) {
      const float v = readF32(points + i * point_step + offset);
      if (std::isnan(v)) {
        *cursor++ = 0;
        continue;
      }
      const float scaled = std::nearbyint(v * inv_resolution_f);
      if (!(std::fabs(scaled) < kV6FloatQuantized)) {
        coded = false;
        break;
      }
      const auto q = static_cast<int32_t>(scaled);
      cursor += encodeVarint64(int64_t{q} - previous, cursor);
      previous = q;
    }
    if (coded) {
      appendByte(out, static_cast<uint8_t>(V6GeometryMode::Predicted));
      appendBytes(out, streams.data[0].data(), static_cast<size_t>(cursor - streams.data[0].data()));
      return;
    }
  }
#endif
  {
    uint8_t* cursor = streams.data[0].data();
    int64_t previous = 0;
    size_t i = 0;
    for (; i < n; ++i) {
      const float v = readF32(points + i * point_step + offset);
      if (std::isnan(v)) {
        *cursor++ = 0;  // NaN marker; the prediction stays
        continue;
      }
      const float scaled = std::nearbyint(v * inv_resolution_f);
      if (!(std::fabs(scaled) < kV6FloatQuantized)) {
        break;
      }
      const auto q = static_cast<int64_t>(scaled);
      cursor += encodeVarint64(q - previous, cursor);
      previous = q;
    }
    if (i == n) {
      appendByte(out, static_cast<uint8_t>(V6GeometryMode::Predicted));
      appendBytes(out, streams.data[0].data(), static_cast<size_t>(cursor - streams.data[0].data()));
      return;
    }
  }
  quantized.resize(n);
  nan.resize(n);
  const double inv_resolution = 1.0 / static_cast<double>(resolution);
  for (size_t i = 0; i < n; ++i) {
    const float v = readF32(points + i * point_step + offset);
    nan[i] = std::isnan(v);
    const double scaled = quantizeV6Value(v, inv_resolution_f, inv_resolution);
    if (!(std::fabs(scaled) < kV6MaxQuantized)) {  // infinite or too large: the column is stored raw
      appendByte(out, static_cast<uint8_t>(V6GeometryMode::Raw));
      appendRawFloatColumn(out, points, point_step, offset, n);
      return;
    }
    quantized[i] = static_cast<int64_t>(scaled);
  }
  appendByte(out, static_cast<uint8_t>(V6GeometryMode::Predicted));
  const size_t bytes = encodeV6Axis<V6Predictor::Previous>(
      quantized.data(), nan.data(), nullptr, n, 0, streams.history.data(), streams.data[0].data());
  appendBytes(out, streams.data[0].data(), bytes);
}

}  // namespace

//==========================================================================================
// Encoder

bool UsesV6Codec(const EncodingInfo& info) {
  if (info.version < 6 || info.encoding_opt != EncodingOptions::LOSSY || info.fields.size() < kV6GeometryFields) {
    return false;
  }
  for (size_t axis = 0; axis < kV6GeometryFields; ++axis) {
    const auto& field = info.fields[axis];
    if (field.type != FieldType::FLOAT32 || !isV6Resolution(field.resolution)) {
      return false;
    }
  }
  return true;
}

size_t V6StageBufferSize(const EncodingInfo& info, size_t points_per_chunk) {
  return V5StageBufferSize(info, points_per_chunk) + points_per_chunk / 8 + 1024;
}

// What an encoder reuses from cloud to cloud: buffers, and the encoder of the integer fields with the modes
// it chose (the fields of a PointcloudEncoder never change).
struct V6EncoderScratch {
  std::unique_ptr<AdaptiveIntSectionsEncoder> adaptive;
  V6ChunkGeometry chunk;
  V6Streams streams;
  std::vector<uint8_t> mask_bits;
  std::vector<size_t> section_starts;
  std::vector<int64_t> column_quantized;
  std::vector<uint8_t> column_nan;
};

namespace {

// Probe the lag, the predictors, the mask kind and the modes of the integer fields again every
// kV6ReprobeInterval clouds.
constexpr uint32_t kV6ReprobeInterval = 16;

// Encodes the chunks of one cloud. The lag, the predictor and mask kind of each chunk, and the modes of the
// integer fields come from the previous clouds of the stream (V6EncoderState), which may have another size:
// unorganized scans vary from cloud to cloud. They are probed again every kV6ReprobeInterval clouds, and
// the chunk choices also for a chunk the previous clouds did not have.
class V6CloudEncoder {
 public:
  using ChunkChoice = V6EncoderState::ChunkChoice;

  V6CloudEncoder(const EncodingInfo& info, V6EncoderState& state, size_t points_count, size_t points_per_chunk)
      : info_(info), state_(state), geometry_(makeV6Geometry(info)) {
    std::vector<size_t> adaptive_indexes;
    for (size_t index = kV6GeometryFields; index < info.fields.size(); ++index) {
      const auto& field = info.fields[index];
      if (isV6FloatColumn(info, index)) {
        regular_.emplace_back(index, nullptr);
      } else if (IsAdaptiveIntType(field.type)) {
        adaptive_indexes.push_back(index);
      } else {
        regular_.emplace_back(index, CreateCompatibleEncoder(info, field));
      }
    }
    if (!state_.scratch) {
      state_.scratch = std::make_shared<V6EncoderScratch>();
    }
    if (!state_.scratch->adaptive) {
      state_.scratch->adaptive = std::make_unique<AdaptiveIntSectionsEncoder>(info, adaptive_indexes, points_per_chunk);
    }

    const size_t chunks_count = (points_count + points_per_chunk - 1) / points_per_chunk;
    if (state_.encodes++ % kV6ReprobeInterval == 0) {
      state_.lag.reset();
      state_.chunks.assign(chunks_count, {});
      state_.scratch->adaptive->resetModes();
    } else {
      state_.chunks.resize(chunks_count);
    }
    if (info.height > 1) {  // organized clouds: the point one row up
      state_.lag = info.width;
    }
  }

  std::vector<size_t>& sectionStarts() {
    return state_.scratch->section_starts;
  }

  // Appends the stage-1 sections of chunk `chunk_index` (n points) to `out`, calling mark_section() before
  // each one.
  void encodeChunk(
      const uint8_t* points, size_t n, size_t chunk_index, BufferView& out, const std::function<void()>& mark_section) {
    ChunkChoice& choice = state_.chunks[chunk_index];
    if (choice.predictor == ChunkChoice::kNotProbed) {
      probeChunk(points, n, choice);
    }
    V6Streams& streams = state_.scratch->streams;
    const V6GeometryHeader header = codeGeometry(points, n, choice);
    appendV6GeometryHeader(header, out);
    for (size_t axis = 0; axis < kV6GeometryFields; ++axis) {
      mark_section();
      if (header.mode == V6GeometryMode::Raw) {
        appendRawFloatColumn(out, points, info_.point_step, geometry_.offset[axis], n);
      } else {
        appendBytes(out, streams.data[axis].data(), streams.size[axis]);
      }
    }
    appendOtherFields(points, n, out, mark_section);
  }

 private:
  // Chooses the lag (once per probe) and the predictor of the chunk on its first points, and counts its
  // mask kind over all of them, so that every choice is the one the two-pass path would make. A chunk
  // whose first points are raw stays unprobed: it is raw.
  void probeChunk(const uint8_t* points, size_t n, ChunkChoice& choice) {
    V6ChunkGeometry& chunk = state_.scratch->chunk;
    const V6MaskKind chunk_mask = scanV6MaskKind(geometry_, points, info_.point_step, n);
    const size_t prefix = std::min(n, state_.lag ? kV6ProbePoints : kV6LagDetectPoints);
    quantizeV6Chunk(geometry_, points, info_.point_step, prefix, chunk, chunk_mask);
    if (chunk.raw) {
      return;
    }
    if (!state_.lag) {
      state_.lag = detectV6Lag(chunk);
    }
    choice.predictor = static_cast<uint8_t>(chooseV6Predictor(chunk, *state_.lag, state_.scratch->streams));
    choice.mask = static_cast<uint8_t>(chunk_mask);
  }

  // The predictor of the chunk, or Previous when the lag does not fit the chunk.
  V6Predictor usablePredictor(const ChunkChoice& choice, size_t n) const {
    const auto predictor = static_cast<V6Predictor>(choice.predictor);
    if ((predictor == V6Predictor::LagK || predictor == V6Predictor::Median) &&
        (*state_.lag < 2 || *state_.lag + 1 >= n)) {
      return V6Predictor::Previous;
    }
    return predictor;
  }

  // Codes the geometry of the chunk into the scratch streams: in one pass (the steady state), or in two
  // (quantize, then code) when a value needs double precision. Returns the header of the geometry section:
  // Raw when a value cannot be coded.
  V6GeometryHeader codeGeometry(const uint8_t* points, size_t n, ChunkChoice& choice) {
    V6EncoderScratch& scratch = *state_.scratch;
    std::vector<uint8_t>& mask_bits = scratch.mask_bits;
    V6GeometryHeader header;
    header.mode = V6GeometryMode::Predicted;
    header.lag = state_.lag.value_or(0);

    bool coded = false;
    if (choice.predictor != ChunkChoice::kNotProbed) {
      header.predictor = usablePredictor(choice, n);
      header.mask = static_cast<V6MaskKind>(choice.mask);
      mask_bits.assign(header.mask == V6MaskKind::None ? 0 : (n + 7) / 8, 0);
      coded = buildV6StreamsFused(
          geometry_, points, info_.point_step, n, header.predictor, header.lag, header.mask, scratch.streams,
          mask_bits.data());
    }
    if (!coded) {
      V6ChunkGeometry& chunk = scratch.chunk;
      quantizeV6Chunk(geometry_, points, info_.point_step, n, chunk);
      if (chunk.raw) {
        return V6GeometryHeader{};  // mode Raw
      }
      // the chunk was probed (it is not raw); the mask kind is the one of this cloud
      choice.mask = static_cast<uint8_t>(chunk.mask);
      header.predictor = usablePredictor(choice, n);
      header.mask = chunk.mask;
      buildV6Streams(chunk, n, header.predictor, header.lag, scratch.streams);
      mask_bits.assign(header.mask == V6MaskKind::None ? 0 : (n + 7) / 8, 0);
      if (header.mask != V6MaskKind::None) {
        for (size_t i = 0; i < n; ++i) {
          mask_bits[i / 8] |= static_cast<uint8_t>(chunk.valid[i] << (i % 8));
        }
      }
    }
    header.valid_bits = mask_bits;
    header.stream_bytes = {scratch.streams.size[0], scratch.streams.size[1]};
    return header;
  }

  // Every other non-integer field in its own column, then the integer fields as V5 adaptive sections.
  void appendOtherFields(const uint8_t* points, size_t n, BufferView& out, const std::function<void()>& mark_section) {
    V6EncoderScratch& scratch = *state_.scratch;
    for (auto& [index, encoder] : regular_) {
      mark_section();
      const auto& field = info_.fields[index];
      if (!encoder) {
        encodeV6FloatColumn(
            points, info_.point_step, field.offset, *field.resolution, n, scratch.streams, scratch.column_quantized,
            scratch.column_nan, out);
        continue;
      }
      encoder->reset();
      encoder->encodePoints(points, info_.point_step, n, out);  // one call per column: copied fields in one loop
      encoder->flush(out);
    }
    scratch.adaptive->encodeChunk(points, info_.point_step, n, info_.compression_opt, out, mark_section);
  }

  const EncodingInfo& info_;
  V6EncoderState& state_;
  const V6Geometry geometry_;
  // non-integer fields after x, y, z in field order: a float column (null encoder) or a V4 field encoder
  std::vector<std::pair<size_t, std::unique_ptr<FieldEncoder>>> regular_;
};

}  // namespace

void EncodeV6Stage1(
    const EncodingInfo& info, V6EncoderState& state, ConstBufferView cloud_data, size_t points_count,
    size_t points_per_chunk, const std::function<BufferView()>& get_stage_buffer,
    const std::function<void(size_t serialized_size, std::span<const size_t> section_starts)>& write_stage1_chunk) {
  V6CloudEncoder encoder(info, state, points_count, points_per_chunk);
  std::vector<size_t>& section_starts = encoder.sectionStarts();
  size_t chunk_index = 0;
  for (size_t first = 0; first < points_count; first += points_per_chunk, ++chunk_index) {
    const size_t n = std::min(points_per_chunk, points_count - first);
    BufferView stage_buffer = get_stage_buffer();
    BufferView out(stage_buffer.data(), stage_buffer.size());
    section_starts.clear();
    // stage 2 starts a new ZSTD block at each section
    auto mark_section = [&] { section_starts.push_back(stage_buffer.size() - out.size()); };
    encoder.encodeChunk(cloud_data.data() + first * info.point_step, n, chunk_index, out, mark_section);
    write_stage1_chunk(stage_buffer.size() - out.size(), section_starts);
  }
}

//==========================================================================================
// Decoder

void BuildV6Decoders(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders, size_t& min_encoded_point_bytes) {
  decoders.clear();
  min_encoded_point_bytes = 0;
  for (size_t index = kV6GeometryFields; index < info.fields.size(); ++index) {
    if (isV6RegularField(info, index) && !isV6FloatColumn(info, index)) {
      decoders.push_back(CreateCompatibleDecoder(info, info.fields[index]));
    }
  }
}

namespace {

// Reads one residual; returns false for the NaN marker. Unchecked: at least kMaxVarintBytes are readable.
// Not forced inline: with its long-varint fallback inlined too, the geometry loop decodes 1-3% slower.
template <bool Checked>
inline bool v6ReadResidual(const uint8_t*& ptr, const uint8_t* end, int64_t& residual) {
  if constexpr (Checked) {
    if (ptr == end) {
      throw std::runtime_error("V6: truncated geometry stream");
    }
  }
  if (*ptr == 0) {
    ++ptr;
    return false;
  }
  uint64_t uval;
  if (!Checked && ptr[0] < 0x80u) {
    uval = ptr[0];
    ptr += 1;
  } else if (!Checked && ptr[1] < 0x80u) {
    uval = static_cast<uint64_t>(ptr[0] & 0x7Fu) | (static_cast<uint64_t>(ptr[1]) << 7);
    ptr += 2;
  } else if (!Checked && ptr[2] < 0x80u) {
    uval = static_cast<uint64_t>(ptr[0] & 0x7Fu) | (static_cast<uint64_t>(ptr[1] & 0x7Fu) << 7) |
           (static_cast<uint64_t>(ptr[2]) << 14);
    ptr += 3;
  } else {
    ptr += Checked ? decodeVarint(ptr, static_cast<size_t>(end - ptr), residual) : decodeVarintUnchecked(ptr, residual);
    return true;
  }
  uval--;  // same zig-zag as decodeVarint (0 is the NaN marker, handled above)
  residual = static_cast<int64_t>((uval >> 1) ^ static_cast<uint64_t>(-static_cast<int64_t>(uval & 1)));
  return true;
}

// Value i of a valid point: the prediction plus the residual, NaN for the NaN marker (the history then keeps
// the prediction). The Previous predictor reads and updates only `previous`, the others `history`.
template <V6Predictor P, bool Checked, bool Interior>
CLOUDINI_V6_INLINE float decodeV6Value(
    const uint8_t*& ptr, const uint8_t* end, int64_t* history, int64_t& previous, double resolution, size_t i,
    size_t lag) {
  int64_t& slot = P == V6Predictor::Previous ? previous : history[i];
  const int64_t prediction = P == V6Predictor::Previous ? previous : v6Predict<P, Interior>(history, i, lag);
  int64_t residual = 0;
  if (!v6ReadResidual<Checked>(ptr, end, residual)) {
    slot = prediction;
    return std::numeric_limits<float>::quiet_NaN();
  }
  slot = wrapV6(uint64_t(prediction) + uint64_t(residual));
  return v6Reconstruct(slot, resolution);
}

CLOUDINI_V6_INLINE void storeV6Value(uint8_t* out, size_t i, size_t point_step, float value) {
  if (out) {
    std::memcpy(out + i * point_step, &value, sizeof(float));
  }
}

// One axis of the geometry being decoded: where its stream is, the values the predictors read, and where
// the floats go.
struct V6StreamReader {
  const uint8_t* ptr = nullptr;
  const uint8_t* end = nullptr;
  int64_t* history = nullptr;  // the spatial predictors read every reconstructed value
  int64_t previous = 0;        // the Previous predictor reads only the last one
  uint8_t* out = nullptr;      // null: kDecodeButSkipStore
  double resolution = 0.0;

  template <V6Predictor P, bool Checked, bool Interior>
  CLOUDINI_V6_INLINE float decode(size_t i, size_t lag) {
    return decodeV6Value<P, Checked, Interior>(ptr, end, history, previous, resolution, i, lag);
  }

  // Point i is invalid (not in the stream): it repeats the previous value.
  template <V6Predictor P>
  CLOUDINI_V6_INLINE void skip(size_t i) {
    if constexpr (P != V6Predictor::Previous) {
      history[i] = i >= 1 ? history[i - 1] : 0;
    }
  }

  CLOUDINI_V6_INLINE void store(size_t i, size_t point_step, float value) const {
    storeV6Value(out, i, point_step, value);
  }
};

// Points [first, first + count) of the geometry: `valid` has bit k set when point first + k is valid.
// Checked: the streams may end within the block. Interior: first >= lag + 2.
template <V6Predictor P, bool Checked, bool Interior>
CLOUDINI_V6_INLINE void decodeV6GeometryBlock(
    std::array<V6StreamReader, 3>& readers, size_t first, size_t count, uint64_t valid, bool all_valid, size_t lag,
    size_t point_step, float invalid_value) {
  // locals, so that the stream pointers stay in registers
  V6StreamReader x = readers[0], y = readers[1], z = readers[2];
  for (size_t i = first; i < first + count; ++i) {
    if (!all_valid && !((valid >> (i - first)) & 1u)) {
      x.skip<P>(i);
      y.skip<P>(i);
      z.skip<P>(i);
      x.store(i, point_step, invalid_value);
      y.store(i, point_step, invalid_value);
      z.store(i, point_step, invalid_value);
      continue;
    }
    const float vx = x.decode<P, Checked, Interior>(i, lag);
    const float vy = y.decode<P, Checked, Interior>(i, lag);
    const float vz = z.decode<P, Checked, Interior>(i, lag);
    x.store(i, point_step, vx);
    y.store(i, point_step, vy);
    z.store(i, point_step, vz);
  }
  // only the stream positions and the Previous values change
  readers[0].ptr = x.ptr;
  readers[1].ptr = y.ptr;
  readers[2].ptr = z.ptr;
  readers[0].previous = x.previous;
  readers[1].previous = y.previous;
  readers[2].previous = z.previous;
}

// Decodes x, y and z together, 64 points at a time: one word of the validity mask per block, and a block
// of valid points skips the per-point test. A block whose streams all have room for its longest varints is
// decoded unchecked.
template <V6Predictor P>
void decodeV6Geometry(
    std::array<V6StreamReader, 3>& readers, size_t n, size_t lag, std::span<const uint8_t> valid_bits,
    size_t point_step, float invalid_value) {
  for (size_t first = 0; first < n; first += 64) {
    const size_t count = std::min<size_t>(64, n - first);
    uint64_t valid = ~uint64_t(0);
    if (!valid_bits.empty()) {
      valid = 0;
      std::memcpy(&valid, valid_bits.data() + first / 8, std::min<size_t>(8, valid_bits.size() - first / 8));
    }
    const uint64_t block_bits = count == 64 ? ~uint64_t(0) : (uint64_t(1) << count) - 1;
    const bool all_valid = (~valid & block_bits) == 0;
    const size_t worst = count * kMaxVarintBytes;
    const bool fast = std::all_of(readers.begin(), readers.end(), [&](const V6StreamReader& reader) {
      return static_cast<size_t>(reader.end - reader.ptr) >= worst;
    });
    if (!fast) {
      decodeV6GeometryBlock<P, true, false>(readers, first, count, valid, all_valid, lag, point_step, invalid_value);
    } else if (P != V6Predictor::Previous && first >= lag + 2) {  // Previous has no boundary tests to skip
      decodeV6GeometryBlock<P, false, true>(readers, first, count, valid, all_valid, lag, point_step, invalid_value);
    } else {
      decodeV6GeometryBlock<P, false, false>(readers, first, count, valid, all_valid, lag, point_step, invalid_value);
    }
  }
}

// The geometry section of a chunk of n points, after its header.
void decodeV6GeometrySection(
    const V6GeometryHeader& header, const V6Geometry& geometry, ConstBufferView& input, uint8_t* points,
    size_t point_step, size_t n) {
  if (header.mode == V6GeometryMode::Raw) {
    for (size_t axis = 0; axis < kV6GeometryFields; ++axis) {
      readRawFloatColumn(input, points, point_step, geometry.offset[axis], n);
    }
    return;
  }
  const uint8_t* x_stream = input.data();
  const uint8_t* y_stream = x_stream + header.stream_bytes[0];
  const uint8_t* z_stream = y_stream + header.stream_bytes[1];
  const uint8_t* section_end = input.data() + input.size();
  const std::array<const uint8_t*, 3> begin = {x_stream, y_stream, z_stream};
  const std::array<const uint8_t*, 3> end = {y_stream, z_stream, section_end};

  thread_local std::vector<int64_t> history;
  history.resize(3 * n);
  std::array<V6StreamReader, 3> readers;
  for (size_t axis = 0; axis < kV6GeometryFields; ++axis) {
    readers[axis].ptr = begin[axis];
    readers[axis].end = end[axis];
    readers[axis].history = history.data() + axis * n;
    readers[axis].out = geometry.offset[axis] == kDecodeButSkipStore ? nullptr : points + geometry.offset[axis];
    readers[axis].resolution = static_cast<double>(geometry.resolution[axis]);
  }
  const float invalid_value = header.mask == V6MaskKind::NaN ? std::numeric_limits<float>::quiet_NaN() : 0.0f;
  withV6Predictor(header.predictor, [&](auto predictor_tag) {
    decodeV6Geometry<decltype(predictor_tag)::value>(
        readers, n, header.lag, header.valid_bits, point_step, invalid_value);
  });
  if (readers[0].ptr != end[0] || readers[1].ptr != end[1]) {
    throw std::runtime_error("V6: geometry stream sizes do not match their content");
  }
  input.trim_front(static_cast<size_t>(readers[2].ptr - input.data()));
}

// A float column with the Previous predictor, after its mode byte.
void decodeV6FloatColumn(
    ConstBufferView& input, uint8_t* points, size_t point_step, uint32_t offset, float resolution, size_t n) {
  const uint8_t* ptr = input.data();
  const uint8_t* const end = input.data() + input.size();
  uint8_t* const out = offset == kDecodeButSkipStore ? nullptr : points + offset;
  const double res = static_cast<double>(resolution);
  int64_t previous = 0;
  size_t i = 0;
  for (; i < n && static_cast<size_t>(end - ptr) >= kMaxVarintBytes; ++i) {
    storeV6Value(
        out, i, point_step, decodeV6Value<V6Predictor::Previous, false, false>(ptr, end, nullptr, previous, res, i, 0));
  }
  for (; i < n; ++i) {
    storeV6Value(
        out, i, point_step, decodeV6Value<V6Predictor::Previous, true, false>(ptr, end, nullptr, previous, res, i, 0));
  }
  input.trim_front(static_cast<size_t>(ptr - input.data()));
}

// The float columns and the V4-coded fields after x, y, z, in field order.
void decodeV6RegularFields(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders, ConstBufferView& input,
    uint8_t* points, size_t n) {
  size_t next_decoder = 0;
  for (size_t index = kV6GeometryFields; index < info.fields.size(); ++index) {
    if (!isV6RegularField(info, index)) {
      continue;
    }
    if (!isV6FloatColumn(info, index)) {
      if (next_decoder >= decoders.size()) {
        throw std::runtime_error("V6: decoders do not match the fields");
      }
      auto& decoder = decoders[next_decoder++];
      decoder->reset();
      decoder->decodePoints(input, points, info.point_step, n);
      continue;
    }
    const auto& field = info.fields[index];
    uint8_t mode = 0;
    decode(input, mode);
    if (mode == static_cast<uint8_t>(V6GeometryMode::Raw)) {
      readRawFloatColumn(input, points, info.point_step, field.offset, n);
    } else if (mode == static_cast<uint8_t>(V6GeometryMode::Predicted)) {
      decodeV6FloatColumn(input, points, info.point_step, field.offset, *field.resolution, n);
    } else {
      throw std::runtime_error("V6: unknown float column mode");
    }
  }
}

}  // namespace

void DecodeV6Stage1Chunk(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders, ConstBufferView& encoded_view,
    BufferView& output_buffer, size_t expected_points) {
  const size_t n = expected_points;
  if (n == 0) {
    throw std::runtime_error("V6 chunks require an expected point count");
  }
  if (output_buffer.size() < n * info.point_step) {
    throw std::runtime_error("Output buffer is too small to hold the decoded V6 data");
  }
  uint8_t* points = output_buffer.data();

  const V6GeometryHeader header = readV6GeometryHeader(encoded_view, n);
  decodeV6GeometrySection(header, makeV6Geometry(info), encoded_view, points, info.point_step, n);
  decodeV6RegularFields(info, decoders, encoded_view, points, n);
  DecodeAdaptiveIntSections(info, encoded_view, points, info.point_step, n);
  if (!encoded_view.empty()) {
    throw std::runtime_error("V6 chunk has trailing bytes after decode");
  }
  output_buffer.trim_front(n * info.point_step);
}

}  // namespace Cloudini::detail
