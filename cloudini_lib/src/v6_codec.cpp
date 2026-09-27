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
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include "cloudini_lib/encoding_utils.hpp"
#include "cloudini_lib/field_encoder.hpp"
#include "codec_common.hpp"
#include "v5_codec.hpp"

namespace Cloudini::detail {

//==========================================================================================
// V6: geometry predicted from the best neighbour, invalid-point mask, one stream per field.
//
// Chunk layout (stage 1):
//   geometry section   u8 mode: 0 = raw FLOAT32 columns x, y, z; 1 = predicted, followed by
//                        u8 predictor (V6Predictor), uvarint K, u8 mask kind (V6MaskKind),
//                        [ceil(n / 8) bytes validity bits, LSB first, when a mask is used],
//                        uvarint size of the x stream, uvarint size of the y stream,
//                        then the x, y and z streams: one varint per valid point (encodeVarint64,
//                        0 = NaN), the residual against the prediction of the quantized value.
//   The decoder reconstructs float(double(q) * resolution). The encoder computes q in float precision
//   (as V4 does) when it fits in 2^22, in double precision otherwise.
//   regular columns    every other non-integer field, its field encoder run over all the points.
//   integer sections   the V5 adaptive sections.
namespace {

enum class V6GeometryMode : uint8_t { Raw = 0, Predicted = 1 };
enum class V6Predictor : uint8_t { Previous = 0, LagK = 1, Median = 2, SecondOrder = 3 };
enum class V6MaskKind : uint8_t { None = 0, NaN = 1, Zero = 2 };

constexpr size_t kV6GeometryFields = 3;
constexpr size_t kV6MaxLag = 2100;
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

struct V6Geometry {
  std::array<uint32_t, 3> offset{};
  std::array<float, 3> resolution{};
  std::array<double, 3> inv_resolution{};
  std::array<float, 3> inv_resolution_f{};
  bool contiguous = false;  // x, y, z at consecutive offsets, 16 bytes readable from x within the point
};

V6Geometry makeV6Geometry(const EncodingInfo& info) {
  V6Geometry g;
  g.contiguous = info.fields[1].offset == info.fields[0].offset + 4 &&
                 info.fields[2].offset == info.fields[0].offset + 8 && info.fields[0].offset + 16 <= info.point_step;
  for (size_t a = 0; a < kV6GeometryFields; ++a) {
    g.offset[a] = info.fields[a].offset;
    g.resolution[a] = *info.fields[a].resolution;
    g.inv_resolution[a] = 1.0 / static_cast<double>(g.resolution[a]);
    g.inv_resolution_f[a] = 1.0f / g.resolution[a];
  }
  return g;
}

float readF32(const uint8_t* ptr) {
  float v;
  std::memcpy(&v, ptr, sizeof(v));
  return v;
}

inline int64_t wrapV6(uint64_t value) {
  return static_cast<int64_t>(value);
}

// Prediction of value i of one axis from the values already reconstructed (same axis only). The
// arithmetic wraps around: values decoded from a corrupted stream can be anywhere in the int64 range
// (the encoder's values stay below 2^50, where wrapping never happens).
template <V6Predictor P>
#if defined(__GNUC__)
__attribute__((always_inline))
#endif
inline int64_t
v6Predict(const int64_t* q, size_t i, size_t K) {
  const int64_t prev = i >= 1 ? q[i - 1] : 0;
  if constexpr (P == V6Predictor::Previous) {
    return prev;
  } else if constexpr (P == V6Predictor::LagK) {
    return i >= K ? q[i - K] : prev;
  } else if constexpr (P == V6Predictor::Median) {
    if (i <= K) {
      return prev;
    }
    // LOCO-I median edge detector
    const int64_t pa = prev, pb = q[i - K], pc = q[i - K - 1];
    const int64_t mx = std::max(pa, pb), mn = std::min(pa, pb);
    return pc >= mx ? mn : (pc <= mn ? mx : wrapV6(uint64_t(pa) + uint64_t(pb) - uint64_t(pc)));
  } else {
    return i >= 2 ? wrapV6(2 * uint64_t(prev) - uint64_t(q[i - 2])) : prev;
  }
}

// Quantized geometry of one chunk, axis-major (value i of axis a at a * points + i).
struct V6ChunkGeometry {
  size_t points = 0;
  std::vector<int64_t> quantized;  // NaN axes as 0
  std::vector<uint8_t> nan;
  std::vector<uint8_t> valid;  // 1 per point: coded in the streams
  V6MaskKind mask = V6MaskKind::None;
  bool raw = false;
};

// x, y, z of one point quantized in float precision (same values as quantizeV6Chunk). Bits 0-2 of
// `nan` / `zero` flag the NaN / 0.0f axes.
struct V6PointQuantized {
  alignas(16) int32_t q[4];
  int nan;
  int zero;
};

// false when a value is infinite or quantizes beyond kV6FloatQuantized.
#if defined(__GNUC__)
__attribute__((always_inline))
#endif
inline bool
quantizeV6Point(const V6Geometry& g, const uint8_t* p, V6PointQuantized& out) {
#if defined(__SSE4_1__)
  const __m128 v =
      g.contiguous ? _mm_blend_ps(_mm_loadu_ps(reinterpret_cast<const float*>(p + g.offset[0])), _mm_setzero_ps(), 0x8)
                   : _mm_setr_ps(readF32(p + g.offset[0]), readF32(p + g.offset[1]), readF32(p + g.offset[2]), 0.0f);
  const __m128 inv = _mm_setr_ps(g.inv_resolution_f[0], g.inv_resolution_f[1], g.inv_resolution_f[2], 0.0f);
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
  for (size_t a = 0; a < 3; ++a) {
    const float v = readF32(p + g.offset[a]);
    const float scaled = std::nearbyint(v * g.inv_resolution_f[a]);
    if (std::isnan(v)) {
      out.nan |= 1 << a;
      out.q[a] = 0;
      continue;
    }
    if (!(std::fabs(scaled) < kV6FloatQuantized)) {
      return false;
    }
    out.zero |= (v == 0.0f) << a;
    out.q[a] = static_cast<int32_t>(scaled);
  }
  return true;
#endif
}

// Mask kind of a chunk: the more frequent of all-NaN and all-zero points, None when there are neither.
V6MaskKind scanV6MaskKind(const V6Geometry& g, const uint8_t* points, size_t point_step, size_t n) {
  size_t nan_points = 0, zero_points = 0;
#if defined(__SSE4_1__)
  for (size_t i = 0; i < n; ++i) {
    const uint8_t* p = points + i * point_step;
    const __m128 v =
        g.contiguous
            ? _mm_blend_ps(_mm_loadu_ps(reinterpret_cast<const float*>(p + g.offset[0])), _mm_setzero_ps(), 0x8)
            : _mm_setr_ps(readF32(p + g.offset[0]), readF32(p + g.offset[1]), readF32(p + g.offset[2]), 0.0f);
    nan_points += (_mm_movemask_ps(_mm_cmpunord_ps(v, v)) & 7) == 7;
    zero_points += (_mm_movemask_ps(_mm_cmpeq_ps(v, _mm_setzero_ps())) & 7) == 7;
  }
#else
  for (size_t i = 0; i < n; ++i) {
    const uint8_t* p = points + i * point_step;
    int nan_axes = 0, zero_axes = 0;
    for (size_t a = 0; a < 3; ++a) {
      const float v = readF32(p + g.offset[a]);
      nan_axes += std::isnan(v);
      zero_axes += (v == 0.0f);
    }
    nan_points += (nan_axes == 3);
    zero_points += (zero_axes == 3);
  }
#endif
  if (nan_points == 0 && zero_points == 0) {
    return V6MaskKind::None;
  }
  return nan_points >= zero_points ? V6MaskKind::NaN : V6MaskKind::Zero;
}

// Quantizes the first n points of a chunk. The mask kind is the one of these n points, unless `mask` gives
// the kind of the whole chunk (scanV6MaskKind), when only a prefix of the chunk is quantized for probing.
void quantizeV6Chunk(
    const V6Geometry& g, const uint8_t* points, size_t point_step, size_t n, V6ChunkGeometry& out,
    std::optional<V6MaskKind> mask = std::nullopt) {
  out.points = n;
  out.quantized.resize(n * 3);
  out.nan.resize(n * 3);
  out.valid.assign(n, 1);
  out.raw = false;
  out.mask = V6MaskKind::None;
  size_t nan_points = 0, zero_points = 0;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t* p = points + i * point_step;
    V6PointQuantized pq;
    if (quantizeV6Point(g, p, pq)) {  // the same values as the per-axis code below
      for (size_t a = 0; a < 3; ++a) {
        out.nan[a * n + i] = (pq.nan >> a) & 1;
        out.quantized[a * n + i] = out.raw ? 0 : pq.q[a];
      }
      nan_points += (pq.nan == 7);
      zero_points += (pq.zero == 7);
      continue;
    }
    int nan_axes = 0, zero_axes = 0;
    for (size_t a = 0; a < 3; ++a) {
      const float v = readF32(p + g.offset[a]);
      const bool is_nan = std::isnan(v);
      out.nan[a * n + i] = is_nan;
      nan_axes += is_nan;
      zero_axes += (v == 0.0f);
      const float scaled_f = std::nearbyint(v * g.inv_resolution_f[a]);
      const double scaled = is_nan ? 0.0
                            : std::fabs(scaled_f) < kV6FloatQuantized
                                ? static_cast<double>(scaled_f)
                                : std::nearbyint(static_cast<double>(v) * g.inv_resolution[a]);
      if (!(std::fabs(scaled) < kV6MaxQuantized)) {
        out.raw = true;  // infinite or too large for the integer streams
      }
      out.quantized[a * n + i] = out.raw ? 0 : static_cast<int64_t>(scaled);
    }
    nan_points += (nan_axes == 3);
    zero_points += (zero_axes == 3);
  }
  if (out.raw) {
    return;
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
    for (size_t a = 0; a < 3; ++a) {
      const bool is_nan = out.nan[a * n + i];
      invalid &= out.mask == V6MaskKind::NaN ? is_nan
                                             : (!is_nan && out.quantized[a * n + i] == 0 &&
                                                readF32(points + i * point_step + g.offset[a]) == 0.0f);
    }
    out.valid[i] = !invalid;
  }
}

// Residual stream of one axis for predictor P, written at `out`; returns its size. `f` receives the
// values the decoder reconstructs (an invalid point repeats the previous value, a NaN axis the prediction).
template <V6Predictor P>
size_t encodeV6Axis(
    const int64_t* q, const uint8_t* nan, const uint8_t* valid, size_t n, size_t K, int64_t* f, uint8_t* out) {
  uint8_t* o = out;
  for (size_t i = 0; i < n; ++i) {
    if (valid && !valid[i]) {
      f[i] = i >= 1 ? f[i - 1] : 0;
      continue;
    }
    const int64_t pred = v6Predict<P>(f, i, K);
    if (nan[i]) {
      f[i] = pred;
      *o++ = 0;  // NaN marker
      continue;
    }
    f[i] = q[i];
    o += encodeVarint64(q[i] - pred, o);
  }
  return static_cast<size_t>(o - out);
}

// Varint length of a value by its count of leading zero bits: ceil((64 - clz) / 7).
constexpr std::array<uint8_t, 64> kV6VarintBytes = [] {
  std::array<uint8_t, 64> t{};
  for (int clz = 0; clz < 64; ++clz) {
    t[clz] = static_cast<uint8_t>((64 - clz + 6) / 7);
  }
  return t;
}();

// Size of the residual stream of one axis for predictor P, without writing it.
template <V6Predictor P>
// Stops early, returning at least `budget`, once the size reaches `budget`.
size_t estimateV6Axis(
    const int64_t* q, const uint8_t* nan, const uint8_t* valid, size_t n, size_t K, int64_t* f, size_t budget) {
  size_t bytes = 0;
  for (size_t i = 0; i < n; ++i) {
    if ((i & 511) == 0 && bytes >= budget) {
      return bytes;
    }
    if (valid && !valid[i]) {
      f[i] = i >= 1 ? f[i - 1] : 0;
      continue;
    }
    const int64_t pred = v6Predict<P>(f, i, K);
    if (nan[i]) {
      f[i] = pred;
      ++bytes;
      continue;
    }
    f[i] = q[i];
    const int64_t d = q[i] - pred;
    const uint64_t zz = (static_cast<uint64_t>(d) << 1) ^ static_cast<uint64_t>(d >> 63);
    bytes += kV6VarintBytes[__builtin_clzll(zz + 1)];  // encodeVarint64 codes zz + 1
  }
  return bytes;
}

// Scratch array that grows without zero-filling: its contents are always written before they are read,
// and are not kept when it grows.
template <typename T>
class ScratchArray {
 public:
  T* data() {
    return data_.get();
  }
  size_t size() const {
    return size_;
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

struct V6Streams {
  std::array<ScratchArray<uint8_t>, 3> data;
  std::array<size_t, 3> size{0, 0, 0};
  ScratchArray<int64_t> filled;
};

// The three residual streams of the first `count` points in one pass over the points.
template <V6Predictor P>
void encodeV6Geometry(const V6ChunkGeometry& c, size_t count, size_t K, V6Streams& s) {
  const size_t n = c.points;
  const uint8_t* valid = c.mask == V6MaskKind::None ? nullptr : c.valid.data();
  const int64_t* q[3] = {c.quantized.data(), c.quantized.data() + n, c.quantized.data() + 2 * n};
  const uint8_t* nan[3] = {c.nan.data(), c.nan.data() + n, c.nan.data() + 2 * n};
  int64_t* f[3] = {s.filled.data(), s.filled.data() + count, s.filled.data() + 2 * count};
  uint8_t* o[3] = {s.data[0].data(), s.data[1].data(), s.data[2].data()};
  for (size_t i = 0; i < count; ++i) {
    if (valid && !valid[i]) {
      for (size_t a = 0; a < 3; ++a) {
        f[a][i] = i >= 1 ? f[a][i - 1] : 0;
      }
      continue;
    }
    for (size_t a = 0; a < 3; ++a) {
      const int64_t pred = v6Predict<P>(f[a], i, K);
      if (nan[a][i]) {
        f[a][i] = pred;
        *o[a]++ = 0;  // NaN marker
      } else {
        f[a][i] = q[a][i];
        o[a] += encodeVarint64(q[a][i] - pred, o[a]);
      }
    }
  }
  for (size_t a = 0; a < 3; ++a) {
    s.size[a] = static_cast<size_t>(o[a] - s.data[a].data());
  }
}

// Quantizes and codes the chunk in one pass, for a predictor and a mask kind already known. With a
// mask, sets the validity bit of each valid point in `valid_bits` (zeroed by the caller). Returns false
// when a value needs the double-precision path (quantizeV6Chunk).
template <V6Predictor P, V6MaskKind M>
bool encodeV6GeometryFused(
    const V6Geometry& g, const uint8_t* points, size_t point_step, size_t n, size_t K, V6Streams& s,
    uint8_t* valid_bits) {
  int64_t* f[3] = {s.filled.data(), s.filled.data() + n, s.filled.data() + 2 * n};
  uint8_t* o[3] = {s.data[0].data(), s.data[1].data(), s.data[2].data()};
  V6PointQuantized pq;
  for (size_t i = 0; i < n; ++i) {
    if (!quantizeV6Point(g, points + i * point_step, pq)) {
      return false;
    }
    if constexpr (M != V6MaskKind::None) {
      if ((M == V6MaskKind::NaN ? pq.nan : pq.zero) == 7) {  // invalid point: repeats the previous value
        for (size_t a = 0; a < 3; ++a) {
          f[a][i] = i >= 1 ? f[a][i - 1] : 0;
        }
        continue;
      }
      valid_bits[i / 8] |= static_cast<uint8_t>(1u << (i % 8));
    }
    for (size_t a = 0; a < 3; ++a) {
      const int64_t pred = v6Predict<P>(f[a], i, K);
      if (pq.nan & (1 << a)) {
        f[a][i] = pred;
        *o[a]++ = 0;  // NaN marker
      } else {
        f[a][i] = pq.q[a];
        o[a] += encodeVarint64(pq.q[a] - pred, o[a]);
      }
    }
  }
  for (size_t a = 0; a < 3; ++a) {
    s.size[a] = static_cast<size_t>(o[a] - s.data[a].data());
  }
  return true;
}

// Stream buffers for `count` points (grow only).
void reserveV6Streams(V6Streams& s, size_t count) {
  s.filled.ensure(3 * count);
  for (size_t a = 0; a < 3; ++a) {
    s.data[a].ensure(count * kMaxVarintBytes);
  }
}

template <V6Predictor P>
bool encodeV6GeometryFused(
    const V6Geometry& g, const uint8_t* points, size_t point_step, size_t n, size_t K, V6MaskKind mask, V6Streams& s,
    uint8_t* valid_bits) {
  switch (mask) {
    case V6MaskKind::None:
      return encodeV6GeometryFused<P, V6MaskKind::None>(g, points, point_step, n, K, s, valid_bits);
    case V6MaskKind::NaN:
      return encodeV6GeometryFused<P, V6MaskKind::NaN>(g, points, point_step, n, K, s, valid_bits);
    default:
      return encodeV6GeometryFused<P, V6MaskKind::Zero>(g, points, point_step, n, K, s, valid_bits);
  }
}

bool buildV6StreamsFused(
    const V6Geometry& g, const uint8_t* points, size_t point_step, size_t n, V6Predictor predictor, size_t K,
    V6MaskKind mask, V6Streams& s, uint8_t* valid_bits) {
  reserveV6Streams(s, n);
  switch (predictor) {
    case V6Predictor::Previous:
      return encodeV6GeometryFused<V6Predictor::Previous>(g, points, point_step, n, K, mask, s, valid_bits);
    case V6Predictor::LagK:
      return encodeV6GeometryFused<V6Predictor::LagK>(g, points, point_step, n, K, mask, s, valid_bits);
    case V6Predictor::Median:
      return encodeV6GeometryFused<V6Predictor::Median>(g, points, point_step, n, K, mask, s, valid_bits);
    default:
      return encodeV6GeometryFused<V6Predictor::SecondOrder>(g, points, point_step, n, K, mask, s, valid_bits);
  }
}

// Residual streams of the first `count` points of the chunk for a predictor.
void buildV6Streams(const V6ChunkGeometry& c, size_t count, V6Predictor predictor, size_t K, V6Streams& s) {
  reserveV6Streams(s, count);
  switch (predictor) {
    case V6Predictor::Previous:
      encodeV6Geometry<V6Predictor::Previous>(c, count, K, s);
      break;
    case V6Predictor::LagK:
      encodeV6Geometry<V6Predictor::LagK>(c, count, K, s);
      break;
    case V6Predictor::Median:
      encodeV6Geometry<V6Predictor::Median>(c, count, K, s);
      break;
    case V6Predictor::SecondOrder:
      encodeV6Geometry<V6Predictor::SecondOrder>(c, count, K, s);
      break;
  }
}

// Lag between a point and "the same laser one firing earlier", from the first chunk: the K with the
// smallest mean distance between points i and i - K, on a sample of points.
size_t detectV6Lag(const V6ChunkGeometry& c) {
  const size_t n = std::min<size_t>(c.points, kV6LagDetectPoints);
  if (n < 64) {
    return 0;
  }
  // forward-filled copy of the first n points, point-major for the lag scan
  std::vector<int64_t> q(n * 3, 0);
  for (size_t i = 0; i < n; ++i) {
    for (size_t a = 0; a < 3; ++a) {
      const bool keep = c.valid[i] && !c.nan[a * c.points + i];
      q[i * 3 + a] = keep ? c.quantized[a * c.points + i] : (i >= 1 ? q[(i - 1) * 3 + a] : 0);
    }
  }
  const size_t max_lag = std::min(kV6MaxLag, n / 2);
  constexpr size_t kSamples = 64;
  const size_t span = n - max_lag;
  size_t best_lag = 0;
  int64_t best_cost = 0;
  for (size_t K = 2; K <= max_lag; ++K) {
    int64_t cost = 0;
    for (size_t s = 0; s < kSamples; ++s) {
      const size_t i = max_lag + (s * span) / kSamples;
      for (size_t a = 0; a < 3; ++a) {
        cost += std::llabs(q[i * 3 + a] - q[(i - K) * 3 + a]);
      }
      if (best_lag != 0 && cost >= best_cost) {
        break;  // a lag wins only with a smaller cost
      }
    }
    if (best_lag == 0 || cost < best_cost) {
      best_lag = K;
      best_cost = cost;
    }
  }
  return best_lag;
}

// Stage-1 size of the first `count` points for a predictor; stops early once it reaches `budget` (the
// result is then at least `budget`).
size_t estimateV6Streams(
    const V6ChunkGeometry& c, size_t count, V6Predictor predictor, size_t K, V6Streams& s, size_t budget) {
  s.filled.ensure(count);
  const uint8_t* valid = c.mask == V6MaskKind::None ? nullptr : c.valid.data();
  size_t bytes = 0;
  for (size_t a = 0; a < 3 && bytes < budget; ++a) {
    const int64_t* q = c.quantized.data() + a * c.points;
    const uint8_t* nan = c.nan.data() + a * c.points;
    switch (predictor) {
      case V6Predictor::Previous:
        bytes += estimateV6Axis<V6Predictor::Previous>(q, nan, valid, count, K, s.filled.data(), budget - bytes);
        break;
      case V6Predictor::LagK:
        bytes += estimateV6Axis<V6Predictor::LagK>(q, nan, valid, count, K, s.filled.data(), budget - bytes);
        break;
      case V6Predictor::Median:
        bytes += estimateV6Axis<V6Predictor::Median>(q, nan, valid, count, K, s.filled.data(), budget - bytes);
        break;
      case V6Predictor::SecondOrder:
        bytes += estimateV6Axis<V6Predictor::SecondOrder>(q, nan, valid, count, K, s.filled.data(), budget - bytes);
        break;
    }
  }
  return bytes;
}

// The predictor with the smallest stage-1 size on the first points of the chunk.
V6Predictor chooseV6Predictor(const V6ChunkGeometry& c, size_t K, V6Streams& s) {
  const size_t probe = std::min(c.points, kV6ProbePoints);
  V6Predictor candidates[4] = {V6Predictor::Previous, V6Predictor::SecondOrder, V6Predictor::LagK, V6Predictor::Median};
  const size_t count = (K > 1 && K + 1 < probe) ? 4 : 2;
  V6Predictor best = V6Predictor::Previous;
  size_t best_cost = std::numeric_limits<size_t>::max();
  for (size_t k = 0; k < count; ++k) {
    // a candidate wins only with a smaller size, so its estimate can stop at the best size so far
    const size_t cost = estimateV6Streams(c, probe, candidates[k], K, s, best_cost);
    if (cost < best_cost) {
      best_cost = cost;
      best = candidates[k];
    }
  }
  return best;
}

void appendBytes(BufferView& out, const uint8_t* data, size_t size) {
  if (out.size() < size) {
    throw std::runtime_error("V6: output buffer full");
  }
  std::memcpy(out.data(), data, size);
  out.trim_front(size);
}

bool isV6RegularField(const EncodingInfo& info, size_t index) {
  return index >= kV6GeometryFields && !IsAdaptiveIntType(info.fields[index].type);
}

// FLOAT32 fields with a resolution after x, y, z (e.g. intensity): coded like one geometry axis, with the
// previous-value predictor, in a column of their own.
bool isV6FloatColumn(const EncodingInfo& info, size_t index) {
  const auto& field = info.fields[index];
  return index >= kV6GeometryFields && field.type == FieldType::FLOAT32 && isV6Resolution(field.resolution);
}

// One V6 float column: u8 mode (V6GeometryMode), then the raw FLOAT32 values or the residual stream.
void encodeV6FloatColumn(
    const uint8_t* base, size_t step, uint32_t offset, float resolution, size_t n, V6Streams& s,
    std::vector<int64_t>& quantized, std::vector<uint8_t>& nan, BufferView& out) {
  s.filled.ensure(n);
  s.data[0].ensure(n * kMaxVarintBytes);
  // one pass in float precision; values that need double precision take the two-pass path below
  const float inv_f = 1.0f / resolution;
  {
    uint8_t* o = s.data[0].data();
    int64_t prev = 0;
    size_t i = 0;
    for (; i < n; ++i) {
      const float v = readF32(base + i * step + offset);
      if (std::isnan(v)) {
        *o++ = 0;  // NaN marker; the prediction stays
        continue;
      }
      const float scaled = std::nearbyint(v * inv_f);
      if (!(std::fabs(scaled) < kV6FloatQuantized)) {
        break;
      }
      const auto q = static_cast<int64_t>(scaled);
      o += encodeVarint64(q - prev, o);
      prev = q;
    }
    if (i == n) {
      appendByte(out, static_cast<uint8_t>(V6GeometryMode::Predicted));
      appendBytes(out, s.data[0].data(), static_cast<size_t>(o - s.data[0].data()));
      return;
    }
  }
  quantized.resize(n);
  nan.resize(n);
  const double inv = 1.0 / static_cast<double>(resolution);
  bool raw = false;
  for (size_t i = 0; i < n; ++i) {
    const float v = readF32(base + i * step + offset);
    nan[i] = std::isnan(v);
    const float scaled_f = std::nearbyint(v * inv_f);
    const double scaled = nan[i]                                    ? 0.0
                          : std::fabs(scaled_f) < kV6FloatQuantized ? static_cast<double>(scaled_f)
                                                                    : std::nearbyint(static_cast<double>(v) * inv);
    raw |= !(std::fabs(scaled) < kV6MaxQuantized);
    quantized[i] = raw ? 0 : static_cast<int64_t>(scaled);
  }
  if (raw) {
    appendByte(out, static_cast<uint8_t>(V6GeometryMode::Raw));
    for (size_t i = 0; i < n; ++i) {
      appendBytes(out, base + i * step + offset, sizeof(float));
    }
    return;
  }
  appendByte(out, static_cast<uint8_t>(V6GeometryMode::Predicted));
  const size_t bytes = encodeV6Axis<V6Predictor::Previous>(
      quantized.data(), nan.data(), nullptr, n, 0, s.filled.data(), s.data[0].data());
  appendBytes(out, s.data[0].data(), bytes);
}

}  // namespace

//==========================================================================================
// V6

bool UsesV6Codec(const EncodingInfo& info) {
  if (info.version < 6 || info.encoding_opt != EncodingOptions::LOSSY || info.fields.size() < kV6GeometryFields) {
    return false;
  }
  for (size_t a = 0; a < kV6GeometryFields; ++a) {
    const auto& field = info.fields[a];
    if (field.type != FieldType::FLOAT32 || !isV6Resolution(field.resolution)) {
      return false;
    }
  }
  return true;
}

size_t V6StageBufferSize(const EncodingInfo& info, size_t points_per_chunk) {
  return V5StageBufferSize(info, points_per_chunk) + points_per_chunk / 8 + 1024;
}

namespace {
struct V6EncoderScratch {
  V6ChunkGeometry chunk;
  V6Streams streams;
  std::vector<uint8_t> mask_bits;
  std::vector<size_t> section_starts;
  std::vector<int64_t> column_quantized;
  std::vector<uint8_t> column_nan;
};
}  // namespace

void EncodeV6Stage1(
    const EncodingInfo& info, V6EncoderState& state, ConstBufferView cloud_data, size_t points_count,
    size_t points_per_chunk, const std::function<BufferView()>& get_stage_buffer,
    const std::function<void(size_t serialized_size, std::span<const size_t> section_starts)>& write_stage1_chunk) {
  const V6Geometry geometry = makeV6Geometry(info);
  // non-integer columns in field order: a V6 float column (null encoder) or a V5 field encoder
  std::vector<std::pair<size_t, std::unique_ptr<FieldEncoder>>> regular;
  std::vector<size_t> adaptive_indexes;
  for (size_t i = kV6GeometryFields; i < info.fields.size(); ++i) {
    const auto& field = info.fields[i];
    if (isV6FloatColumn(info, i)) {
      regular.emplace_back(i, nullptr);
    } else if (IsAdaptiveIntType(field.type)) {
      adaptive_indexes.push_back(i);
    } else {
      regular.emplace_back(i, CreateCompatibleEncoder(info, field));
    }
  }
  AdaptiveIntSectionsEncoder adaptive(info, adaptive_indexes, points_per_chunk);
  if (!state.scratch) {
    state.scratch = std::make_shared<V6EncoderScratch>();
  }
  auto& scratch = *static_cast<V6EncoderScratch*>(state.scratch.get());
  auto& column_quantized = scratch.column_quantized;
  auto& column_nan = scratch.column_nan;
  auto& chunk = scratch.chunk;
  auto& streams = scratch.streams;
  auto& mask_bits = scratch.mask_bits;
  auto& section_starts = scratch.section_starts;

  // Lag, predictor and mask kind of each chunk: reused from the previous clouds of the stream (their size
  // may differ: unorganized scans vary from cloud to cloud), probed again every kV6ReprobeInterval clouds.
  // A chunk the previous clouds did not have is probed now.
  const size_t chunks_count = (points_count + points_per_chunk - 1) / points_per_chunk;
  if (state.encodes % kV6ReprobeInterval == 0) {
    state.lag_known = false;
    state.predictors.assign(chunks_count, 0xFF);
    state.masks.assign(chunks_count, 0xFF);
    state.encodes = 0;
  } else if (state.predictors.size() != chunks_count) {
    state.predictors.resize(chunks_count, 0xFF);
    state.masks.resize(chunks_count, 0xFF);
  }
  state.encodes++;
  if (info.height > 1) {  // organized clouds: the point one row up
    state.lag = info.width;
    state.lag_known = true;
  }

  size_t chunk_index = 0;

  size_t points_left = points_count;
  size_t point_offset = 0;
  while (points_left > 0) {
    const size_t n = std::min(points_left, points_per_chunk);
    const uint8_t* base = cloud_data.data() + point_offset * info.point_step;

    BufferView stage_buffer = get_stage_buffer();
    BufferView out(stage_buffer.data(), stage_buffer.size());
    section_starts.clear();
    // stage 2 starts a new ZSTD block at each section
    auto mark_section = [&] { section_starts.push_back(stage_buffer.size() - out.size()); };

    uint8_t& cached = state.predictors[chunk_index];
    uint8_t& cached_mask = state.masks[chunk_index];
    auto usable_predictor = [&] {
      const auto predictor = static_cast<V6Predictor>(cached);
      if ((predictor == V6Predictor::LagK || predictor == V6Predictor::Median) &&
          (state.lag < 2 || state.lag + 1 >= n)) {
        return V6Predictor::Previous;  // the lag does not fit this chunk
      }
      return predictor;
    };

    // Steady state (lag, predictor and mask kind known from an earlier cloud): quantize and code the
    // geometry in one pass. Otherwise, or when a value needs double precision, the two-pass path.
    bool coded = false;
    V6Predictor predictor = V6Predictor::Previous;
    V6MaskKind mask = V6MaskKind::None;
    const bool chunk_probe_done = state.lag_known && cached != 0xFF && cached_mask != 0xFF;
    if (chunk_probe_done) {
      predictor = usable_predictor();
      mask = static_cast<V6MaskKind>(cached_mask);
      mask_bits.assign(mask == V6MaskKind::None ? 0 : (n + 7) / 8, 0);
      coded = buildV6StreamsFused(
          geometry, base, info.point_step, n, predictor, state.lag, mask, streams, mask_bits.data());
    }
    if (!coded && !chunk_probe_done) {
      // First encode of this cloud size (or a re-probe): the lag and the predictor are chosen on the first
      // points only, so quantize those, then code the chunk in one pass like the steady state. The mask
      // kind is counted over the whole chunk, so every choice is the one the two-pass path would make.
      const V6MaskKind chunk_mask = scanV6MaskKind(geometry, base, info.point_step, n);
      const size_t prefix = std::min(n, state.lag_known ? kV6ProbePoints : kV6LagDetectPoints);
      quantizeV6Chunk(geometry, base, info.point_step, prefix, chunk, chunk_mask);
      if (!chunk.raw) {
        if (!state.lag_known) {
          state.lag = detectV6Lag(chunk);
          state.lag_known = true;
        }
        if (cached == 0xFF) {
          cached = static_cast<uint8_t>(chooseV6Predictor(chunk, state.lag, streams));
        }
        cached_mask = static_cast<uint8_t>(chunk_mask);
        predictor = usable_predictor();
        mask = chunk_mask;
        mask_bits.assign(mask == V6MaskKind::None ? 0 : (n + 7) / 8, 0);
        coded = buildV6StreamsFused(
            geometry, base, info.point_step, n, predictor, state.lag, mask, streams, mask_bits.data());
      }
    }
    if (!coded) {
      quantizeV6Chunk(geometry, base, info.point_step, n, chunk);
    }
    if (!coded && chunk.raw) {
      appendByte(out, static_cast<uint8_t>(V6GeometryMode::Raw));
      for (size_t a = 0; a < kV6GeometryFields; ++a) {
        mark_section();
        for (size_t i = 0; i < n; ++i) {
          appendBytes(out, base + i * info.point_step + geometry.offset[a], sizeof(float));
        }
      }
    } else {
      if (!coded) {
        if (!state.lag_known) {
          state.lag = detectV6Lag(chunk);
          state.lag_known = true;
        }
        if (cached == 0xFF) {
          cached = static_cast<uint8_t>(chooseV6Predictor(chunk, state.lag, streams));
        }
        cached_mask = static_cast<uint8_t>(chunk.mask);
        predictor = usable_predictor();
        mask = chunk.mask;
        buildV6Streams(chunk, n, predictor, state.lag, streams);
        if (mask != V6MaskKind::None) {
          mask_bits.assign((n + 7) / 8, 0);
          for (size_t i = 0; i < n; ++i) {
            mask_bits[i / 8] |= static_cast<uint8_t>(chunk.valid[i] << (i % 8));
          }
        }
      }
      appendByte(out, static_cast<uint8_t>(V6GeometryMode::Predicted));
      appendByte(out, static_cast<uint8_t>(predictor));
      appendUVarint(state.lag, out);
      appendByte(out, static_cast<uint8_t>(mask));
      if (mask != V6MaskKind::None) {
        appendBytes(out, mask_bits.data(), mask_bits.size());
      }
      appendUVarint(streams.size[0], out);
      appendUVarint(streams.size[1], out);
      for (size_t a = 0; a < kV6GeometryFields; ++a) {
        mark_section();
        appendBytes(out, streams.data[a].data(), streams.size[a]);
      }
    }

    // every other non-integer field in its own column
    for (auto& [index, encoder] : regular) {
      mark_section();
      if (!encoder) {
        const auto& field = info.fields[index];
        encodeV6FloatColumn(
            base, info.point_step, field.offset, *field.resolution, n, streams, column_quantized, column_nan, out);
        continue;
      }
      encoder->reset();
      for (size_t i = 0; i < n; ++i) {
        encoder->encode(ConstBufferView(base + i * info.point_step, info.point_step), out);
      }
      encoder->flush(out);
    }

    // integer fields: the V5 adaptive sections
    adaptive.encodeChunk(base, info.point_step, n, info.compression_opt, out, mark_section);

    write_stage1_chunk(stage_buffer.size() - out.size(), section_starts);
    point_offset += n;
    points_left -= n;
    ++chunk_index;
  }
}

void BuildV6Decoders(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders, size_t& min_encoded_point_bytes) {
  decoders.clear();
  min_encoded_point_bytes = 0;
  for (size_t i = kV6GeometryFields; i < info.fields.size(); ++i) {
    if (isV6RegularField(info, i) && !isV6FloatColumn(info, i)) {
      decoders.push_back(CreateCompatibleDecoder(info, info.fields[i]));
    }
  }
}

namespace {

// Value of a quantized step. While q is exact in a float, float(q) * resolution gives the same bits (the
// double product of two floats is exact, and both round it once), so the double product costs nothing more.
// Defined for every q: V6 resolutions are below kV6MaxResolution, so |q * res| < 2^63 * 1e18 < FLT_MAX.
inline float v6Reconstruct(int64_t q, double res) {
  return static_cast<float>(static_cast<double>(q) * res);
}

// Reads one residual; returns false for the NaN marker. Unchecked: at least kMaxVarintBytes are readable.
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

struct V6AxisOut {
  uint8_t* dst = nullptr;  // null: kDecodeButSkipStore
  double res = 0.0;
};

// Decodes one axis value of a valid point.
template <V6Predictor P, bool Checked>
inline float v6DecodeValue(
    const uint8_t*& ptr, const uint8_t* end, int64_t* q, size_t i, size_t K, const V6AxisOut& axis) {
  const int64_t pred = v6Predict<P>(q, i, K);
  int64_t residual = 0;
  if (!v6ReadResidual<Checked>(ptr, end, residual)) {
    q[i] = pred;
    return std::numeric_limits<float>::quiet_NaN();
  }
  q[i] = static_cast<int64_t>(static_cast<uint64_t>(pred) + static_cast<uint64_t>(residual));
  return v6Reconstruct(q[i], axis.res);
}

// Decodes x, y and z together: one pass over the points, three stream pointers. The validity mask is
// read 64 points at a time, and a block of valid points skips the per-point test.
template <V6Predictor P>
void decodeV6Geometry(
    std::array<const uint8_t*, 3>& ptr, const std::array<const uint8_t*, 3>& end, size_t n, size_t K,
    const uint8_t* valid_bits, size_t mask_bytes, std::array<int64_t*, 3> q, size_t step,
    const std::array<V6AxisOut, 3>& axes, float invalid_value) {
  for (size_t block = 0; block < n; block += 64) {
    const size_t block_end = std::min(n, block + 64);
    uint64_t bits = ~uint64_t(0);
    if (valid_bits) {
      bits = 0;
      std::memcpy(&bits, valid_bits + block / 8, std::min<size_t>(8, mask_bytes - block / 8));
    }
    const size_t count = block_end - block;
    const bool all_valid =
        !valid_bits || (count == 64 ? bits == ~uint64_t(0) : (~bits & ((uint64_t(1) << count) - 1)) == 0);
    const size_t worst = count * kMaxVarintBytes;
    const bool fast = static_cast<size_t>(end[0] - ptr[0]) >= worst && static_cast<size_t>(end[1] - ptr[1]) >= worst &&
                      static_cast<size_t>(end[2] - ptr[2]) >= worst;
    auto run = [&](auto checked_tag) {
      constexpr bool Checked = decltype(checked_tag)::value;
      // locals, so that the stream pointers stay in registers
      const uint8_t* p0 = ptr[0];
      const uint8_t* p1 = ptr[1];
      const uint8_t* p2 = ptr[2];
      const uint8_t *e0 = end[0], *e1 = end[1], *e2 = end[2];
      int64_t *q0 = q[0], *q1 = q[1], *q2 = q[2];
      const V6AxisOut a0 = axes[0], a1 = axes[1], a2 = axes[2];
      for (size_t i = block; i < block_end; ++i) {
        if (!all_valid && !((bits >> (i - block)) & 1u)) {
          q0[i] = i >= 1 ? q0[i - 1] : 0;
          q1[i] = i >= 1 ? q1[i - 1] : 0;
          q2[i] = i >= 1 ? q2[i - 1] : 0;
          for (const V6AxisOut* a : {&a0, &a1, &a2}) {
            if (a->dst) {
              std::memcpy(a->dst + i * step, &invalid_value, sizeof(float));
            }
          }
          continue;
        }
        const float x = v6DecodeValue<P, Checked>(p0, e0, q0, i, K, a0);
        const float y = v6DecodeValue<P, Checked>(p1, e1, q1, i, K, a1);
        const float z = v6DecodeValue<P, Checked>(p2, e2, q2, i, K, a2);
        if (a0.dst) {
          std::memcpy(a0.dst + i * step, &x, sizeof(float));
        }
        if (a1.dst) {
          std::memcpy(a1.dst + i * step, &y, sizeof(float));
        }
        if (a2.dst) {
          std::memcpy(a2.dst + i * step, &z, sizeof(float));
        }
      }
      ptr[0] = p0;
      ptr[1] = p1;
      ptr[2] = p2;
    };
    if (fast) {
      run(std::false_type{});
    } else {
      run(std::true_type{});
    }
  }
}

// Decodes a single-stream V6 float column with the previous-value predictor.
void decodeV6FloatColumn(
    ConstBufferView& input, size_t n, int64_t* q, uint8_t* base, size_t step, uint32_t offset, float res) {
  const uint8_t* ptr = input.data();
  const uint8_t* const end = input.data() + input.size();
  V6AxisOut axis;
  axis.dst = offset == kDecodeButSkipStore ? nullptr : base + offset;
  axis.res = static_cast<double>(res);
  size_t i = 0;
  for (; i < n && static_cast<size_t>(end - ptr) >= kMaxVarintBytes; ++i) {
    const float value = v6DecodeValue<V6Predictor::Previous, false>(ptr, end, q, i, 0, axis);
    if (axis.dst) {
      std::memcpy(axis.dst + i * step, &value, sizeof(float));
    }
  }
  for (; i < n; ++i) {
    const float value = v6DecodeValue<V6Predictor::Previous, true>(ptr, end, q, i, 0, axis);
    if (axis.dst) {
      std::memcpy(axis.dst + i * step, &value, sizeof(float));
    }
  }
  input.trim_front(static_cast<size_t>(ptr - input.data()));
}

}  // namespace

void DecodeV6Stage1Chunk(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders, ConstBufferView& encoded_view,
    BufferView& output_buffer, size_t expected_points) {
  const size_t n = expected_points;
  if (n == 0) {
    throw std::runtime_error("V6 chunks require an expected point count");
  }
  const size_t step = info.point_step;
  if (output_buffer.size() < n * step) {
    throw std::runtime_error("Output buffer is too small to hold the decoded V6 data");
  }
  uint8_t* base = output_buffer.data();
  const V6Geometry geometry = makeV6Geometry(info);

  if (encoded_view.empty()) {
    throw std::runtime_error("V6: missing geometry mode");
  }
  const uint8_t mode = encoded_view.data()[0];
  encoded_view.trim_front(1);
  if (mode == static_cast<uint8_t>(V6GeometryMode::Raw)) {
    for (size_t a = 0; a < kV6GeometryFields; ++a) {
      if (encoded_view.size() < n * sizeof(float)) {
        throw std::runtime_error("V6: truncated raw geometry");
      }
      if (geometry.offset[a] != kDecodeButSkipStore) {
        for (size_t i = 0; i < n; ++i) {
          std::memcpy(base + i * step + geometry.offset[a], encoded_view.data() + i * sizeof(float), sizeof(float));
        }
      }
      encoded_view.trim_front(n * sizeof(float));
    }
  } else if (mode == static_cast<uint8_t>(V6GeometryMode::Predicted)) {
    if (encoded_view.size() < 2) {
      throw std::runtime_error("V6: truncated geometry header");
    }
    const uint8_t predictor_byte = encoded_view.data()[0];
    encoded_view.trim_front(1);
    if (predictor_byte > static_cast<uint8_t>(V6Predictor::SecondOrder)) {
      throw std::runtime_error("V6: unknown predictor");
    }
    const auto predictor = static_cast<V6Predictor>(predictor_byte);
    const uint64_t lag64 = readUVarint(encoded_view);
    const size_t lag = static_cast<size_t>(std::min<uint64_t>(lag64, n + 1));
    if (encoded_view.empty()) {
      throw std::runtime_error("V6: truncated geometry header");
    }
    const uint8_t mask_byte = encoded_view.data()[0];
    encoded_view.trim_front(1);
    if (mask_byte > static_cast<uint8_t>(V6MaskKind::Zero)) {
      throw std::runtime_error("V6: unknown mask kind");
    }
    const auto mask = static_cast<V6MaskKind>(mask_byte);
    const uint8_t* valid_bits = nullptr;
    if (mask != V6MaskKind::None) {
      const size_t mask_bytes = (n + 7) / 8;
      if (encoded_view.size() < mask_bytes) {
        throw std::runtime_error("V6: truncated validity mask");
      }
      valid_bits = encoded_view.data();
      encoded_view.trim_front(mask_bytes);
    }
    const float invalid_value = mask == V6MaskKind::NaN ? std::numeric_limits<float>::quiet_NaN() : 0.0f;
    const uint64_t x_size = readUVarint(encoded_view);
    const uint64_t y_size = readUVarint(encoded_view);
    if (x_size > encoded_view.size() || y_size > encoded_view.size() - x_size) {
      throw std::runtime_error("V6: geometry stream sizes exceed the chunk");
    }
    std::array<const uint8_t*, 3> ptr = {
        encoded_view.data(), encoded_view.data() + x_size, encoded_view.data() + x_size + y_size};
    const std::array<const uint8_t*, 3> end = {ptr[1], ptr[2], encoded_view.data() + encoded_view.size()};
    thread_local std::vector<int64_t> q;
    q.resize(3 * n);
    const std::array<int64_t*, 3> qa = {q.data(), q.data() + n, q.data() + 2 * n};
    std::array<V6AxisOut, 3> axes;
    for (size_t a = 0; a < kV6GeometryFields; ++a) {
      axes[a].dst = geometry.offset[a] == kDecodeButSkipStore ? nullptr : base + geometry.offset[a];
      axes[a].res = static_cast<double>(geometry.resolution[a]);
    }
    const size_t mask_bytes = valid_bits ? (n + 7) / 8 : 0;
    switch (predictor) {
      case V6Predictor::Previous:
        decodeV6Geometry<V6Predictor::Previous>(
            ptr, end, n, lag, valid_bits, mask_bytes, qa, step, axes, invalid_value);
        break;
      case V6Predictor::LagK:
        decodeV6Geometry<V6Predictor::LagK>(ptr, end, n, lag, valid_bits, mask_bytes, qa, step, axes, invalid_value);
        break;
      case V6Predictor::Median:
        decodeV6Geometry<V6Predictor::Median>(ptr, end, n, lag, valid_bits, mask_bytes, qa, step, axes, invalid_value);
        break;
      case V6Predictor::SecondOrder:
        decodeV6Geometry<V6Predictor::SecondOrder>(
            ptr, end, n, lag, valid_bits, mask_bytes, qa, step, axes, invalid_value);
        break;
    }
    if (ptr[0] != end[0] || ptr[1] != end[1]) {
      throw std::runtime_error("V6: geometry stream sizes do not match their content");
    }
    encoded_view.trim_front(static_cast<size_t>(ptr[2] - encoded_view.data()));
  } else {
    throw std::runtime_error("V6: unknown geometry mode");
  }

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
      decoder->decodePoints(encoded_view, base, step, n);
      continue;
    }
    const auto& field = info.fields[index];
    if (encoded_view.empty()) {
      throw std::runtime_error("V6: missing float column mode");
    }
    const uint8_t column_mode = encoded_view.data()[0];
    encoded_view.trim_front(1);
    if (column_mode == static_cast<uint8_t>(V6GeometryMode::Raw)) {
      if (encoded_view.size() < n * sizeof(float)) {
        throw std::runtime_error("V6: truncated raw float column");
      }
      if (field.offset != kDecodeButSkipStore) {
        for (size_t i = 0; i < n; ++i) {
          std::memcpy(base + i * step + field.offset, encoded_view.data() + i * sizeof(float), sizeof(float));
        }
      }
      encoded_view.trim_front(n * sizeof(float));
    } else if (column_mode == static_cast<uint8_t>(V6GeometryMode::Predicted)) {
      thread_local std::vector<int64_t> column;
      column.resize(n);
      decodeV6FloatColumn(encoded_view, n, column.data(), base, step, field.offset, *field.resolution);
    } else {
      throw std::runtime_error("V6: unknown float column mode");
    }
  }

  DecodeAdaptiveIntSections(info, encoded_view, base, step, n);
  if (!encoded_view.empty()) {
    throw std::runtime_error("V6 chunk has trailing bytes after decode");
  }
  output_buffer.trim_front(n * step);
}

}  // namespace Cloudini::detail
