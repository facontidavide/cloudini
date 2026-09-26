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

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include "cloudini_lib/cloudini.hpp"

using namespace Cloudini;

namespace {

// x, y, z, intensity, t (per column), ring, padding: an Ouster-like point
struct OusterPoint {
  float x, y, z;
  float intensity;
  uint32_t t;
  uint16_t ring;
  uint16_t reflectivity;
};

std::vector<PointField> ousterFields(float resolution) {
  return {
      {"x", offsetof(OusterPoint, x), FieldType::FLOAT32, resolution},
      {"y", offsetof(OusterPoint, y), FieldType::FLOAT32, resolution},
      {"z", offsetof(OusterPoint, z), FieldType::FLOAT32, resolution},
      {"intensity", offsetof(OusterPoint, intensity), FieldType::FLOAT32, 1.0f},
      {"t", offsetof(OusterPoint, t), FieldType::UINT32, std::nullopt},
      {"ring", offsetof(OusterPoint, ring), FieldType::UINT16, std::nullopt},
      {"reflectivity", offsetof(OusterPoint, reflectivity), FieldType::UINT16, std::nullopt}};
}

// Organized scan (rows = beams): no-return points are (0, 0, 0), as Ouster publishes them.
std::vector<OusterPoint> organizedScan(uint32_t width, uint32_t height, std::mt19937& rng) {
  std::uniform_real_distribution<float> noise(-0.01f, 0.01f);
  std::vector<OusterPoint> points(size_t(width) * height);
  for (uint32_t r = 0; r < height; ++r) {
    for (uint32_t c = 0; c < width; ++c) {
      auto& p = points[size_t(r) * width + c];
      const float az = 6.2831853f * float(c) / float(width);
      const float el = -0.3f + 0.6f * float(r) / float(height);
      const float range = 8.0f + 3.0f * std::sin(az * 3.0f) + noise(rng);
      const bool no_return = (rng() % 17) == 0;
      p.x = no_return ? 0.0f : range * std::cos(el) * std::cos(az);
      p.y = no_return ? 0.0f : range * std::cos(el) * std::sin(az);
      p.z = no_return ? 0.0f : range * std::sin(el);
      p.intensity = float(rng() % 300);
      p.t = c * 48828u;
      p.ring = uint16_t(r);
      p.reflectivity = uint16_t(rng() % 50);
    }
  }
  return points;
}

// Firing-order scan (unorganized, height 1): every laser of one firing, then the next firing.
std::vector<OusterPoint> firingOrderScan(size_t firings, size_t lasers, std::mt19937& rng) {
  std::uniform_real_distribution<float> noise(-0.005f, 0.005f);
  std::vector<OusterPoint> points(firings * lasers);
  for (size_t f = 0; f < firings; ++f) {
    for (size_t l = 0; l < lasers; ++l) {
      auto& p = points[f * lasers + l];
      const float az = 6.2831853f * float(f) / float(firings);
      const float el = -0.5f + float(l) / float(lasers);
      const float range = 5.0f + 20.0f * float(l) / float(lasers) + std::sin(az * 5.0f) + noise(rng);
      p.x = range * std::cos(el) * std::cos(az);
      p.y = range * std::cos(el) * std::sin(az);
      p.z = range * std::sin(el);
      p.intensity = float(rng() % 256);
      p.t = uint32_t(f * 55);
      p.ring = uint16_t(l);
      p.reflectivity = 0;
    }
  }
  return points;
}

EncodingInfo makeInfo(
    const std::vector<PointField>& fields, uint32_t width, uint32_t height, uint32_t step, uint8_t version,
    CompressionOption compression) {
  EncodingInfo info;
  info.fields = fields;
  info.width = width;
  info.height = height;
  info.point_step = step;
  info.version = version;
  info.compression_opt = compression;
  return info;
}

std::vector<uint8_t> encode(const EncodingInfo& info, const void* data, size_t size) {
  std::vector<uint8_t> out;
  PointcloudEncoder encoder(info);
  encoder.encode(ConstBufferView(static_cast<const uint8_t*>(data), size), out);
  return out;
}

std::vector<uint8_t> decode(const std::vector<uint8_t>& encoded, EncodingInfo* header_out = nullptr) {
  ConstBufferView view(encoded.data(), encoded.size());
  EncodingInfo header = DecodeHeader(view);
  if (header_out) {
    *header_out = header;
  }
  std::vector<uint8_t> out;
  PointcloudDecoder().decode(header, view, out);
  return out;
}

// x, y, z within half a resolution (NaN and exact zeros preserved); every other field identical to V5.
void expectGeometryAndFields(
    const std::vector<OusterPoint>& input, const std::vector<uint8_t>& v6, const std::vector<uint8_t>& v5,
    float resolution) {
  ASSERT_EQ(v6.size(), input.size() * sizeof(OusterPoint));
  ASSERT_EQ(v5.size(), v6.size());
  for (size_t i = 0; i < input.size(); ++i) {
    OusterPoint a, b;
    std::memcpy(&a, v6.data() + i * sizeof(OusterPoint), sizeof(OusterPoint));
    std::memcpy(&b, v5.data() + i * sizeof(OusterPoint), sizeof(OusterPoint));
    const float in[3] = {input[i].x, input[i].y, input[i].z};
    const float out[3] = {a.x, a.y, a.z};
    for (int k = 0; k < 3; ++k) {
      if (std::isnan(in[k])) {
        ASSERT_TRUE(std::isnan(out[k])) << "point " << i << " axis " << k;
      } else {
        ASSERT_NEAR(out[k], in[k], 0.5f * resolution * 1.001f + 1e-6f * std::fabs(in[k])) << "point " << i;
      }
    }
    ASSERT_EQ(a.intensity, b.intensity) << i;
    ASSERT_EQ(a.t, b.t) << i;
    ASSERT_EQ(a.ring, b.ring) << i;
    ASSERT_EQ(a.reflectivity, b.reflectivity) << i;
  }
}

}  // namespace

TEST(V6, OrganizedScanRoundTripAndSmaller) {
  std::mt19937 rng(1);
  const uint32_t width = 1024, height = 64;
  const auto points = organizedScan(width, height, rng);
  for (auto compression : {CompressionOption::NONE, CompressionOption::LZ4, CompressionOption::ZSTD}) {
    const auto fields = ousterFields(0.001f);
    const auto v5_info = makeInfo(fields, width, height, sizeof(OusterPoint), 5, compression);
    const auto v6_info = makeInfo(fields, width, height, sizeof(OusterPoint), 6, compression);
    const auto v5 = encode(v5_info, points.data(), points.size() * sizeof(OusterPoint));
    const auto v6 = encode(v6_info, points.data(), points.size() * sizeof(OusterPoint));
    EncodingInfo header;
    const auto decoded = decode(v6, &header);
    EXPECT_EQ(header.version, 6);
    expectGeometryAndFields(points, decoded, decode(v5), 0.001f);
    if (compression == CompressionOption::ZSTD) {
      EXPECT_LT(v6.size(), v5.size()) << "V6 " << v6.size() << " vs V5 " << v5.size();
    }
  }
}

TEST(V6, FiringOrderScanUsesTheLaserOneFiringBack) {
  std::mt19937 rng(2);
  const auto points = firingOrderScan(2000, 32, rng);  // 64000 points: two chunks
  const auto fields = ousterFields(0.001f);
  const uint32_t n = uint32_t(points.size());
  for (bool threads : {false, true}) {
    auto v5_info = makeInfo(fields, n, 1, sizeof(OusterPoint), 5, CompressionOption::ZSTD);
    auto v6_info = makeInfo(fields, n, 1, sizeof(OusterPoint), 6, CompressionOption::ZSTD);
    v5_info.use_threads = v6_info.use_threads = threads;
    const auto v5 = encode(v5_info, points.data(), points.size() * sizeof(OusterPoint));
    const auto v6 = encode(v6_info, points.data(), points.size() * sizeof(OusterPoint));
    expectGeometryAndFields(points, decode(v6), decode(v5), 0.001f);
    // predicting from the same laser one firing back must beat the previous point by a wide margin
    EXPECT_LT(double(v6.size()), 0.8 * double(v5.size())) << "V6 " << v6.size() << " vs V5 " << v5.size();
  }
}

TEST(V6, NaNPointsPartialNaNAndHugeValues) {
  std::mt19937 rng(3);
  auto points = firingOrderScan(300, 16, rng);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (size_t i = 0; i < points.size(); i += 7) {
    points[i].x = points[i].y = points[i].z = nan;  // no-return points published as NaN
  }
  points[5].y = nan;  // one NaN axis
  points[6].z = nan;
  const auto fields = ousterFields(0.01f);
  const uint32_t n = uint32_t(points.size());
  for (auto compression : {CompressionOption::NONE, CompressionOption::ZSTD}) {
    const auto v5 = decode(encode(
        makeInfo(fields, n, 1, sizeof(OusterPoint), 5, compression), points.data(),
        points.size() * sizeof(OusterPoint)));
    const auto v6 = decode(encode(
        makeInfo(fields, n, 1, sizeof(OusterPoint), 6, compression), points.data(),
        points.size() * sizeof(OusterPoint)));
    expectGeometryAndFields(points, v6, v5, 0.01f);
  }

  // a value too large for the integer streams: that chunk stores its geometry raw, bit-exact
  auto huge = points;
  huge[10].x = 3.0e30f;
  huge[11].y = std::numeric_limits<float>::infinity();
  const auto encoded = encode(
      makeInfo(fields, n, 1, sizeof(OusterPoint), 6, CompressionOption::ZSTD), huge.data(),
      huge.size() * sizeof(OusterPoint));
  const auto decoded = decode(encoded);
  for (size_t i = 0; i < huge.size(); ++i) {
    OusterPoint p;
    std::memcpy(&p, decoded.data() + i * sizeof(OusterPoint), sizeof(OusterPoint));
    ASSERT_EQ(std::memcmp(&p.x, &huge[i].x, 3 * sizeof(float)), 0) << i;
  }
}

TEST(V6, SmallCloudsAndMultiChunk) {
  std::mt19937 rng(4);
  const auto fields = ousterFields(0.001f);
  for (size_t count : {size_t(1), size_t(2), size_t(7), size_t(32768), size_t(32769), size_t(70001)}) {
    auto points = firingOrderScan(count / 32 + 1, 32, rng);
    points.resize(count);
    const uint32_t n = uint32_t(count);
    const auto v5 = decode(encode(
        makeInfo(fields, n, 1, sizeof(OusterPoint), 5, CompressionOption::ZSTD), points.data(),
        count * sizeof(OusterPoint)));
    const auto v6 = decode(encode(
        makeInfo(fields, n, 1, sizeof(OusterPoint), 6, CompressionOption::ZSTD), points.data(),
        count * sizeof(OusterPoint)));
    expectGeometryAndFields(points, v6, v5, 0.001f);
  }
}

TEST(V6, OtherLayoutsFallBackOrRoundTrip) {
  // x, y, z only; packed 26-byte point with a FLOAT64 timestamp; no resolution (not V6: falls back)
  struct Packed {
    float x, y, z, intensity;
    uint16_t ring;
    double timestamp;
  } __attribute__((packed));
  static_assert(sizeof(Packed) == 26);
  std::vector<Packed> points(5000);
  for (size_t i = 0; i < points.size(); ++i) {
    points[i] = {0.01f * float(i % 97), -0.02f * float(i % 31), 1.0f + 0.001f * float(i % 13),
                 float(i % 200) * 0.5f, uint16_t(i % 16),       1.7e9 + double(i) * 1e-5};
  }
  const std::vector<PointField> fields = {
      {"x", 0, FieldType::FLOAT32, 0.001f},          {"y", 4, FieldType::FLOAT32, 0.001f},
      {"z", 8, FieldType::FLOAT32, 0.001f},          {"intensity", 12, FieldType::FLOAT32, 0.5f},
      {"ring", 16, FieldType::UINT16, std::nullopt}, {"timestamp", 18, FieldType::FLOAT64, std::nullopt}};
  const uint32_t n = uint32_t(points.size());
  const auto v6 = decode(encode(
      makeInfo(fields, n, 1, sizeof(Packed), 6, CompressionOption::ZSTD), points.data(),
      points.size() * sizeof(Packed)));
  const auto v5 = decode(encode(
      makeInfo(fields, n, 1, sizeof(Packed), 5, CompressionOption::ZSTD), points.data(),
      points.size() * sizeof(Packed)));
  ASSERT_EQ(v6.size(), v5.size());
  for (size_t i = 0; i < points.size(); ++i) {
    Packed a, b;
    std::memcpy(&a, v6.data() + i * sizeof(Packed), sizeof(Packed));
    std::memcpy(&b, v5.data() + i * sizeof(Packed), sizeof(Packed));
    ASSERT_NEAR(a.x, points[i].x, 0.0006f);
    ASSERT_NEAR(a.z, points[i].z, 0.0006f);
    ASSERT_EQ(a.intensity, b.intensity);
    ASSERT_EQ(a.ring, b.ring);
    ASSERT_EQ(a.timestamp, b.timestamp);
  }

  // lossless x, y, z: V6 does not apply, the cloud is encoded with the V5 / V4 codec and still decodes
  auto lossless = fields;
  for (auto& f : lossless) {
    f.resolution.reset();
  }
  auto info = makeInfo(lossless, n, 1, sizeof(Packed), 6, CompressionOption::ZSTD);
  info.encoding_opt = EncodingOptions::LOSSLESS;
  const auto decoded = decode(encode(info, points.data(), points.size() * sizeof(Packed)));
  ASSERT_EQ(std::memcmp(decoded.data(), points.data(), decoded.size()), 0);
}

TEST(V6, TruncatedOrCorruptedPayloadThrows) {
  std::mt19937 rng(5);
  const auto points = organizedScan(256, 16, rng);
  const auto fields = ousterFields(0.001f);
  const auto encoded = encode(
      makeInfo(fields, 256, 16, sizeof(OusterPoint), 6, CompressionOption::NONE), points.data(),
      points.size() * sizeof(OusterPoint));
  ConstBufferView view(encoded.data(), encoded.size());
  const EncodingInfo header = DecodeHeader(view);
  const size_t header_size = encoded.size() - view.size();
  for (size_t len = header_size; len < encoded.size(); len += 3) {
    std::vector<uint8_t> truncated(encoded.begin(), encoded.begin() + std::ptrdiff_t(len));
    std::vector<uint8_t> out(points.size() * sizeof(OusterPoint));
    EXPECT_THROW(
        PointcloudDecoder().decode(
            header, ConstBufferView(truncated.data() + header_size, len - header_size),
            BufferView(out.data(), out.size())),
        std::runtime_error)
        << len;
  }
  std::mt19937 flip(6);
  for (int it = 0; it < 2000; ++it) {
    auto mutated = encoded;
    mutated[header_size + flip() % (mutated.size() - header_size)] = uint8_t(flip());
    std::vector<uint8_t> out(points.size() * sizeof(OusterPoint));
    try {
      PointcloudDecoder().decode(
          header, ConstBufferView(mutated.data() + header_size, mutated.size() - header_size),
          BufferView(out.data(), out.size()));
    } catch (const std::runtime_error&) {}
  }
}
