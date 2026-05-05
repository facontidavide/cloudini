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

#include <cstddef>
#include <cstdint>
#include <vector>

#include "cloudini_lib/cloudini.hpp"

namespace {

struct VersionPoint {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
  uint16_t ring = 0;
  uint32_t time = 0;
};

Cloudini::EncodingInfo makeVersionedLossyInfo(size_t points) {
  using namespace Cloudini;
  EncodingInfo info;
  info.width = static_cast<uint32_t>(points);
  info.height = 1;
  info.point_step = sizeof(VersionPoint);
  info.encoding_opt = EncodingOptions::LOSSY;
  info.compression_opt = CompressionOption::NONE;
  info.use_threads = false;
  info.fields.push_back({"x", offsetof(VersionPoint, x), FieldType::FLOAT32, 0.001F});
  info.fields.push_back({"y", offsetof(VersionPoint, y), FieldType::FLOAT32, 0.001F});
  info.fields.push_back({"z", offsetof(VersionPoint, z), FieldType::FLOAT32, 0.001F});
  info.fields.push_back({"intensity", offsetof(VersionPoint, intensity), FieldType::FLOAT32, 0.001F});
  info.fields.push_back({"ring", offsetof(VersionPoint, ring), FieldType::UINT16, std::nullopt});
  info.fields.push_back({"time", offsetof(VersionPoint, time), FieldType::UINT32, std::nullopt});
  return info;
}

std::vector<VersionPoint> makeVersionedPoints(size_t points) {
  std::vector<VersionPoint> data(points);
  for (size_t i = 0; i < data.size(); ++i) {
    auto& point = data[i];
    point.x = 0.001F * static_cast<float>(i);
    point.y = -0.002F * static_cast<float>(i % 1000);
    point.z = 1.0F + 0.003F * static_cast<float>(i % 257);
    point.intensity = 0.1F * static_cast<float>(i % 32);
    point.ring = static_cast<uint16_t>(i % 128);
    point.time = static_cast<uint32_t>(1000 + (i % 7) * 10);
  }
  return data;
}

std::vector<uint8_t> encodeVersionedPoints(
    const Cloudini::EncodingInfo& info, const std::vector<VersionPoint>& points) {
  Cloudini::PointcloudEncoder encoder(info);
  Cloudini::ConstBufferView in_view(
      reinterpret_cast<const uint8_t*>(points.data()), points.size() * sizeof(VersionPoint));
  std::vector<uint8_t> encoded;
  encoder.encode(in_view, encoded);
  return encoded;
}

void expectVersionedRoundTrip(
    const Cloudini::EncodingInfo& expected_info,
    const std::vector<VersionPoint>& input,
    const std::vector<uint8_t>& encoded) {
  Cloudini::ConstBufferView encoded_view(encoded.data(), encoded.size());
  const Cloudini::EncodingInfo decoded_info = Cloudini::DecodeHeader(encoded_view);
  ASSERT_EQ(decoded_info.version, expected_info.version);
  ASSERT_EQ(decoded_info.encoding_opt, expected_info.encoding_opt);
  ASSERT_EQ(decoded_info.compression_opt, expected_info.compression_opt);
  ASSERT_EQ(decoded_info.fields, expected_info.fields);

  std::vector<VersionPoint> output(input.size());
  Cloudini::PointcloudDecoder decoder;
  Cloudini::BufferView out_view(reinterpret_cast<uint8_t*>(output.data()), output.size() * sizeof(VersionPoint));
  decoder.decode(decoded_info, encoded_view, out_view);

  constexpr float kTolerance = 0.0011F;
  for (size_t i = 0; i < input.size(); ++i) {
    ASSERT_NEAR(input[i].x, output[i].x, kTolerance) << "x @" << i;
    ASSERT_NEAR(input[i].y, output[i].y, kTolerance) << "y @" << i;
    ASSERT_NEAR(input[i].z, output[i].z, kTolerance) << "z @" << i;
    ASSERT_NEAR(input[i].intensity, output[i].intensity, kTolerance) << "intensity @" << i;
    ASSERT_EQ(input[i].ring, output[i].ring) << "ring @" << i;
    ASSERT_EQ(input[i].time, output[i].time) << "time @" << i;
  }
}

}  // namespace

TEST(Cloudini, Header) {
  using namespace Cloudini;

  EncodingInfo header;
  header.width = 10;
  header.height = 20;
  header.point_step = sizeof(float) * 4;
  header.encoding_opt = EncodingOptions::LOSSY;
  header.compression_opt = CompressionOption::ZSTD;

  header.fields.push_back({"x", 0, FieldType::FLOAT32, 0.01});
  header.fields.push_back({"y", 4, FieldType::FLOAT32, 0.01});
  header.fields.push_back({"z", 8, FieldType::FLOAT32, 0.01});
  header.fields.push_back({"intensity", 12, FieldType::FLOAT32, 0.01});

  std::vector<uint8_t> buffer;
  EncodeHeader(header, buffer);

  ConstBufferView input(buffer.data(), buffer.size());
  auto decoded_header = DecodeHeader(input);

  ASSERT_EQ(decoded_header.width, header.width);
  ASSERT_EQ(decoded_header.height, header.height);
  ASSERT_EQ(decoded_header.point_step, header.point_step);
  ASSERT_EQ(decoded_header.encoding_opt, header.encoding_opt);
  ASSERT_EQ(decoded_header.compression_opt, header.compression_opt);
  ASSERT_EQ(decoded_header.fields.size(), header.fields.size());
  for (size_t i = 0; i < header.fields.size(); ++i) {
    ASSERT_EQ(decoded_header.fields[i].name, header.fields[i].name);
    ASSERT_EQ(decoded_header.fields[i].offset, header.fields[i].offset);
    ASSERT_EQ(decoded_header.fields[i].type, header.fields[i].type);
    ASSERT_EQ(decoded_header.fields[i].resolution, header.fields[i].resolution);
  }
}

TEST(Cloudini, DefaultV5AndExplicitV4RoundTrip) {
  using namespace Cloudini;

  const size_t kPoints = 4096 + 17;
  const std::vector<VersionPoint> points = makeVersionedPoints(kPoints);

  EncodingInfo default_info = makeVersionedLossyInfo(kPoints);
  ASSERT_EQ(default_info.version, kEncodingVersion);
  const std::vector<uint8_t> default_encoded = encodeVersionedPoints(default_info, points);
  ASSERT_GE(default_encoded.size(), 12u);
  EXPECT_EQ(std::string(reinterpret_cast<const char*>(default_encoded.data()), 12), "CLOUDINI_V05");
  expectVersionedRoundTrip(default_info, points, default_encoded);

  EncodingInfo v4_info = makeVersionedLossyInfo(kPoints);
  v4_info.version = 4;
  const std::vector<uint8_t> v4_encoded = encodeVersionedPoints(v4_info, points);
  ASSERT_GE(v4_encoded.size(), 12u);
  EXPECT_EQ(std::string(reinterpret_cast<const char*>(v4_encoded.data()), 12), "CLOUDINI_V04");
  expectVersionedRoundTrip(v4_info, points, v4_encoded);

  EXPECT_NE(default_encoded, v4_encoded);
}

TEST(Cloudini, HeaderTruncatedInput) {
  using namespace Cloudini;

  std::vector<uint8_t> buffer = {'C', 'L', 'O', 'U'};
  ConstBufferView input(buffer.data(), buffer.size());
  EXPECT_THROW(DecodeHeader(input), std::runtime_error);
}

TEST(Cloudini, DecodeV3_FromLegacyEncoder) {
  using namespace Cloudini;

  // Backward-compat contract: files written with the v3 wire format must still
  // decode correctly with the current (v4-capable) library. We simulate v3 by
  // setting info.version = 3 on the encoder. EncodeHeader honors this to write
  // a "03" magic header, and the dispatch code selects the v3 encoders
  // (FieldEncoderFloat_XOR for FLOAT64 lossless, no Gorilla).
  struct Point {
    float x, y, z;
    double stamp;
  };
  static_assert(sizeof(Point) == 24, "unexpected layout");

  const size_t n = 64 * 1024 + 7;  // multi-chunk (POINTS_PER_CHUNK = 32K)
  std::vector<Point> input(n);
  for (size_t i = 0; i < n; ++i) {
    input[i].x = 0.01f * static_cast<float>(i);
    input[i].y = -0.02f * static_cast<float>(i) + 0.5f;
    input[i].z = 0.001f * static_cast<float>(i) - 0.25f;
    input[i].stamp = 1700000000.0 + 0.000001 * static_cast<double>(i);  // monotonic timestamp
  }

  EncodingInfo info;
  info.version = 3;  // force v3 wire format
  info.width = static_cast<uint32_t>(n);
  info.height = 1;
  info.point_step = sizeof(Point);
  info.encoding_opt = EncodingOptions::LOSSY;
  info.compression_opt = CompressionOption::ZSTD;
  info.fields.push_back({"x", 0, FieldType::FLOAT32, 0.001f});
  info.fields.push_back({"y", 4, FieldType::FLOAT32, 0.001f});
  info.fields.push_back({"z", 8, FieldType::FLOAT32, 0.001f});
  info.fields.push_back({"stamp", 16, FieldType::FLOAT64, std::nullopt});  // lossless

  std::vector<uint8_t> compressed;
  {
    PointcloudEncoder encoder(info);
    ConstBufferView in_view(reinterpret_cast<const uint8_t*>(input.data()), input.size() * sizeof(Point));
    encoder.encode(in_view, compressed);
  }

  // Verify the written magic is "CLOUDINI_V03" (v3), not v4.
  ASSERT_GE(compressed.size(), 12u);
  ASSERT_EQ(std::string(reinterpret_cast<const char*>(compressed.data()), 12), "CLOUDINI_V03");

  ConstBufferView compressed_view(compressed.data(), compressed.size());
  const auto decoded_info = DecodeHeader(compressed_view);
  ASSERT_EQ(decoded_info.version, 3);

  std::vector<Point> output(n);
  {
    PointcloudDecoder decoder;
    BufferView out_view(reinterpret_cast<uint8_t*>(output.data()), output.size() * sizeof(Point));
    decoder.decode(decoded_info, compressed_view, out_view);
  }

  const float tol = 0.001f * 1.01f;
  for (size_t i = 0; i < n; ++i) {
    ASSERT_NEAR(input[i].x, output[i].x, tol) << "x @" << i;
    ASSERT_NEAR(input[i].y, output[i].y, tol) << "y @" << i;
    ASSERT_NEAR(input[i].z, output[i].z, tol) << "z @" << i;
    // stamp is LOSSLESS — expect bit-exact via XOR path
    uint64_t a, b;
    std::memcpy(&a, &input[i].stamp, sizeof(double));
    std::memcpy(&b, &output[i].stamp, sizeof(double));
    ASSERT_EQ(a, b) << "stamp @" << i;
  }
}

TEST(Cloudini, HeaderMissingYamlTerminator) {
  using namespace Cloudini;

  EncodingInfo header;
  header.width = 1;
  header.height = 1;
  header.point_step = sizeof(float) * 3;
  header.encoding_opt = EncodingOptions::LOSSY;
  header.compression_opt = CompressionOption::ZSTD;
  header.fields.push_back({"x", 0, FieldType::FLOAT32, 0.01F});
  header.fields.push_back({"y", 4, FieldType::FLOAT32, 0.01F});
  header.fields.push_back({"z", 8, FieldType::FLOAT32, 0.01F});

  std::vector<uint8_t> buffer;
  EncodeHeader(header, buffer);
  buffer.pop_back();  // remove YAML null terminator

  ConstBufferView input(buffer.data(), buffer.size());
  EXPECT_THROW(DecodeHeader(input), std::runtime_error);
}
