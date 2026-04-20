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

#include "cloudini_lib/cloudini.hpp"

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
