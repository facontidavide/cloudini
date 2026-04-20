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

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/field_decoder.hpp"
#include "cloudini_lib/field_encoder.hpp"

TEST(FieldEncoders, IntField) {
  const size_t kNumpoints = 100;
  std::vector<uint32_t> input_data(kNumpoints);
  std::vector<uint32_t> output_data(kNumpoints, 0);

  const size_t kBufferSize = kNumpoints * sizeof(uint32_t);

  // create a sequence of random numbers
  std::generate(input_data.begin(), input_data.end(), []() { return std::rand() % 1000; });

  using namespace Cloudini;

  std::vector<uint8_t> buffer(kNumpoints * sizeof(uint32_t));

  const int memory_offset = 0;
  FieldEncoderInt<uint32_t> encoder(memory_offset);
  FieldDecoderInt<uint32_t> decoder(memory_offset);

  //------------- Encode -------------
  {
    ConstBufferView input_buffer(input_data.data(), kBufferSize);
    BufferView buffer_data = {buffer.data(), buffer.size()};

    size_t encoded_size = 0;
    for (size_t i = 0; i < kNumpoints; ++i) {
      encoded_size += encoder.encode(input_buffer, buffer_data);
      input_buffer.trim_front(sizeof(uint32_t));
    }
    buffer.resize(encoded_size);

    std::cout << "Original size: " << kBufferSize << "   encoded size: " << encoded_size << std::endl;
  }

  //------------- Decode -------------
  {
    ConstBufferView buffer_data = {buffer.data(), buffer.size()};
    BufferView output_buffer(output_data.data(), kBufferSize);

    for (size_t i = 0; i < kNumpoints; ++i) {
      decoder.decode(buffer_data, output_buffer);
      ASSERT_EQ(input_data[i], output_data[i]) << "Mismatch at index " << i;
      output_buffer.trim_front(sizeof(uint32_t));
    }
  }
}

TEST(FieldEncoders, FloatLossy) {
  const size_t kNumpoints = 1000000;
  const float kResolution = 0.01F;

  std::vector<float> input_data(kNumpoints);
  std::vector<float> output_data(kNumpoints, 0.0F);

  const size_t kBufferSize = kNumpoints * sizeof(float);

  // create a sequence of random numbers
  std::generate(input_data.begin(), input_data.end(), []() { return 0.001 * static_cast<float>(std::rand() % 10000); });

  const auto nan_value = std::numeric_limits<float>::quiet_NaN();
  input_data[1] = nan_value;
  input_data[15] = nan_value;
  input_data[16] = nan_value;

  using namespace Cloudini;

  PointField field_info;
  field_info.name = "the_float";
  field_info.offset = 0;
  field_info.type = FieldType::FLOAT32;
  field_info.resolution = kResolution;

  std::vector<uint8_t> buffer(kNumpoints * sizeof(float));

  FieldEncoderFloat_Lossy encoder(0, kResolution);
  FieldDecoderFloat_Lossy decoder(0, kResolution);
  //------------- Encode -------------
  {
    ConstBufferView input_buffer(input_data.data(), kBufferSize);
    BufferView buffer_data = {buffer.data(), buffer.size()};

    size_t encoded_size = 0;
    for (size_t i = 0; i < kNumpoints; ++i) {
      encoded_size += encoder.encode(input_buffer, buffer_data);
      input_buffer.trim_front(sizeof(float));
    }
    buffer.resize(encoded_size);

    std::cout << "Original size: " << kBufferSize << "   encoded size: " << encoded_size << std::endl;
  }

  //------------- Decode -------------
  {
    ConstBufferView buffer_data = {buffer.data(), buffer.size()};
    BufferView output_buffer(output_data.data(), kBufferSize);

    const float kTolerance = static_cast<float>(kResolution * 1.0001);

    float max_difference = 0.0F;

    for (size_t i = 0; i < kNumpoints; ++i) {
      decoder.decode(buffer_data, output_buffer);
      output_buffer.trim_front(sizeof(float));

      auto diff = std::abs(input_data[i] - output_data[i]);
      max_difference = std::max(max_difference, diff);

      if (std::isnan(input_data[i])) {
        ASSERT_TRUE(std::isnan(output_data[i])) << "Mismatch at index " << i;
        continue;
      }
      ASSERT_NEAR(input_data[i], output_data[i], kTolerance) << "Mismatch at index " << i;
    }
    std::cout << "Max difference: " << max_difference << std::endl;
  }
}

namespace {

// Helper: round-trip a sequence of FloatType values through encoder/decoder,
// periodically flushing+resetting both at chunk boundaries to exercise the
// chunk-flush path (the classic bit-packer gotcha).
template <typename EncoderT, typename DecoderT, typename FloatType>
void runFieldRoundTrip(const std::vector<FloatType>& input, size_t chunk_points, size_t worst_case_bytes_per_value) {
  using namespace Cloudini;

  const size_t n = input.size();
  std::vector<uint8_t> buffer(std::max<size_t>(n * worst_case_bytes_per_value + 16, 64));
  BufferView buf_view(buffer.data(), buffer.size());

  EncoderT encoder(0);
  size_t encoded_bytes = 0;
  size_t points_in_chunk = 0;
  // Chunk boundaries: after every `chunk_points` we flush+reset the encoder,
  // emulating what PointcloudEncoder does at chunk boundaries.
  for (size_t i = 0; i < n; ++i) {
    ConstBufferView point_view(reinterpret_cast<const uint8_t*>(&input[i]), sizeof(FloatType));
    encoded_bytes += encoder.encode(point_view, buf_view);
    points_in_chunk++;
    if (points_in_chunk >= chunk_points || i + 1 == n) {
      encoded_bytes += encoder.flush(buf_view);
      encoder.reset();
      points_in_chunk = 0;
    }
  }

  // Decode
  std::vector<FloatType> output(n, FloatType{0});
  DecoderT decoder(0);
  ConstBufferView enc_view(buffer.data(), encoded_bytes);

  size_t chunk_remaining = 0;
  size_t idx = 0;
  while (idx < n) {
    if (chunk_remaining == 0) {
      chunk_remaining = std::min(chunk_points, n - idx);
      decoder.reset();
    }
    BufferView point_view(reinterpret_cast<uint8_t*>(&output[idx]), sizeof(FloatType));
    decoder.decode(enc_view, point_view);
    idx++;
    chunk_remaining--;
  }

  // Exact bit-for-bit equality for lossless
  for (size_t i = 0; i < n; ++i) {
    using IntT = std::conditional_t<std::is_same<FloatType, float>::value, uint32_t, uint64_t>;
    IntT a_bits, b_bits;
    std::memcpy(&a_bits, &input[i], sizeof(FloatType));
    std::memcpy(&b_bits, &output[i], sizeof(FloatType));
    ASSERT_EQ(a_bits, b_bits) << "Mismatch at index " << i << " input=" << input[i] << " output=" << output[i];
  }
}

}  // namespace

TEST(FieldEncoders, FloatXOR_RoundTrip_Float32) {
  // Multi-chunk test: ensures the chunk-boundary reset path is covered.
  const size_t kChunkPoints = 32 * 1024;            // must match PointcloudEncoder::POINTS_PER_CHUNK
  const size_t kNumpoints = kChunkPoints * 3 + 17;  // cross chunk boundary several times

  std::mt19937 rng(42);
  std::uniform_real_distribution<float> dist(-1000.0f, 1000.0f);
  std::vector<float> input(kNumpoints);
  for (auto& v : input) {
    v = dist(rng);
  }

  using namespace Cloudini;
  runFieldRoundTrip<FieldEncoderFloat_XOR<float>, FieldDecoderFloat_XOR<float>, float>(
      input, kChunkPoints, sizeof(float));
}

TEST(FieldEncoders, FloatXOR_RoundTrip_Float64) {
  const size_t kChunkPoints = 32 * 1024;
  const size_t kNumpoints = kChunkPoints * 2 + 99;

  std::mt19937 rng(123);
  std::uniform_real_distribution<double> dist(-1e6, 1e6);
  std::vector<double> input(kNumpoints);
  for (auto& v : input) {
    v = dist(rng);
  }

  using namespace Cloudini;
  runFieldRoundTrip<FieldEncoderFloat_XOR<double>, FieldDecoderFloat_XOR<double>, double>(
      input, kChunkPoints, sizeof(double));
}

TEST(FieldEncoders, FloatGorilla_RoundTrip_Float32) {
  const size_t kChunkPoints = 32 * 1024;
  const size_t kNumpoints = kChunkPoints * 3 + 17;

  std::mt19937 rng(7);
  std::uniform_real_distribution<float> dist(-1000.0f, 1000.0f);
  std::vector<float> input(kNumpoints);
  for (auto& v : input) {
    v = dist(rng);
  }
  // Also include some repeated values to exercise the "same as prev" 1-bit path.
  for (size_t i = 100; i < 110; ++i) {
    input[i] = 3.14159f;
  }
  // And values near chunk boundary
  input[kChunkPoints - 1] = 1.0f;
  input[kChunkPoints] = 1.0f;
  input[kChunkPoints + 1] = 2.0f;

  using namespace Cloudini;
  runFieldRoundTrip<FieldEncoderFloat_Gorilla<float>, FieldDecoderFloat_Gorilla<float>, float>(input, kChunkPoints, 7);
}

TEST(FieldEncoders, FloatGorilla_RoundTrip_Float64) {
  const size_t kChunkPoints = 32 * 1024;
  const size_t kNumpoints = kChunkPoints * 2 + 123;

  std::mt19937 rng(99);
  std::uniform_real_distribution<double> dist(-1e6, 1e6);
  std::vector<double> input(kNumpoints);
  for (auto& v : input) {
    v = dist(rng);
  }
  for (size_t i = 200; i < 220; ++i) {
    input[i] = 2.718281828;
  }
  input[kChunkPoints - 1] = -0.5;
  input[kChunkPoints] = -0.5;

  using namespace Cloudini;
  runFieldRoundTrip<FieldEncoderFloat_Gorilla<double>, FieldDecoderFloat_Gorilla<double>, double>(
      input, kChunkPoints, 11);
}

TEST(FieldEncoders, FloatGorilla_EdgeCases_Float32) {
  // Deterministic edge cases: first value, zero, repeats, big jump, near-zero.
  std::vector<float> input = {
      0.0f,        // first value
      0.0f,        // same
      0.0f,        // same
      1.0f,        // big change
      1.0000001f,  // small change - tests window reuse
      1.0f,        // back
      1e-20f,      // tiny
      1e20f,       // huge
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
  };
  using namespace Cloudini;
  runFieldRoundTrip<FieldEncoderFloat_Gorilla<float>, FieldDecoderFloat_Gorilla<float>, float>(
      input, /*chunk_points=*/input.size() + 1, 7);
}

// Full PointcloudEncoder/Decoder round-trip in LOSSLESS mode, > POINTS_PER_CHUNK points,
// covers the full chunk-flush integration.
TEST(FieldEncoders, PointcloudLossless_Gorilla_MultiChunk) {
  using namespace Cloudini;

  struct PointXYZI {
    float x = 0;
    float y = 0;
    float z = 0;
    float i = 0;
  };

  const size_t kChunkPoints = 32 * 1024;
  const size_t kNumpoints = kChunkPoints * 2 + 777;  // multi-chunk

  std::mt19937 rng(2025);
  std::uniform_real_distribution<float> dist_pos(-500.0f, 500.0f);
  std::uniform_real_distribution<float> dist_i(0.0f, 255.0f);

  std::vector<PointXYZI> input(kNumpoints);
  for (auto& p : input) {
    p.x = dist_pos(rng);
    p.y = dist_pos(rng);
    p.z = dist_pos(rng);
    p.i = dist_i(rng);
  }

  EncodingInfo info;
  info.width = kNumpoints;
  info.height = 1;
  info.point_step = sizeof(PointXYZI);
  info.encoding_opt = EncodingOptions::LOSSLESS;
  info.compression_opt = CompressionOption::ZSTD;
  info.fields.push_back({"x", 0, FieldType::FLOAT32, {}});
  info.fields.push_back({"y", 4, FieldType::FLOAT32, {}});
  info.fields.push_back({"z", 8, FieldType::FLOAT32, {}});
  info.fields.push_back({"intensity", 12, FieldType::FLOAT32, {}});

  std::vector<uint8_t> compressed;
  {
    PointcloudEncoder encoder(info);
    ConstBufferView in_view(reinterpret_cast<const uint8_t*>(input.data()), input.size() * sizeof(PointXYZI));
    encoder.encode(in_view, compressed);
  }

  ConstBufferView comp_view(compressed.data(), compressed.size());
  auto decoded_info = DecodeHeader(comp_view);
  ASSERT_EQ(decoded_info.encoding_opt, EncodingOptions::LOSSLESS);

  std::vector<PointXYZI> output(kNumpoints);
  {
    PointcloudDecoder decoder;
    BufferView out_view(reinterpret_cast<uint8_t*>(output.data()), output.size() * sizeof(PointXYZI));
    decoder.decode(decoded_info, comp_view, out_view);
  }

  // Bit-exact equality for lossless.
  for (size_t i = 0; i < kNumpoints; ++i) {
    uint32_t a, b;
    std::memcpy(&a, &input[i].x, 4);
    std::memcpy(&b, &output[i].x, 4);
    ASSERT_EQ(a, b) << "x at " << i;
    std::memcpy(&a, &input[i].y, 4);
    std::memcpy(&b, &output[i].y, 4);
    ASSERT_EQ(a, b) << "y at " << i;
    std::memcpy(&a, &input[i].z, 4);
    std::memcpy(&b, &output[i].z, 4);
    ASSERT_EQ(a, b) << "z at " << i;
    std::memcpy(&a, &input[i].i, 4);
    std::memcpy(&b, &output[i].i, 4);
    ASSERT_EQ(a, b) << "intensity at " << i;
  }
}

// TEST(FieldEncoders, XYZLossy) {
//   const size_t kNumpoints = 1000000;
//   const double kResolution = 0.01F;

//   struct PointXYZ {
//     float x = 0;
//     float y = 0;
//     float z = 0;
//   };

//   std::vector<PointXYZ> input_data(kNumpoints);
//   std::vector<PointXYZ> output_data(kNumpoints);

//   const size_t kBufferSize = kNumpoints * sizeof(PointXYZ);

//   // create a sequence of random numbers
//   std::generate(input_data.begin(), input_data.end(), []() -> PointXYZ {
//     return {
//         0.001F * static_cast<float>(std::rand() % 10000),  //
//         0.001F * static_cast<float>(std::rand() % 10000),  //
//         0.001F * static_cast<float>(std::rand() % 10000)};
//   });

//   using namespace Cloudini;

//   PointField field_info;
//   field_info.name = "the_float";
//   field_info.offset = 0;
//   field_info.type = FieldType::FLOAT32;
//   field_info.resolution = kResolution;

//   std::vector<uint8_t> buffer(kNumpoints * sizeof(PointXYZ));

//   FieldEncoderFloatN_Lossy encoder(sizeof(PointXYZ), kResolution);
//   FieldDecoderXYZ_Lossy decoder(sizeof(PointXYZ), kResolution);
//   //------------- Encode -------------
//   {
//     ConstBufferView input_buffer(input_data.data(), kBufferSize);
//     BufferView buffer_data = {buffer.data(), buffer.size()};

//     size_t encoded_size = 0;
//     for (size_t i = 0; i < kNumpoints; ++i) {
//       encoded_size += encoder.encode(input_buffer, buffer_data);
//     }
//     buffer.resize(encoded_size);
//     std::cout << "Original size: " << kBufferSize << "   encoded size: " << encoded_size << std::endl;
//   }
//   //------------- Decode -------------
//   {
//     ConstBufferView buffer_data = {buffer.data(), buffer.size()};
//     BufferView output_buffer(output_data.data(), kBufferSize);

//     const float kTolerance = static_cast<float>(kResolution * 1.0001);

//     for (size_t i = 0; i < kNumpoints; ++i) {
//       decoder.decode(buffer_data, output_buffer);
//       ASSERT_NEAR(input_data[i].x, output_data[i].x, kTolerance) << "Mismatch at index " << i;
//       ASSERT_NEAR(input_data[i].y, output_data[i].y, kTolerance) << "Mismatch at index " << i;
//       ASSERT_NEAR(input_data[i].z, output_data[i].z, kTolerance) << "Mismatch at index " << i;
//     }
//   }
// }

// Regression guard for the Gorilla narrowing: when info.version = 3, a FLOAT64
// lossless field MUST go through FieldEncoderFloat_XOR (raw 8 bytes per value),
// not Gorilla. A v4 encode of the same data uses Gorilla and produces a
// different byte stream. This locks the dispatch narrowing in place.
TEST(FieldEncoders, Gorilla_DoesNotActivateForV3) {
  using namespace Cloudini;

  const size_t n = 1024;
  std::vector<double> input(n);
  // Monotonic-ish timestamps: Gorilla's best case (huge trailing-zero runs in XOR).
  // If Gorilla were wrongly activated on v3, the v3 output would be much
  // smaller than raw XOR bytes and would MATCH the v4 output.
  for (size_t i = 0; i < n; ++i) {
    input[i] = 1700000000.0 + 1e-6 * static_cast<double>(i);
  }

  auto make_info = [n](uint8_t version) {
    EncodingInfo info;
    info.version = version;
    info.width = static_cast<uint32_t>(n);
    info.height = 1;
    info.point_step = sizeof(double);
    info.encoding_opt = EncodingOptions::LOSSLESS;
    info.compression_opt = CompressionOption::NONE;  // inspect raw stage-1 bytes
    info.use_threads = false;
    info.fields.push_back({"v", 0, FieldType::FLOAT64, std::nullopt});
    return info;
  };

  std::vector<uint8_t> out_v3, out_v4;
  {
    auto info3 = make_info(3);
    PointcloudEncoder enc3(info3);
    ConstBufferView in(reinterpret_cast<const uint8_t*>(input.data()), input.size() * sizeof(double));
    enc3.encode(in, out_v3);
  }
  {
    auto info4 = make_info(4);
    PointcloudEncoder enc4(info4);
    ConstBufferView in(reinterpret_cast<const uint8_t*>(input.data()), input.size() * sizeof(double));
    enc4.encode(in, out_v4);
  }

  // v3 magic must start with CLOUDINI_V03; v4 with CLOUDINI_V04.
  ASSERT_GE(out_v3.size(), 12u);
  ASSERT_GE(out_v4.size(), 12u);
  EXPECT_EQ(std::string(reinterpret_cast<const char*>(out_v3.data()), 12), "CLOUDINI_V03");
  EXPECT_EQ(std::string(reinterpret_cast<const char*>(out_v4.data()), 12), "CLOUDINI_V04");

  // The byte streams must differ: v3 uses raw 8-byte XOR residuals, v4 uses
  // bit-packed Gorilla. For monotonic timestamps Gorilla is substantially
  // smaller than XOR — so out_v4.size() must be STRICTLY less than out_v3.size().
  EXPECT_LT(out_v4.size(), out_v3.size())
      << "Gorilla (v4) should compress monotonic FLOAT64 better than plain XOR (v3).";

  // Both must still round-trip bit-exactly.
  for (auto& blob : {std::cref(out_v3), std::cref(out_v4)}) {
    ConstBufferView view(blob.get().data(), blob.get().size());
    auto info_dec = DecodeHeader(view);
    std::vector<double> output(n, 0.0);
    PointcloudDecoder dec;
    BufferView out_view(reinterpret_cast<uint8_t*>(output.data()), output.size() * sizeof(double));
    dec.decode(info_dec, view, out_view);
    for (size_t i = 0; i < n; ++i) {
      uint64_t a, b;
      std::memcpy(&a, &input[i], sizeof(double));
      std::memcpy(&b, &output[i], sizeof(double));
      ASSERT_EQ(a, b) << "Bit mismatch at " << i << " (version " << static_cast<int>(info_dec.version) << ")";
    }
  }
}
