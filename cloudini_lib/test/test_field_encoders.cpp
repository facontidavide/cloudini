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
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <stdexcept>

#if defined(__unix__) || defined(__APPLE__)
#include <sys/mman.h>
#include <unistd.h>
#endif

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

TEST(FieldEncoders, DecodeVarintRejectsTruncatedInputBeforeReadingPastBound) {
  using namespace Cloudini;

  const std::array<uint8_t, 2> bytes = {0x80u, 0x00u};
  int64_t value = 0;

  EXPECT_THROW(
      {
        // max_size intentionally exposes only the first continuation byte.
        // A correct decoder must reject this without reading bytes[1].
        (void)decodeVarint(bytes.data(), 1, value);
      },
      std::runtime_error);
}

namespace {

// Oracle: the pre-optimization, loop-only decodeVarint. Kept here verbatim so
// the optimized fast-path implementation can be differentially compared against
// it. Any divergence (value, byte count, or throw/no-throw) is a regression.
size_t decodeVarintOracle(const uint8_t* buf, size_t max_size, int64_t& val) {
  if (max_size == 0) {
    throw std::runtime_error("decodeVarint: empty input");
  }
  uint64_t uval = 0;
  uint8_t shift = 0;
  const uint8_t* ptr = buf;
  while (true) {
    if (static_cast<size_t>(ptr - buf) >= max_size) {
      throw std::runtime_error("decodeVarint: truncated input");
    }
    uint8_t byte = *ptr;
    ptr++;
    const uint8_t payload = byte & 0x7f;
    if (shift >= 64 || (shift == 63 && payload > 1)) {
      throw std::runtime_error("decodeVarint: value overflow");
    }
    uval |= (static_cast<uint64_t>(payload) << shift);
    if ((byte & 0x80) == 0) {
      break;
    }
    if (shift >= 63) {
      throw std::runtime_error("decodeVarint: value overflow");
    }
    shift = static_cast<uint8_t>(shift + 7);
  }
  if (uval == 0) {
    throw std::runtime_error("decodeVarint: unexpected NaN marker");
  }
  uval--;
  val = static_cast<int64_t>((uval >> 1) ^ static_cast<uint64_t>(-(static_cast<int64_t>(uval & 1))));
  return static_cast<size_t>(ptr - buf);
}

// Compare the optimized decodeVarint against the oracle for one (buf, max_size)
// case: both must throw, or both must return identical (count, value).
void expectVarintMatchesOracle(const uint8_t* buf, size_t max_size) {
  int64_t opt_val = 0;
  size_t opt_count = 0;
  bool opt_threw = false;
  try {
    opt_count = Cloudini::decodeVarint(buf, max_size, opt_val);
  } catch (const std::exception&) {
    opt_threw = true;
  }

  int64_t ref_val = 0;
  size_t ref_count = 0;
  bool ref_threw = false;
  try {
    ref_count = decodeVarintOracle(buf, max_size, ref_val);
  } catch (const std::exception&) {
    ref_threw = true;
  }

  ASSERT_EQ(opt_threw, ref_threw) << "throw mismatch at max_size=" << max_size;
  if (!opt_threw) {
    ASSERT_EQ(opt_count, ref_count) << "count mismatch at max_size=" << max_size;
    ASSERT_EQ(opt_val, ref_val) << "value mismatch at max_size=" << max_size;
  }
}

}  // namespace

TEST(FieldEncoders, DecodeVarintMatchesOracleExhaustiveAndRandom) {
  // Exhaustive over all 1- and 2-byte prefixes (the new fast paths and their
  // boundary with the general path) with every truncation length in [0, len].
  std::array<uint8_t, 16> buf{};
  for (int b0 = 0; b0 < 256; ++b0) {
    buf[0] = static_cast<uint8_t>(b0);
    for (size_t ms = 0; ms <= 1; ++ms) {
      expectVarintMatchesOracle(buf.data(), ms);
    }
    for (int b1 = 0; b1 < 256; ++b1) {
      buf[1] = static_cast<uint8_t>(b1);
      for (size_t ms = 0; ms <= 2; ++ms) {
        expectVarintMatchesOracle(buf.data(), ms);
      }
      // 3-byte prefixes: sample b2 at the byte-value boundaries (the general
      // path here is the original code verbatim, so full enumeration is
      // unnecessary; the randomized sweep below covers the interior).
      for (uint8_t b2 : {0x00u, 0x01u, 0x7eu, 0x7fu, 0x80u, 0x81u, 0xfeu, 0xffu}) {
        buf[2] = b2;
        expectVarintMatchesOracle(buf.data(), 3);
      }
    }
  }

  // Randomized sweep over the general (3+ byte) path and all truncation
  // lengths, including malformed all-continuation and overflow-edge varints.
  // The 1- and 2-byte fast paths are already proven exhaustively above and the
  // 3+ byte path is the original code verbatim, so this sweep is supplementary
  // coverage of the interior; 200k keeps the ASan/Debug ctest run fast.
  std::mt19937_64 rng(0xC10D1217ULL);
  for (int iter = 0; iter < 200'000; ++iter) {
    const size_t len = 1 + (rng() % 12);  // up to 12 bytes (varint64 worst case is 10)
    for (size_t i = 0; i < len; ++i) {
      // Bias toward continuation bytes so we exercise long/overflowing varints.
      const uint32_t r = static_cast<uint32_t>(rng());
      uint8_t byte = static_cast<uint8_t>(r);
      if ((r >> 8) & 1) {
        byte |= 0x80u;  // force continuation ~50% of the time
      }
      buf[i] = byte;
    }
    const size_t max_size = rng() % (len + 1);  // 0 .. len
    expectVarintMatchesOracle(buf.data(), max_size);
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

Cloudini::EncodingInfo makeV5IntOnlyInfo(
    size_t points, Cloudini::FieldType type, Cloudini::CompressionOption compression) {
  using namespace Cloudini;
  EncodingInfo info;
  info.version = 5;
  info.width = static_cast<uint32_t>(points);
  info.height = 1;
  info.point_step = static_cast<uint32_t>(SizeOf(type));
  info.encoding_opt = EncodingOptions::LOSSY;
  info.compression_opt = compression;
  info.use_threads = false;
  info.fields.push_back({"value", 0, type, std::nullopt});
  return info;
}

template <typename IntType>
std::vector<uint8_t> encodeV5IntOnly(
    const std::vector<IntType>& values, Cloudini::FieldType type, Cloudini::CompressionOption compression) {
  using namespace Cloudini;
  const EncodingInfo info = makeV5IntOnlyInfo(values.size(), type, compression);
  PointcloudEncoder encoder(info);
  ConstBufferView in_view(reinterpret_cast<const uint8_t*>(values.data()), values.size() * sizeof(IntType));
  std::vector<uint8_t> encoded;
  encoder.encode(in_view, encoded);
  return encoded;
}

template <typename IntType>
void expectV5IntOnlyRoundTrip(
    const std::vector<IntType>& values, Cloudini::FieldType type, const std::vector<uint8_t>& encoded) {
  using namespace Cloudini;
  ConstBufferView encoded_view(encoded.data(), encoded.size());
  const EncodingInfo decoded_info = DecodeHeader(encoded_view);
  ASSERT_EQ(decoded_info.version, 5);
  ASSERT_EQ(decoded_info.encoding_opt, EncodingOptions::LOSSY);
  ASSERT_EQ(decoded_info.point_step, sizeof(IntType));
  ASSERT_EQ(decoded_info.fields.size(), 1u);
  ASSERT_EQ(decoded_info.fields[0].type, type);

  std::vector<IntType> output(values.size(), 0);
  PointcloudDecoder decoder;
  BufferView out_view(reinterpret_cast<uint8_t*>(output.data()), output.size() * sizeof(IntType));
  decoder.decode(decoded_info, encoded_view, out_view);
  ASSERT_EQ(output, values);
}

std::vector<uint8_t> v5UncompressedChunkModes(const std::vector<uint8_t>& encoded) {
  using namespace Cloudini;
  ConstBufferView encoded_view(encoded.data(), encoded.size());
  const EncodingInfo decoded_info = DecodeHeader(encoded_view);
  if (decoded_info.version != 5 || decoded_info.compression_opt != CompressionOption::NONE) {
    throw std::runtime_error("expected uncompressed V5 data");
  }

  std::vector<uint8_t> modes;
  while (!encoded_view.empty()) {
    if (encoded_view.size() < sizeof(uint32_t)) {
      throw std::runtime_error("truncated V5 chunk size");
    }
    uint32_t chunk_size = 0;
    std::memcpy(&chunk_size, encoded_view.data(), sizeof(chunk_size));
    encoded_view.trim_front(sizeof(chunk_size));
    if (chunk_size == 0 || chunk_size > encoded_view.size()) {
      throw std::runtime_error("invalid V5 chunk size");
    }
    modes.push_back(encoded_view.data()[0]);
    encoded_view.trim_front(chunk_size);
  }
  return modes;
}

template <typename IntType, typename Generator>
std::vector<IntType> makeIntSequence(size_t points, Generator generator) {
  std::vector<IntType> values(points);
  for (size_t i = 0; i < points; ++i) {
    values[i] = static_cast<IntType>(generator(i));
  }
  return values;
}

}  // namespace

TEST(FieldEncoders, FloatXOR_RoundTrip_Float32) {
  // Multi-chunk test: ensures the chunk-boundary reset path is covered.
  const size_t kChunkPoints = 32 * 1024;            // must match Cloudini::detail::kPointsPerChunk
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

// Full PointcloudEncoder/Decoder round-trip in LOSSLESS mode across multiple
// kPointsPerChunk-sized chunks; exercises the chunk-flush integration.
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
  ASSERT_EQ(decoded_info.version, kEncodingVersion);
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

// Regression: on incompressible data the stage-1 encoding (Gorilla FLOAT64, varint ints) is
// larger than width*height*point_step. For a single-chunk cloud the decoder used to size its
// decompression buffer from the raw cloud size, so LZ4/ZSTD rejected valid data.
TEST(FieldEncoders, PointcloudLossless_ExpandingStage1_SingleChunk) {
  using namespace Cloudini;

  struct Point {
    double t = 0;
    uint32_t a = 0;
    uint32_t b = 0;
  };
  static_assert(sizeof(Point) == 16);

  const size_t kNumpoints = 1000;
  std::mt19937_64 rng(139);
  std::vector<Point> input(kNumpoints);
  for (auto& p : input) {
    const uint64_t bits = rng();
    std::memcpy(&p.t, &bits, sizeof(bits));
    p.a = static_cast<uint32_t>(rng());
    p.b = static_cast<uint32_t>(rng());
  }
  const ConstBufferView in_view(reinterpret_cast<const uint8_t*>(input.data()), input.size() * sizeof(Point));

  for (const auto compression : {CompressionOption::NONE, CompressionOption::LZ4, CompressionOption::ZSTD}) {
    EncodingInfo info;
    info.width = kNumpoints;
    info.height = 1;
    info.point_step = sizeof(Point);
    info.encoding_opt = EncodingOptions::LOSSLESS;
    info.compression_opt = compression;
    info.fields.push_back({"t", 0, FieldType::FLOAT64, {}});
    info.fields.push_back({"a", 8, FieldType::UINT32, {}});
    info.fields.push_back({"b", 12, FieldType::UINT32, {}});

    std::vector<uint8_t> compressed;
    PointcloudEncoder encoder(info);
    encoder.encode(in_view, compressed);

    ConstBufferView comp_view(compressed.data(), compressed.size());
    const auto decoded_info = DecodeHeader(comp_view);
    if (compression == CompressionOption::NONE) {
      // Precondition of the regression: stage-1 really is larger than the raw cloud.
      ASSERT_GT(comp_view.size(), in_view.size());
    }

    std::vector<Point> output(kNumpoints);
    PointcloudDecoder decoder;
    BufferView out_view(reinterpret_cast<uint8_t*>(output.data()), output.size() * sizeof(Point));
    ASSERT_NO_THROW(decoder.decode(decoded_info, comp_view, out_view))
        << "compression " << static_cast<int>(compression);
    ASSERT_EQ(0, std::memcmp(input.data(), output.data(), input.size() * sizeof(Point)));
  }
}

TEST(FieldEncoders, PointcloudV5_AdaptiveIntModes_RoundTripAndModeSelection) {
  using namespace Cloudini;

  constexpr size_t kPoints = 32 * 1024 + 19;
  constexpr uint8_t kPaletteMode = 1;
  constexpr uint8_t kRleMode = 2;
  constexpr uint8_t kDeltaRleMode = 3;

  {
    const auto values = makeIntSequence<uint32_t>(kPoints, [](size_t i) { return 100000u + i * 3u; });
    const std::vector<uint8_t> encoded_none = encodeV5IntOnly(values, FieldType::UINT32, CompressionOption::NONE);
    EXPECT_EQ(v5UncompressedChunkModes(encoded_none), std::vector<uint8_t>({kDeltaRleMode, kDeltaRleMode}));
    expectV5IntOnlyRoundTrip(values, FieldType::UINT32, encoded_none);

    const std::vector<uint8_t> encoded_zstd = encodeV5IntOnly(values, FieldType::UINT32, CompressionOption::ZSTD);
    expectV5IntOnlyRoundTrip(values, FieldType::UINT32, encoded_zstd);
  }

  {
    const auto values = makeIntSequence<uint32_t>(kPoints, [](size_t i) { return static_cast<uint32_t>(i % 4); });
    const std::vector<uint8_t> encoded_none = encodeV5IntOnly(values, FieldType::UINT32, CompressionOption::NONE);
    EXPECT_EQ(v5UncompressedChunkModes(encoded_none), std::vector<uint8_t>({kPaletteMode, kPaletteMode}));
    expectV5IntOnlyRoundTrip(values, FieldType::UINT32, encoded_none);

    const std::vector<uint8_t> encoded_zstd = encodeV5IntOnly(values, FieldType::UINT32, CompressionOption::ZSTD);
    expectV5IntOnlyRoundTrip(values, FieldType::UINT32, encoded_zstd);
  }

  {
    const auto values =
        makeIntSequence<uint16_t>(kPoints, [](size_t i) { return static_cast<uint16_t>((i / 256) % 8); });
    const std::vector<uint8_t> encoded_none = encodeV5IntOnly(values, FieldType::UINT16, CompressionOption::NONE);
    EXPECT_EQ(v5UncompressedChunkModes(encoded_none), std::vector<uint8_t>({kRleMode, kRleMode}));
    expectV5IntOnlyRoundTrip(values, FieldType::UINT16, encoded_none);

    const std::vector<uint8_t> encoded_zstd = encodeV5IntOnly(values, FieldType::UINT16, CompressionOption::ZSTD);
    expectV5IntOnlyRoundTrip(values, FieldType::UINT16, encoded_zstd);
  }

  {
    std::vector<uint32_t> values(kPoints);
    uint32_t value = 1000;
    for (size_t i = 0; i < values.size(); ++i) {
      const uint32_t diff = ((i / 64) % 2 == 0) ? 3u : 7u;
      value += diff;
      values[i] = value;
    }
    const std::vector<uint8_t> encoded_none = encodeV5IntOnly(values, FieldType::UINT32, CompressionOption::NONE);
    EXPECT_EQ(v5UncompressedChunkModes(encoded_none), std::vector<uint8_t>({kDeltaRleMode, kDeltaRleMode}));
    expectV5IntOnlyRoundTrip(values, FieldType::UINT32, encoded_none);
  }

  {
    const auto values =
        makeIntSequence<int32_t>(kPoints, [](size_t i) { return 200000 - static_cast<int32_t>(i * 5); });
    const std::vector<uint8_t> encoded_none = encodeV5IntOnly(values, FieldType::INT32, CompressionOption::NONE);
    EXPECT_EQ(v5UncompressedChunkModes(encoded_none), std::vector<uint8_t>({kDeltaRleMode, kDeltaRleMode}));
    expectV5IntOnlyRoundTrip(values, FieldType::INT32, encoded_none);
  }

  {
    std::mt19937 rng(12345);
    std::uniform_int_distribution<uint32_t> dist(0, 0xFFFFu);
    std::vector<uint32_t> values(kPoints);
    for (uint32_t& value : values) {
      value = dist(rng);
    }
    const std::vector<uint8_t> encoded_none = encodeV5IntOnly(values, FieldType::UINT32, CompressionOption::NONE);
    const std::vector<uint8_t> modes = v5UncompressedChunkModes(encoded_none);
    ASSERT_EQ(modes.size(), 2u);
    for (uint8_t mode : modes) {
      EXPECT_NE(mode, kDeltaRleMode);
    }
    expectV5IntOnlyRoundTrip(values, FieldType::UINT32, encoded_none);
  }
}

TEST(FieldEncoders, PointcloudV5_AdaptiveProbeBoundaries_RoundTrip) {
  using namespace Cloudini;

  constexpr uint8_t kDeltaRleMode = 3;
  const std::array<size_t, 5> point_counts = {4095, 4096, 4097, 32 * 1024, 32 * 1024 + 7};

  for (size_t points : point_counts) {
    const auto values = makeIntSequence<uint32_t>(points, [](size_t i) { return static_cast<uint32_t>(1000 + i * 3); });
    const std::vector<uint8_t> encoded = encodeV5IntOnly(values, FieldType::UINT32, CompressionOption::NONE);
    const std::vector<uint8_t> modes = v5UncompressedChunkModes(encoded);
    ASSERT_FALSE(modes.empty()) << "points=" << points;
    for (uint8_t mode : modes) {
      EXPECT_EQ(mode, kDeltaRleMode) << "points=" << points;
    }
    expectV5IntOnlyRoundTrip(values, FieldType::UINT32, encoded);
  }
}

TEST(FieldEncoders, PointcloudV5_LossyFloatOnlyRoundTrip) {
  using namespace Cloudini;

  struct PointXYZI {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
  };

  constexpr size_t kPoints = 4096 + 37;
  std::vector<PointXYZI> input(kPoints);
  for (size_t i = 0; i < input.size(); ++i) {
    input[i].x = 0.001F * static_cast<float>(i);
    input[i].y = -0.002F * static_cast<float>(i % 97);
    input[i].z = 10.0F + 0.003F * static_cast<float>(i % 251);
    input[i].intensity = 0.01F * static_cast<float>(i % 1024);
  }

  EncodingInfo info;
  info.width = static_cast<uint32_t>(input.size());
  info.height = 1;
  info.point_step = sizeof(PointXYZI);
  info.encoding_opt = EncodingOptions::LOSSY;
  info.compression_opt = CompressionOption::NONE;
  info.use_threads = false;
  info.version = 5;  // the V5 path (the default is V6)
  info.fields.push_back({"x", offsetof(PointXYZI, x), FieldType::FLOAT32, 0.001F});
  info.fields.push_back({"y", offsetof(PointXYZI, y), FieldType::FLOAT32, 0.001F});
  info.fields.push_back({"z", offsetof(PointXYZI, z), FieldType::FLOAT32, 0.001F});
  info.fields.push_back({"intensity", offsetof(PointXYZI, intensity), FieldType::FLOAT32, 0.001F});

  std::vector<uint8_t> encoded;
  {
    PointcloudEncoder encoder(info);
    ConstBufferView in_view(reinterpret_cast<const uint8_t*>(input.data()), input.size() * sizeof(PointXYZI));
    encoder.encode(in_view, encoded);
  }

  EncodingInfo v4_info = info;
  v4_info.version = 4;
  std::vector<uint8_t> encoded_v4;
  {
    PointcloudEncoder encoder(v4_info);
    ConstBufferView in_view(reinterpret_cast<const uint8_t*>(input.data()), input.size() * sizeof(PointXYZI));
    encoder.encode(in_view, encoded_v4);
  }

  ConstBufferView encoded_view(encoded.data(), encoded.size());
  const EncodingInfo decoded_info = DecodeHeader(encoded_view);
  ASSERT_EQ(decoded_info.version, 5);
  ASSERT_EQ(decoded_info.encoding_opt, EncodingOptions::LOSSY);

  ConstBufferView encoded_v4_view(encoded_v4.data(), encoded_v4.size());
  const EncodingInfo decoded_v4_info = DecodeHeader(encoded_v4_view);
  ASSERT_EQ(decoded_v4_info.version, 4);
  ASSERT_EQ(encoded_view.size(), encoded_v4_view.size());
  EXPECT_EQ(
      std::vector<uint8_t>(encoded_view.data(), encoded_view.data() + encoded_view.size()),
      std::vector<uint8_t>(encoded_v4_view.data(), encoded_v4_view.data() + encoded_v4_view.size()));

  std::vector<PointXYZI> output(input.size());
  {
    PointcloudDecoder decoder;
    BufferView out_view(reinterpret_cast<uint8_t*>(output.data()), output.size() * sizeof(PointXYZI));
    decoder.decode(decoded_info, encoded_view, out_view);
  }

  constexpr float kTolerance = 0.0011F;
  for (size_t i = 0; i < input.size(); ++i) {
    ASSERT_NEAR(input[i].x, output[i].x, kTolerance) << "x @" << i;
    ASSERT_NEAR(input[i].y, output[i].y, kTolerance) << "y @" << i;
    ASSERT_NEAR(input[i].z, output[i].z, kTolerance) << "z @" << i;
    ASSERT_NEAR(input[i].intensity, output[i].intensity, kTolerance) << "intensity @" << i;
  }
}

TEST(FieldEncoders, PointcloudDecoderRejectsMissingChunksForDeclaredPoints) {
  using namespace Cloudini;

  EncodingInfo info;
  info.version = 4;
  info.width = 1;
  info.height = 1;
  info.point_step = sizeof(uint8_t);
  info.encoding_opt = EncodingOptions::NONE;
  info.compression_opt = CompressionOption::NONE;
  info.use_threads = false;
  info.fields.push_back({"value", 0, FieldType::UINT8, std::nullopt});

  std::array<uint8_t, 1> output = {0};
  std::vector<uint8_t> encoded;
  ConstBufferView encoded_view(encoded.data(), encoded.size());
  BufferView output_view(output.data(), output.size());

  PointcloudDecoder decoder;
  EXPECT_THROW(decoder.decode(info, encoded_view, output_view), std::runtime_error);
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

TEST(FieldEncoders, FloatNDecodePointsMatchesPerPointDecode) {
  using namespace Cloudini;

  struct Point {
    float v[4];
    uint32_t pad = 0xDEADBEEF;
  };
  constexpr size_t kPoints = 3000;
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> step(-0.05f, 0.05f);
  std::uniform_real_distribution<float> jump(-300.0f, 300.0f);
  std::vector<Point> input(kPoints);
  float walk[4] = {1.0f, -2.0f, 3.0f, 10.0f};
  for (size_t i = 0; i < kPoints; ++i) {
    for (int k = 0; k < 4; ++k) {
      walk[k] += (rng() % 50 == 0) ? jump(rng) : step(rng);  // mostly 1-2 byte varints, some long ones
      input[i].v[k] = (rng() % 40 == 0) ? std::numeric_limits<float>::quiet_NaN() : walk[k];
    }
  }

  for (size_t fields = 2; fields <= 4; ++fields) {
    for (bool skip_one : {false, true}) {
      std::vector<FieldEncoderFloatN_Lossy::FieldData> enc_fields;
      std::vector<FieldDecoderFloatN_Lossy::FieldData> dec_fields;
      for (size_t k = 0; k < fields; ++k) {
        enc_fields.emplace_back(k * sizeof(float), 0.001f);
        const size_t dec_offset = (skip_one && k == 1) ? kDecodeButSkipStore : k * sizeof(float);
        dec_fields.emplace_back(dec_offset, 0.001f);
      }
      FieldEncoderFloatN_Lossy encoder(enc_fields);
      std::vector<uint8_t> encoded(kPoints * fields * kMaxVarintBytes);
      BufferView enc_view(encoded.data(), encoded.size());
      size_t encoded_size = 0;
      for (const auto& p : input) {
        encoded_size += encoder.encode(ConstBufferView(reinterpret_cast<const uint8_t*>(&p), sizeof(Point)), enc_view);
      }

      std::vector<Point> per_point(kPoints), batch(kPoints);
      FieldDecoderFloatN_Lossy decoder_a(dec_fields);
      ConstBufferView in_a(encoded.data(), encoded_size);
      for (size_t i = 0; i < kPoints; ++i) {
        decoder_a.decode(in_a, BufferView(reinterpret_cast<uint8_t*>(&per_point[i]), sizeof(Point)));
      }
      FieldDecoderFloatN_Lossy decoder_b(dec_fields);
      ConstBufferView in_b(encoded.data(), encoded_size);
      decoder_b.decodePoints(in_b, reinterpret_cast<uint8_t*>(batch.data()), sizeof(Point), kPoints);

      EXPECT_TRUE(in_a.empty());
      EXPECT_TRUE(in_b.empty());
      ASSERT_EQ(0, std::memcmp(per_point.data(), batch.data(), kPoints * sizeof(Point)))
          << "fields=" << fields << " skip_one=" << skip_one;

      // Truncated input must throw, not read past the end
      FieldDecoderFloatN_Lossy decoder_c(dec_fields);
      ConstBufferView truncated(encoded.data(), encoded_size - 1);
      EXPECT_THROW(
          decoder_c.decodePoints(truncated, reinterpret_cast<uint8_t*>(batch.data()), sizeof(Point), kPoints),
          std::runtime_error);
    }
  }
}

TEST(FieldEncoders, EncodeToVectorMatchesPreallocatedBuffer) {
  using namespace Cloudini;

  constexpr size_t kPoints = 3 * 32 * 1024 + 11;  // several chunks
  std::vector<float> cloud(kPoints * 4);
  for (size_t i = 0; i < kPoints; ++i) {
    cloud[4 * i + 0] = std::sin(0.001f * i) * 10.0f;
    cloud[4 * i + 1] = std::cos(0.001f * i) * 10.0f;
    cloud[4 * i + 2] = 0.0001f * i;
    cloud[4 * i + 3] = static_cast<float>(i % 256);
  }
  EncodingInfo info;
  info.width = kPoints;
  info.point_step = 4 * sizeof(float);
  for (int k = 0; k < 4; ++k) {
    info.fields.push_back({std::string(1, "xyzi"[k]), static_cast<uint32_t>(4 * k), FieldType::FLOAT32, 0.001f});
  }
  const ConstBufferView in(reinterpret_cast<const uint8_t*>(cloud.data()), cloud.size() * sizeof(float));
  for (auto compression : {CompressionOption::NONE, CompressionOption::LZ4, CompressionOption::ZSTD}) {
    info.compression_opt = compression;
    PointcloudEncoder encoder(info);
    std::vector<uint8_t> as_vector;
    // twice: the second call reuses the scratch buffer
    encoder.encode(in, as_vector);
    const size_t size = encoder.encode(in, as_vector);
    ASSERT_EQ(size, as_vector.size());

    std::vector<uint8_t> prealloc(MaxCompressedSize(info, kPoints, true));
    BufferView view(prealloc.data(), prealloc.size());
    const size_t prealloc_size = encoder.encode(in, view, true);
    ASSERT_EQ(prealloc_size, size);
    ASSERT_EQ(0, std::memcmp(prealloc.data(), as_vector.data(), size));
  }
}

namespace {

// Encode `values` as a single-field cloud with the given version and compression, return the payload size.
template <typename IntType>
size_t encodedIntFieldSize(
    const std::vector<IntType>& values, Cloudini::FieldType type, uint8_t version,
    Cloudini::CompressionOption compression) {
  using namespace Cloudini;
  EncodingInfo info = makeV5IntOnlyInfo(values.size(), type, compression);
  info.version = version;
  PointcloudEncoder encoder(info);
  std::vector<uint8_t> encoded;
  encoder.encode(
      ConstBufferView(reinterpret_cast<const uint8_t*>(values.data()), values.size() * sizeof(IntType)), encoded);
  if (version == 5) {
    expectV5IntOnlyRoundTrip(values, type, encoded);
  }
  return encoded.size() - encoder.getHeader().size();
}

}  // namespace

TEST(FieldEncoders, PointcloudV5_ModeSelectionAccountsForStage2) {
  using namespace Cloudini;
  constexpr uint8_t kPaletteMode = 1;

  // Per-column timestamps of an organized scan (like Ouster's `t`): every row repeats the same 1024 values.
  // Before stage 2, the palette (10-bit indexes plus a table of the 1024 raw values) beats delta-varint
  // (3 bytes per value); after it, the raw table barely compresses while the repeated deltas do, so V5
  // must not end up larger than plain V4 deltas.
  std::vector<uint32_t> column_time(1024);
  std::mt19937 rng(7);
  for (size_t col = 0; col < column_time.size(); ++col) {
    column_time[col] = static_cast<uint32_t>(col * 97656 + rng() % 64);
  }
  const auto t = makeIntSequence<uint32_t>(64 * 1024, [&](size_t i) { return column_time[i % 1024]; });
  EXPECT_EQ(
      v5UncompressedChunkModes(encodeV5IntOnly(t, FieldType::UINT32, CompressionOption::NONE)),
      std::vector<uint8_t>({kPaletteMode, kPaletteMode}));
  for (auto compression : {CompressionOption::ZSTD, CompressionOption::LZ4}) {
    const size_t v4 = encodedIntFieldSize(t, FieldType::UINT32, 4, compression);
    const size_t v5 = encodedIntFieldSize(t, FieldType::UINT32, 5, compression);
    EXPECT_LE(v5, v4 + 64) << "compression " << ToString(compression);
  }

  // Random choice among 16 values: the palette stays the better choice after compression too.
  std::vector<uint32_t> levels(16);
  for (auto& level : levels) {
    level = static_cast<uint32_t>(rng());
  }
  const auto labels = makeIntSequence<uint32_t>(64 * 1024, [&](size_t) { return levels[rng() % 16]; });
  const size_t v4 = encodedIntFieldSize(labels, FieldType::UINT32, 4, CompressionOption::ZSTD);
  const size_t v5 = encodedIntFieldSize(labels, FieldType::UINT32, 5, CompressionOption::ZSTD);
  EXPECT_LT(v5, v4);
}

TEST(FieldEncoders, PointcloudV5_MixedFieldsRoundTripWithStage2) {
  using namespace Cloudini;

  // xyzi floats followed by several adaptive integer sections, over multiple chunks, with and without the
  // compression thread: the ZSTD blocks are split where the sections start.
  struct Point {
    float x, y, z, intensity;
    uint32_t t;
    uint16_t reflectivity, ring, ambient;
    uint16_t pad;
    uint32_t range;
  };
  constexpr size_t kWidth = 1024;
  constexpr size_t kRows = 80;  // 81920 points: 3 chunks
  std::vector<Point> input(kWidth * kRows);
  std::mt19937 rng(3);
  for (size_t r = 0; r < kRows; ++r) {
    for (size_t c = 0; c < kWidth; ++c) {
      Point& p = input[r * kWidth + c];
      const float range = 5.0f + 0.001f * static_cast<float>(rng() % 20000);
      const float az = 6.2831853f * static_cast<float>(c) / kWidth;
      p.x = range * std::cos(az);
      p.y = range * std::sin(az);
      p.z = 0.01f * static_cast<float>(r);
      p.intensity = static_cast<float>(rng() % 300);
      p.t = static_cast<uint32_t>(c * 48828);
      p.reflectivity = static_cast<uint16_t>(rng() % 256);
      p.ring = static_cast<uint16_t>(r);
      p.ambient = static_cast<uint16_t>(500 + rng() % 40);
      p.pad = 0;
      p.range = static_cast<uint32_t>(range * 1000.0f);
    }
  }
  EncodingInfo info;
  info.width = kWidth;
  info.height = kRows;
  info.point_step = sizeof(Point);
  info.fields = {
      {"x", 0, FieldType::FLOAT32, 0.001f}, {"y", 4, FieldType::FLOAT32, 0.001f},
      {"z", 8, FieldType::FLOAT32, 0.001f}, {"intensity", 12, FieldType::FLOAT32, 0.001f},
      {"t", 16, FieldType::UINT32, {}},     {"reflectivity", 20, FieldType::UINT16, {}},
      {"ring", 22, FieldType::UINT16, {}},  {"ambient", 24, FieldType::UINT16, {}},
      {"range", 28, FieldType::UINT32, {}},
  };
  const ConstBufferView in(reinterpret_cast<const uint8_t*>(input.data()), input.size() * sizeof(Point));

  std::vector<uint8_t> reference;
  for (auto compression : {CompressionOption::NONE, CompressionOption::LZ4, CompressionOption::ZSTD}) {
    for (bool threads : {false, true}) {
      info.compression_opt = compression;
      info.use_threads = threads;
      PointcloudEncoder encoder(info);
      std::vector<uint8_t> encoded;
      encoder.encode(in, encoded);

      ConstBufferView view(encoded.data(), encoded.size());
      const EncodingInfo header = DecodeHeader(view);
      std::vector<uint8_t> decoded(input.size() * sizeof(Point), 0);
      PointcloudDecoder decoder;
      decoder.decode(header, view, decoded);
      if (reference.empty()) {
        reference = decoded;
      }
      ASSERT_EQ(decoded, reference) << ToString(compression) << " threads=" << threads;
    }
  }
  // integer fields are lossless
  const auto* decoded_points = reinterpret_cast<const Point*>(reference.data());
  for (size_t i = 0; i < input.size(); ++i) {
    ASSERT_EQ(decoded_points[i].t, input[i].t);
    ASSERT_EQ(decoded_points[i].reflectivity, input[i].reflectivity);
    ASSERT_EQ(decoded_points[i].ring, input[i].ring);
    ASSERT_EQ(decoded_points[i].ambient, input[i].ambient);
    ASSERT_EQ(decoded_points[i].range, input[i].range);
    ASSERT_NEAR(decoded_points[i].x, input[i].x, 0.0006f);
  }
}

TEST(FieldEncoders, RefineResolutionsToData) {
  using namespace Cloudini;

  struct Point {
    float x, y, z;
    float intensity;    // integer values
    float reflectance;  // 0.01 steps
    float noise;        // no grid
  };
  constexpr size_t kPoints = 5000;
  std::mt19937 rng(11);
  std::uniform_real_distribution<float> uniform(-1.0f, 1.0f);
  std::vector<Point> input(kPoints);
  float walk = 0.0f;
  for (auto& p : input) {
    walk += 0.01f * uniform(rng);
    p.x = 10.0f + walk;
    p.y = -3.0f + 2.0f * walk;
    p.z = 0.5f * walk;
    p.intensity = static_cast<float>(rng() % 256);
    p.reflectance = static_cast<float>(rng() % 100) * 0.01f;
    p.noise = uniform(rng);
  }
  input[17].intensity = std::numeric_limits<float>::quiet_NaN();  // NaN does not prevent the refinement

  EncodingInfo info;
  info.width = kPoints;
  info.point_step = sizeof(Point);
  const char* names[] = {"x", "y", "z", "intensity", "reflectance", "noise"};
  for (uint32_t k = 0; k < 6; ++k) {
    info.fields.push_back({names[k], k * 4, FieldType::FLOAT32, 0.001f});
  }
  const ConstBufferView in(reinterpret_cast<const uint8_t*>(input.data()), input.size() * sizeof(Point));

  EncodingInfo refined = info;
  RefineResolutionsToData(refined, in);
  for (size_t k : {0, 1, 2, 5}) {
    EXPECT_EQ(refined.fields[k].resolution, 0.001f) << names[k];
  }
  EXPECT_EQ(refined.fields[3].resolution, 1.0f);
  ASSERT_TRUE(refined.fields[4].resolution.has_value());
  EXPECT_NEAR(*refined.fields[4].resolution, 0.01f, 1e-7f);

  auto encode = [&](const EncodingInfo& encoding) {
    PointcloudEncoder encoder(encoding);
    std::vector<uint8_t> out;
    encoder.encode(in, out);
    return out;
  };
  const std::vector<uint8_t> plain = encode(info);
  const std::vector<uint8_t> compact = encode(refined);
  EXPECT_LT(compact.size(), plain.size());

  ConstBufferView view(compact.data(), compact.size());
  const EncodingInfo header = DecodeHeader(view);
  EXPECT_EQ(header.fields[3].resolution, 1.0f);
  std::vector<Point> output(kPoints);
  PointcloudDecoder decoder;
  decoder.decode(header, view, BufferView(reinterpret_cast<uint8_t*>(output.data()), output.size() * sizeof(Point)));
  for (size_t i = 0; i < kPoints; ++i) {
    if (i == 17) {
      EXPECT_TRUE(std::isnan(output[i].intensity));
    } else {
      ASSERT_EQ(output[i].intensity, input[i].intensity) << i;  // lossless
    }
    // half the requested resolution, plus float rounding
    ASSERT_NEAR(output[i].reflectance, input[i].reflectance, 0.0006f) << i;
    ASSERT_NEAR(output[i].noise, input[i].noise, 0.0006f) << i;
    ASSERT_NEAR(output[i].x, input[i].x, 0.0006f) << i;
  }

  // A half-integer value: the values still lie on a 0.5 grid
  input[123].intensity = 3.5f;
  EncodingInfo halves = info;
  RefineResolutionsToData(halves, in);
  EXPECT_EQ(halves.fields[3].resolution, 0.5f);

  // Values on no coarser grid keep the requested resolution
  input[123].intensity = 3.1234f;
  input[456].reflectance = 0.123f;
  EncodingInfo unchanged = info;
  RefineResolutionsToData(unchanged, in);
  EXPECT_EQ(unchanged.fields[3].resolution, 0.001f);
  EXPECT_EQ(unchanged.fields[4].resolution, 0.001f);
}

namespace {

// Copy of `data` placed so that it ends exactly where an unreadable page starts (on POSIX systems), so
// that reading even one byte past its end crashes the test instead of going unnoticed.
class GuardedBuffer {
 public:
  explicit GuardedBuffer(const std::vector<uint8_t>& data) {
#if defined(__unix__) || defined(__APPLE__)
    const size_t page = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    const size_t data_pages = (data.size() + page - 1) / page;
    size_ = (data_pages + 1) * page;
    void* mem = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (mem == MAP_FAILED) {
      throw std::runtime_error("mmap failed");
    }
    base_ = static_cast<uint8_t*>(mem);
    mprotect(base_ + data_pages * page, page, PROT_NONE);
    data_ = base_ + data_pages * page - data.size();
    size_data_ = data.size();
    memcpy(data_, data.data(), data.size());
#else
    fallback_ = data;
    data_ = fallback_.data();
    size_data_ = fallback_.size();
#endif
  }
  ~GuardedBuffer() {
#if defined(__unix__) || defined(__APPLE__)
    munmap(base_, size_);
#endif
  }
  GuardedBuffer(const GuardedBuffer&) = delete;
  GuardedBuffer& operator=(const GuardedBuffer&) = delete;

  Cloudini::ConstBufferView view() const {
    return {data_, size_data_};
  }

 private:
  uint8_t* base_ = nullptr;
  size_t size_ = 0;
  uint8_t* data_ = nullptr;
  size_t size_data_ = 0;
  std::vector<uint8_t> fallback_;
};

}  // namespace

// A corrupted or malicious message must make the decoder throw, never read past the payload. The
// per-point size check counts the minimum size of every field (1 byte per varint), so a chunk whose
// last point has a 2-byte varint and is one byte short passes it. Before the fix, the decoder of the
// next field (copy, XOR or single lossy float) then read past the end of the chunk. With NONE
// compression the chunk is read in place, i.e. past the end of the caller's buffer.
TEST(FieldEncoders, CorruptedPayloadNeverReadsPastTheEnd) {
  using namespace Cloudini;

  struct Case {
    const char* name;
    std::vector<PointField> fields;
    uint32_t point_step;
    EncodingOptions encoding;
    std::vector<uint8_t> good_point;  // stage-1 bytes of one well-formed point
    std::vector<uint8_t> last_point;  // stage-1 bytes of the damaged last point
    std::vector<uint8_t> versions;
  };
  const std::vector<Case> cases = {
      // x, y, z, intensity as one vector of four varints, then a uint8 label stored as is
      {"copy",
       {{"x", 0, FieldType::FLOAT32, 0.001f},
        {"y", 4, FieldType::FLOAT32, 0.001f},
        {"z", 8, FieldType::FLOAT32, 0.001f},
        {"intensity", 12, FieldType::FLOAT32, 0.001f},
        {"label", 16, FieldType::UINT8, std::nullopt}},
       17,
       EncodingOptions::LOSSY,
       {0x81, 0x01, 0x81, 0x01, 0x81, 0x01, 0x81, 0x01, 0x07},
       {0x81, 0x01, 0x81, 0x01, 0x81, 0x01, 0x81, 0x01},
       {4, 5}},
      // lossless: an int32 varint, then a float stored as a 4-byte XOR residual
      {"xor",
       {{"t", 0, FieldType::INT32, std::nullopt}, {"intensity", 4, FieldType::FLOAT32, std::nullopt}},
       8,
       EncodingOptions::LOSSLESS,
       {0x81, 0x01, 0x00, 0x00, 0x20, 0x41},
       {0x81, 0x01, 0x00, 0x00, 0x20},
       {4, 5}},
      // lossy: an int32 varint, then a float that is not part of the leading xyz vector
      {"lossy float",
       {{"t", 0, FieldType::INT32, std::nullopt}, {"intensity", 4, FieldType::FLOAT32, 0.01f}},
       8,
       EncodingOptions::LOSSY,
       {0x81, 0x01, 0x03},
       {0x81, 0x01},
       {4}},  // in V5 the int32 field is an adaptive section, not a per-point varint
  };

  constexpr size_t kGoodPoints = 50;
  for (const auto& c : cases) {
    std::vector<uint8_t> payload(sizeof(uint32_t));
    for (size_t i = 0; i < kGoodPoints; ++i) {
      payload.insert(payload.end(), c.good_point.begin(), c.good_point.end());
    }
    payload.insert(payload.end(), c.last_point.begin(), c.last_point.end());
    const uint32_t chunk_size = static_cast<uint32_t>(payload.size() - sizeof(uint32_t));
    memcpy(payload.data(), &chunk_size, sizeof(chunk_size));
    // the payload ends where an unreadable page begins: a read past its end crashes
    GuardedBuffer guarded(payload);

    for (uint8_t version : c.versions) {
      EncodingInfo info;
      info.width = kGoodPoints + 1;
      info.height = 1;
      info.point_step = c.point_step;
      info.fields = c.fields;
      info.encoding_opt = c.encoding;
      info.compression_opt = CompressionOption::NONE;
      info.version = version;

      std::vector<uint8_t> output(info.width * info.point_step);
      PointcloudDecoder decoder;
      EXPECT_THROW(decoder.decode(info, guarded.view(), BufferView(output.data(), output.size())), std::runtime_error)
          << c.name << ", version " << int(version);
    }
  }
}

// The refined resolution is stored as a float, so it is not exactly g * r: each decoded value moves
// by (value / resolution) * |R - g * r|. For large values that can exceed the original error bound.
// Realistic case: the odometer reading of a vehicle in meters (FLOAT64, ~100 km), logged in 1 cm steps
// and stored with a 1 mm resolution. Refining must never make the decoded values worse than before.
// Beyond 2^22 steps V6 quantizes in double precision and decodes within r / 2, where the float path of
// V4 does not: a refined grid must not be accepted just because it is no worse than V4 there.
TEST(FieldEncoders, RefineResolutionsToDataKeepsV6BoundBeyond2e22Steps) {
  using namespace Cloudini;
  struct Point {
    float x, y, z;
  };
  const std::vector<Point> input = {{10825.5400390625f, 0.0f, 0.0f}, {4506.31005859375f, 0.0f, 0.0f}};
  EncodingInfo info;
  info.width = static_cast<uint32_t>(input.size());
  info.point_step = sizeof(Point);
  info.version = 6;
  info.compression_opt = CompressionOption::NONE;
  info.fields = {
      {"x", offsetof(Point, x), FieldType::FLOAT32, 0.001f},
      {"y", offsetof(Point, y), FieldType::FLOAT32, 0.001f},
      {"z", offsetof(Point, z), FieldType::FLOAT32, 0.001f}};
  const ConstBufferView in(reinterpret_cast<const uint8_t*>(input.data()), input.size() * sizeof(Point));
  RefineResolutionsToData(info, in);

  std::vector<uint8_t> encoded;
  PointcloudEncoder(info).encode(in, encoded);
  ConstBufferView view(encoded.data(), encoded.size());
  const EncodingInfo decoded_info = DecodeHeader(view);
  std::vector<Point> output(input.size());
  PointcloudDecoder().decode(
      decoded_info, view, BufferView(reinterpret_cast<uint8_t*>(output.data()), output.size() * sizeof(Point)));
  for (size_t i = 0; i < input.size(); ++i) {
    EXPECT_LE(std::abs(static_cast<double>(output[i].x) - input[i].x), 0.0005 * 1.001) << i;
  }
}

TEST(FieldEncoders, RefineResolutionsToDataKeepsErrorBoundForLargeValues) {
  using namespace Cloudini;

  struct Point {
    float x, y, z;
    float ring;         // integer values: refined to resolution 1
    double odometer;    // meters, on a 1 cm grid
    float temperature;  // ~9000-10000 on a 0.01 grid: float precision is ~0.001 there
  };
  constexpr size_t kPoints = 20000;
  std::vector<Point> input(kPoints);
  for (size_t i = 0; i < kPoints; ++i) {
    auto& p = input[i];
    p.x = 0.001f * static_cast<float>(i % 700);
    p.y = -0.002f * static_cast<float>(i % 300);
    p.z = 1.5f;
    p.ring = static_cast<float>(i % 64);
    p.odometer = 100000.0 - static_cast<double>(i) * 0.01;
    p.temperature = 9000.0f + static_cast<float>(i % 100000) * 0.05f;
  }

  EncodingInfo info;
  info.width = kPoints;
  info.point_step = sizeof(Point);
  info.fields = {
      {"x", offsetof(Point, x), FieldType::FLOAT32, 0.001f},
      {"y", offsetof(Point, y), FieldType::FLOAT32, 0.001f},
      {"z", offsetof(Point, z), FieldType::FLOAT32, 0.001f},
      {"ring", offsetof(Point, ring), FieldType::FLOAT32, 0.001f},
      {"odometer", offsetof(Point, odometer), FieldType::FLOAT64, 0.001f},
      {"temperature", offsetof(Point, temperature), FieldType::FLOAT32, 0.001f}};

  const ConstBufferView in(reinterpret_cast<const uint8_t*>(input.data()), kPoints * sizeof(Point));
  auto round_trip = [&](const EncodingInfo& encoding) {
    std::vector<uint8_t> encoded;
    PointcloudEncoder encoder(encoding);
    encoder.encode(in, encoded);
    ConstBufferView view(encoded.data(), encoded.size());
    const EncodingInfo decoded_info = DecodeHeader(view);
    std::vector<Point> output(kPoints);
    PointcloudDecoder decoder;
    decoder.decode(decoded_info, view, BufferView(reinterpret_cast<uint8_t*>(output.data()), kPoints * sizeof(Point)));
    return output;
  };

  const std::vector<Point> plain = round_trip(info);
  EncodingInfo refined_info = info;
  RefineResolutionsToData(refined_info, in);
  EXPECT_EQ(refined_info.fields[3].resolution, 1.0f);  // ring
  const std::vector<Point> refined = round_trip(refined_info);

  double max_error_odometer = 0.0;
  for (size_t i = 0; i < kPoints; ++i) {
    EXPECT_EQ(refined[i].ring, input[i].ring);
    const double odometer_error = std::abs(refined[i].odometer - input[i].odometer);
    max_error_odometer = std::max(max_error_odometer, odometer_error);
    // never worse than without the refinement (for float fields near 1e4, the original error is
    // already above resolution / 2: float has ~0.001 precision there)
    // at most 0.1% of the resolution worse than without the refinement
    ASSERT_LE(odometer_error, std::max(0.0005, std::abs(plain[i].odometer - input[i].odometer)) + 1e-6)
        << "odometer of point " << i << " (" << input[i].odometer << ")";
    ASSERT_LE(
        std::abs(refined[i].temperature - input[i].temperature),
        std::max(0.0005f, std::abs(plain[i].temperature - input[i].temperature)) + 1e-6f)
        << "temperature of point " << i << " (" << input[i].temperature << ")";
  }
  EXPECT_LE(max_error_odometer, 0.0005 + 1e-6);
}
