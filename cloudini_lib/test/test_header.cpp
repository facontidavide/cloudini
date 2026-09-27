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
#include <clocale>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <locale>
#include <stdexcept>
#include <string>
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/ros_message_definitions.hpp"  // second includer: see test_ros_msg.cpp

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
    const Cloudini::EncodingInfo& expected_info, const std::vector<VersionPoint>& input,
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

// Switches the C locale (and optionally the global C++ locale) to one that uses ','
// as decimal separator (and '.' as thousands separator), restoring both on destruction.
class CommaDecimalLocaleGuard {
 public:
  explicit CommaDecimalLocaleGuard(bool set_cpp_global_locale) : previous_cpp_locale_(std::locale()) {
    const char* current = std::setlocale(LC_ALL, nullptr);
    previous_c_locale_ = current ? current : "C";
    for (const char* name : {"de_DE.UTF-8", "de_DE.utf8", "de_DE", "fr_FR.UTF-8", "fr_FR.utf8", "it_IT.UTF-8"}) {
      try {
        std::locale cpp_locale(name);
        if (std::setlocale(LC_ALL, name) == nullptr) {
          continue;
        }
        if (set_cpp_global_locale) {
          std::locale::global(cpp_locale);
        }
        active_name_ = name;
        return;
      } catch (const std::runtime_error&) {
        // locale not installed, try the next one
      }
    }
  }

  ~CommaDecimalLocaleGuard() {
    std::locale::global(previous_cpp_locale_);
    std::setlocale(LC_ALL, previous_c_locale_.c_str());
  }

  CommaDecimalLocaleGuard(const CommaDecimalLocaleGuard&) = delete;
  CommaDecimalLocaleGuard& operator=(const CommaDecimalLocaleGuard&) = delete;

  bool active() const {
    return !active_name_.empty();
  }
  const std::string& name() const {
    return active_name_;
  }

 private:
  std::locale previous_cpp_locale_;
  std::string previous_c_locale_;
  std::string active_name_;
};

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

  const size_t n = 64 * 1024 + 7;  // multi-chunk (kPointsPerChunk = 32K)
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

// Regression tests for issue #123: the YAML header must be written and parsed
// independently of the process locale (C locale and global C++ locale).
static void expectHeaderRoundTripUnderCurrentLocale() {
  using namespace Cloudini;

  EncodingInfo header;
  header.width = 1234567;  // large enough to trigger thousands grouping in de_DE
  header.height = 1;
  header.point_step = sizeof(float) * 4;
  header.encoding_opt = EncodingOptions::LOSSY;
  header.compression_opt = CompressionOption::ZSTD;
  header.fields.push_back({"x", 0, FieldType::FLOAT32, 0.001F});
  header.fields.push_back({"y", 4, FieldType::FLOAT32, 0.001F});
  header.fields.push_back({"z", 8, FieldType::FLOAT32, 0.001F});
  header.fields.push_back({"intensity", 12, FieldType::FLOAT32, 0.1234567F});

  const std::string yaml = EncodingInfoToYAML(header);
  EXPECT_NE(yaml.find("resolution: 0.001\n"), std::string::npos) << yaml;
  EXPECT_NE(yaml.find("width: 1234567\n"), std::string::npos) << yaml;

  std::vector<uint8_t> buffer;
  EncodeHeader(header, buffer);
  ConstBufferView input(buffer.data(), buffer.size());
  const auto decoded_header = DecodeHeader(input);

  ASSERT_EQ(decoded_header.width, header.width);
  ASSERT_EQ(decoded_header.height, header.height);
  ASSERT_EQ(decoded_header.point_step, header.point_step);
  ASSERT_EQ(decoded_header.fields, header.fields);  // bit-exact resolution round-trip

  // Full encode/decode of a small cloud (this is where issue #123 threw).
  const size_t kPoints = 1000;
  const std::vector<VersionPoint> points = makeVersionedPoints(kPoints);
  const EncodingInfo info = makeVersionedLossyInfo(kPoints);
  const std::vector<uint8_t> encoded = encodeVersionedPoints(info, points);
  expectVersionedRoundTrip(info, points, encoded);
}

// Only the C locale is changed (e.g. setlocale(LC_ALL, "") in an application):
// this is the exact scenario of issue #123, where std::stof read "0.001" as 0.
TEST(Cloudini, HeaderLocaleIndependent_CLocale) {
  CommaDecimalLocaleGuard locale_guard(false);
  if (!locale_guard.active()) {
    GTEST_SKIP() << "No locale with ',' decimal separator is installed (e.g. de_DE.UTF-8)";
  }
  ASSERT_STREQ(std::localeconv()->decimal_point, ",") << locale_guard.name();
  expectHeaderRoundTripUnderCurrentLocale();
}

// Both the C locale and the global C++ locale (std::locale::global) are changed.
TEST(Cloudini, HeaderLocaleIndependent_CppGlobalLocale) {
  CommaDecimalLocaleGuard locale_guard(true);
  if (!locale_guard.active()) {
    GTEST_SKIP() << "No locale with ',' decimal separator is installed (e.g. de_DE.UTF-8)";
  }
  ASSERT_STREQ(std::localeconv()->decimal_point, ",") << locale_guard.name();
  expectHeaderRoundTripUnderCurrentLocale();
}

namespace {

struct RlePoint {
  float x, y, z;
  uint16_t ring;
};

// A V5 message (no stage-2 compression, one chunk) whose last section, the RLE section of the `ring`
// field, is rewritten into two runs: one point, then `second_run` points.
std::vector<uint8_t> craftTwoRuns(uint64_t second_run, bool delta_rle) {
  std::vector<RlePoint> points(1000);
  for (size_t i = 0; i < points.size(); ++i) {
    points[i] = {0.01F * float(i), 1.0F, 2.0F, uint16_t(i < 500 ? 0xBEEF : 0x1234)};
  }
  Cloudini::EncodingInfo info;
  info.width = uint32_t(points.size());
  info.height = 1;
  info.point_step = sizeof(RlePoint);
  info.encoding_opt = Cloudini::EncodingOptions::LOSSY;
  info.compression_opt = Cloudini::CompressionOption::NONE;
  info.fields.resize(4);
  const char* names[4] = {"x", "y", "z", "ring"};
  for (size_t k = 0; k < 4; ++k) {
    info.fields[k].name = names[k];
    info.fields[k].offset = uint32_t(4 * k);
    info.fields[k].type = k < 3 ? Cloudini::FieldType::FLOAT32 : Cloudini::FieldType::UINT16;
    if (k < 3) {
      info.fields[k].resolution = 0.001F;
    }
  }
  std::vector<uint8_t> msg;
  Cloudini::PointcloudEncoder(info).encode(
      Cloudini::ConstBufferView(reinterpret_cast<const uint8_t*>(points.data()), points.size() * sizeof(RlePoint)),
      msg);
  Cloudini::ConstBufferView view(msg.data(), msg.size());
  Cloudini::DecodeHeader(view);
  const size_t chunk_size_pos = msg.size() - view.size();

  // the encoder codes ring as two RLE runs: mode 2, u32 run count 2, [0xBEEF, 500], [0x1234, 500]
  const std::vector<uint8_t> tail = {2, 2, 0, 0, 0, 0xEF, 0xBE, 0xF4, 0x03, 0x34, 0x12, 0xF4, 0x03};
  if (msg.size() < tail.size() || !std::equal(tail.begin(), tail.end(), msg.end() - tail.size())) {
    throw std::logic_error("unexpected encoding of the ring section");
  }
  msg.resize(msg.size() - tail.size());
  // RLE: [0xBEEF, 1 point], [0xBEEF, second_run points]
  // Delta-RLE: [+5, 1 point], [+0, second_run points] (encodeVarint64: zig-zag + 1)
  std::vector<uint8_t> section = delta_rle ? std::vector<uint8_t>{3, 2, 0, 0, 0, 0x0B, 1, 0x01}
                                           : std::vector<uint8_t>{2, 2, 0, 0, 0, 0xEF, 0xBE, 1, 0xEF, 0xBE};
  for (uint64_t v = second_run;; v >>= 7) {
    section.push_back(uint8_t((v & 0x7F) | (v > 0x7F ? 0x80 : 0)));
    if (v <= 0x7F) {
      break;
    }
  }
  msg.insert(msg.end(), section.begin(), section.end());
  uint32_t chunk_size = 0;
  std::memcpy(&chunk_size, msg.data() + chunk_size_pos, sizeof(chunk_size));
  chunk_size = uint32_t(chunk_size - tail.size() + section.size());
  std::memcpy(msg.data() + chunk_size_pos, &chunk_size, sizeof(chunk_size));
  return msg;
}

}  // namespace

// A run length that makes out_index + run_len wrap around 2^64 must be rejected, not written past the
// chunk (heap overflow).
TEST(Cloudini, V5RleRunLengthOverflowIsRejected) {
  for (const bool delta_rle : {false, true}) {
    // well-formed: 1 + 999 points
    {
      const auto msg = craftTwoRuns(999, delta_rle);
      Cloudini::ConstBufferView view(msg.data(), msg.size());
      const auto header = Cloudini::DecodeHeader(view);
      std::vector<uint8_t> out;
      Cloudini::PointcloudDecoder().decode(header, view, out);
      ASSERT_EQ(out.size(), 1000 * sizeof(RlePoint));
    }
    for (const uint64_t second_run : {uint64_t(1000), ~uint64_t(0), ~uint64_t(0) - 5}) {
      const auto msg = craftTwoRuns(second_run, delta_rle);
      Cloudini::ConstBufferView view(msg.data(), msg.size());
      const auto header = Cloudini::DecodeHeader(view);
      std::vector<uint8_t> out;
      EXPECT_THROW(Cloudini::PointcloudDecoder().decode(header, view, out), std::runtime_error)
          << "delta_rle " << delta_rle << " second run " << second_run;
    }
  }
}
