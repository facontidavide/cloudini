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

#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/encoding_utils.hpp"
#include "cloudini_lib/ros_message_definitions.hpp"  // also included by test_header.cpp: must not break the link
#include "cloudini_lib/ros_msg_utils.hpp"
#include "data_path.hpp"

using namespace Cloudini;

template <typename TypeA, typename TypeB>
void CompareInfos(const TypeA& a, const TypeB& b) {
  ASSERT_EQ(a.width, b.width);
  ASSERT_EQ(a.height, b.height);
  ASSERT_EQ(a.point_step, b.point_step);
  ASSERT_EQ(a.point_step, b.point_step);
  ASSERT_EQ(a.fields.size(), b.fields.size());

  for (size_t i = 0; i < a.fields.size(); ++i) {
    ASSERT_EQ(a.fields[i].name, b.fields[i].name);
    ASSERT_EQ(a.fields[i].type, b.fields[i].type);
    ASSERT_EQ(a.fields[i].offset, b.fields[i].offset);
  }
}

void VerifyRoundTrip(const EncodingInfo& encoding_info, const std::vector<uint8_t>& original_data, float resolution) {
  // Encode the point cloud data

  PointcloudEncoder pc_encoder(encoding_info);
  std::vector<uint8_t> compressed_data;
  pc_encoder.encode(original_data, compressed_data);

  // Decode the point cloud data
  PointcloudDecoder pc_decoder;
  std::vector<uint8_t> decoded_data;
  ConstBufferView compressed_view(compressed_data.data(), compressed_data.size());
  auto recovered_header = DecodeHeader(compressed_view);
  pc_decoder.decode(recovered_header, compressed_view, decoded_data);

  CompareInfos(encoding_info, recovered_header);
  const uint8_t* original_data_ptr = original_data.data();
  const uint8_t* decoded_data_ptr = decoded_data.data();
  int offset = 0;

  const auto& fields = encoding_info.fields;

  // memcpy: the fields of this packed 26-byte point layout are not naturally aligned
  auto load = [](const uint8_t* ptr, auto& value) { memcpy(&value, ptr, sizeof(value)); };

  for (size_t i = 0; i < encoding_info.width * encoding_info.height; ++i) {
    float original_x, original_y, original_z, original_intensity;
    float decoded_x, decoded_y, decoded_z, decoded_intensity;
    uint16_t original_ring, decoded_ring;
    double original_timestamp, decoded_timestamp;
    load(original_data_ptr + offset + fields[0].offset, original_x);
    load(original_data_ptr + offset + fields[1].offset, original_y);
    load(original_data_ptr + offset + fields[2].offset, original_z);
    load(original_data_ptr + offset + fields[3].offset, original_intensity);
    load(original_data_ptr + offset + fields[4].offset, original_ring);
    load(original_data_ptr + offset + fields[5].offset, original_timestamp);
    load(decoded_data_ptr + offset + fields[0].offset, decoded_x);
    load(decoded_data_ptr + offset + fields[1].offset, decoded_y);
    load(decoded_data_ptr + offset + fields[2].offset, decoded_z);
    load(decoded_data_ptr + offset + fields[3].offset, decoded_intensity);
    load(decoded_data_ptr + offset + fields[4].offset, decoded_ring);
    load(decoded_data_ptr + offset + fields[5].offset, decoded_timestamp);

    ASSERT_NEAR(original_x, decoded_x, resolution) << "Point index: " << i;
    ASSERT_NEAR(original_y, decoded_y, resolution) << "Point index: " << i;
    ASSERT_NEAR(original_z, decoded_z, resolution) << "Point index: " << i;
    ASSERT_NEAR(original_intensity, decoded_intensity, resolution) << "Point index: " << i;
    ASSERT_EQ(original_ring, decoded_ring) << "Point index: " << i;
    ASSERT_EQ(original_timestamp, decoded_timestamp) << "Point index: " << i;

    offset += recovered_header.point_step;
  }
}

TEST(Cloudini, DDS_Roundtrip) {
  const std::string filepath = Cloudini::tests::DATA_PATH + "dds_message.bin";

  std::vector<uint8_t> dds_pointcloud_msg;
  {
    std::ifstream file(filepath, std::ios::binary);
    ASSERT_TRUE(file.is_open()) << "Failed to open file: " << filepath;

    file.seekg(0, std::ios::end);
    dds_pointcloud_msg.resize(file.tellg());
    file.seekg(0, std::ios::beg);
    file.read(reinterpret_cast<char*>(dds_pointcloud_msg.data()), dds_pointcloud_msg.size());
    file.close();
  }

  const float resolution = 0.001f;

  using namespace Cloudini;

  const EncodingInfo expected_infos = {
      .fields =
          {
              {"x", 0, FieldType::FLOAT32, resolution},
              {"y", 4, FieldType::FLOAT32, resolution},
              {"z", 8, FieldType::FLOAT32, resolution},
              {"intensity", 12, FieldType::FLOAT32, resolution},
              {"ring", 16, FieldType::UINT16},
              {"timestamp", 18, FieldType::FLOAT64},
          },
      .width = 64000,
      .height = 1,
      .point_step = 26,
      .encoding_opt = EncodingOptions::LOSSY,
      .compression_opt = CompressionOption::ZSTD,
      .version = kEncodingVersion};

  //-----------------------------------------------------------------------------
  // read the DDS message
  const auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(dds_pointcloud_msg);
  EncodingInfo encoding_info = cloudini_ros::toEncodingInfo(pc_info);

  CompareInfos(pc_info, expected_infos);
  CompareInfos(encoding_info, expected_infos);

  encoding_info.fields[0].resolution = resolution;  // Set resolution for x
  encoding_info.fields[1].resolution = resolution;  // Set resolution for y
  encoding_info.fields[2].resolution = resolution;  // Set resolution for z
  encoding_info.fields[3].resolution = resolution;  // Set resolution for intensity

  //-----------------------------------------------------------------------------
  const std::vector<uint8_t> original_data(pc_info.data.data(), pc_info.data.data() + pc_info.data.size());

  VerifyRoundTrip(encoding_info, original_data, resolution);
}

TEST(Cloudini, RosPointCloud2CopyRebindsOwnedDataView) {
  cloudini_ros::RosPointCloud2 original;
  original.owned_data = {1, 2, 3, 4};
  original.data = ConstBufferView(original.owned_data.data(), original.owned_data.size());

  const cloudini_ros::RosPointCloud2 copied = original;

  ASSERT_EQ(copied.owned_data, original.owned_data);
  EXPECT_EQ(copied.data.data(), copied.owned_data.data());
  EXPECT_EQ(copied.data.size(), copied.owned_data.size());
}

namespace {
std::vector<uint8_t> loadSampleDDSMessage() {
  std::ifstream file(Cloudini::tests::DATA_PATH + "dds_message.bin", std::ios::binary);
  EXPECT_TRUE(file.is_open());
  return std::vector<uint8_t>((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}
}  // namespace

// A PointCloud2 can come from any DDS peer. Metadata that disagrees with the
// payload must be rejected, not encoded: the field encoders read at
// field.offset inside every point, and the header repeats width/height.
TEST(Cloudini, InconsistentPointCloud2IsRejected) {
  const auto dds_msg = loadSampleDDSMessage();
  std::vector<uint8_t> output;

  {
    auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(dds_msg);
    pc_info.fields.back().offset = pc_info.point_step;  // field lies past the end of the point
    const auto info = cloudini_ros::toEncodingInfo(pc_info);
    EXPECT_THROW(cloudini_ros::convertPointCloud2ToCompressedCloud(pc_info, info, output), std::runtime_error);
  }
  {
    auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(dds_msg);
    pc_info.width *= 2;  // header would promise twice the points that are encoded
    const auto info = cloudini_ros::toEncodingInfo(pc_info);
    EXPECT_THROW(cloudini_ros::convertPointCloud2ToCompressedCloud(pc_info, info, output), std::runtime_error);
  }
  {
    auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(dds_msg);
    const auto info = cloudini_ros::toEncodingInfo(pc_info);
    EXPECT_NO_THROW(cloudini_ros::convertPointCloud2ToCompressedCloud(pc_info, info, output));
  }
}

// The encoder itself must refuse a field that does not fit in point_step,
// whoever the caller is (PCL, Python, WASM bindings).
TEST(Cloudini, EncoderRejectsFieldOutsidePointStep) {
  Cloudini::EncodingInfo info;
  info.fields = {{"x", 0, Cloudini::FieldType::FLOAT32, 0.001f}, {"y", 6, Cloudini::FieldType::FLOAT32, 0.001f}};
  info.point_step = 8;  // y would span bytes 6..9
  info.width = 10;
  EXPECT_THROW(Cloudini::PointcloudEncoder encoder(info), std::runtime_error);
}

// Issue #135: many ROS drivers store packed RGB(A) as uint32 bits reinterpreted into a
// FLOAT32 field named "rgb" / "rgba". Such fields must never be quantized by default.
TEST(Cloudini, PackedColorFieldName) {
  EXPECT_TRUE(Cloudini::isPackedColorField("rgb"));
  EXPECT_TRUE(Cloudini::isPackedColorField("rgba"));
  EXPECT_TRUE(Cloudini::isPackedColorField("RGB"));
  EXPECT_TRUE(Cloudini::isPackedColorField("Rgba"));
  EXPECT_TRUE(Cloudini::isPackedColorField("bgra"));
  EXPECT_TRUE(Cloudini::isPackedColorField("argb"));
  EXPECT_FALSE(Cloudini::isPackedColorField("x"));
  EXPECT_FALSE(Cloudini::isPackedColorField("intensity"));
  EXPECT_FALSE(Cloudini::isPackedColorField("rgb_x"));
  EXPECT_FALSE(Cloudini::isPackedColorField(""));
}

TEST(Cloudini, ResolutionProfileKeepsPackedColorLossless) {
  std::vector<Cloudini::PointField> fields = {
      {"x", 0, FieldType::FLOAT32, std::nullopt},     {"y", 4, FieldType::FLOAT32, std::nullopt},
      {"z", 8, FieldType::FLOAT32, std::nullopt},     {"rgb", 12, FieldType::FLOAT32, std::nullopt},
      {"RGBA", 16, FieldType::FLOAT32, std::nullopt}, {"intensity", 20, FieldType::FLOAT32, std::nullopt},
  };
  cloudini_ros::applyResolutionProfile({}, fields, 0.001f);
  EXPECT_EQ(fields[0].resolution, std::optional<float>(0.001f));
  EXPECT_EQ(fields[1].resolution, std::optional<float>(0.001f));
  EXPECT_EQ(fields[2].resolution, std::optional<float>(0.001f));
  EXPECT_FALSE(fields[3].resolution.has_value());
  EXPECT_FALSE(fields[4].resolution.has_value());
  EXPECT_EQ(fields[5].resolution, std::optional<float>(0.001f));

  // An explicit profile entry still wins.
  std::vector<Cloudini::PointField> fields2 = {{"rgb", 0, FieldType::FLOAT32, std::nullopt}};
  cloudini_ros::applyResolutionProfile({{"rgb", 0.5f}}, fields2, 0.001f);
  EXPECT_EQ(fields2[0].resolution, std::optional<float>(0.5f));
}

// x,y,z,rgb (all FLOAT32) encoded with a default resolution: xyz is quantized, rgb must be
// bit-exact (and must not be swallowed into the 4-float SIMD lossy group).
TEST(Cloudini, PackedRgbRoundtripIsBitExact) {
  constexpr size_t kNumPoints = 1000;
  constexpr float kResolution = 0.001f;

  EncodingInfo info;
  info.width = kNumPoints;
  info.height = 1;
  info.point_step = 16;
  info.encoding_opt = EncodingOptions::LOSSY;
  info.compression_opt = CompressionOption::ZSTD;
  info.fields = {
      {"x", 0, FieldType::FLOAT32, std::nullopt},
      {"y", 4, FieldType::FLOAT32, std::nullopt},
      {"z", 8, FieldType::FLOAT32, std::nullopt},
      {"rgb", 12, FieldType::FLOAT32, std::nullopt},
  };
  cloudini_ros::applyResolutionProfile({}, info.fields, kResolution);

  // Includes patterns that are NaN when viewed as float (alpha = 0xFF).
  const uint32_t colors[] = {0x00FF8040, 0x00000001, 0x0012AB34, 0xFFFF8040, 0xFF0000FF, 0x7F7F7F7F, 0x00000000};
  constexpr size_t kNumColors = sizeof(colors) / sizeof(colors[0]);

  std::vector<uint8_t> data(kNumPoints * info.point_step);
  for (size_t i = 0; i < kNumPoints; ++i) {
    uint8_t* pt = data.data() + i * info.point_step;
    const float xyz[3] = {0.01f * i, -0.02f * i + 3.0f, 0.5f + 0.003f * (i % 17)};
    std::memcpy(pt, xyz, sizeof(xyz));
    const uint32_t color = colors[i % kNumColors];
    std::memcpy(pt + 12, &color, sizeof(color));
  }

  std::vector<uint8_t> compressed;
  PointcloudEncoder encoder(info);
  encoder.encode(ConstBufferView(data.data(), data.size()), compressed);

  ConstBufferView compressed_view(compressed.data(), compressed.size());
  const auto header = DecodeHeader(compressed_view);
  ASSERT_EQ(header.fields.size(), 4u);
  EXPECT_FALSE(header.fields[3].resolution.has_value());

  std::vector<uint8_t> decoded;
  PointcloudDecoder decoder;
  decoder.decode(header, compressed_view, decoded);
  ASSERT_EQ(decoded.size(), data.size());

  for (size_t i = 0; i < kNumPoints; ++i) {
    const uint8_t* orig = data.data() + i * info.point_step;
    const uint8_t* dec = decoded.data() + i * info.point_step;
    for (int k = 0; k < 3; ++k) {
      float a = 0;
      float b = 0;
      std::memcpy(&a, orig + 4 * k, 4);
      std::memcpy(&b, dec + 4 * k, 4);
      ASSERT_NEAR(a, b, kResolution) << "point " << i << " axis " << k;
    }
    uint32_t color_orig = 0;
    uint32_t color_dec = 0;
    std::memcpy(&color_orig, orig + 12, 4);
    std::memcpy(&color_dec, dec + 12, 4);
    ASSERT_EQ(color_orig, color_dec) << "point " << i;
  }
}
