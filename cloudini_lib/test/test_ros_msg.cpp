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

#include <fstream>
#include <iostream>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/encoding_utils.hpp"
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

  for (size_t i = 0; i < encoding_info.width * encoding_info.height; ++i) {
    float original_x = *reinterpret_cast<const float*>(original_data_ptr + offset + fields[0].offset);
    float original_y = *reinterpret_cast<const float*>(original_data_ptr + offset + fields[1].offset);
    float original_z = *reinterpret_cast<const float*>(original_data_ptr + offset + fields[2].offset);
    float original_intensity = *reinterpret_cast<const float*>(original_data_ptr + offset + fields[3].offset);
    uint16_t original_ring = *reinterpret_cast<const uint16_t*>(original_data_ptr + offset + fields[4].offset);
    double original_timestamp = *reinterpret_cast<const double*>(original_data_ptr + offset + fields[5].offset);

    float decoded_x = *reinterpret_cast<const float*>(decoded_data_ptr + offset + fields[0].offset);
    float decoded_y = *reinterpret_cast<const float*>(decoded_data_ptr + offset + fields[1].offset);
    float decoded_z = *reinterpret_cast<const float*>(decoded_data_ptr + offset + fields[2].offset);
    float decoded_intensity = *reinterpret_cast<const float*>(decoded_data_ptr + offset + fields[3].offset);
    uint16_t decoded_ring = *reinterpret_cast<const uint16_t*>(decoded_data_ptr + offset + fields[4].offset);
    double decoded_timestamp = *reinterpret_cast<const double*>(decoded_data_ptr + offset + fields[5].offset);

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

  std::vector<uint8_t> raw_dds_msg;
  {
    std::ifstream file(filepath, std::ios::binary);
    ASSERT_TRUE(file.is_open()) << "Failed to open file: " << filepath;

    file.seekg(0, std::ios::end);
    raw_dds_msg.resize(file.tellg());
    file.seekg(0, std::ios::beg);
    file.read(reinterpret_cast<char*>(raw_dds_msg.data()), raw_dds_msg.size());
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
  const auto pc_info = cloudini_ros::parsePointCloudMessage(raw_dds_msg);
  EncodingInfo encoding_info = cloudini_ros::toEncodingInfo(pc_info);

  CompareInfos(pc_info, expected_infos);
  CompareInfos(encoding_info, expected_infos);

  encoding_info.fields[0].resolution = resolution;  // Set resolution for x
  encoding_info.fields[1].resolution = resolution;  // Set resolution for y
  encoding_info.fields[2].resolution = resolution;  // Set resolution for z
  encoding_info.fields[3].resolution = resolution;  // Set resolution for intensity

  //-----------------------------------------------------------------------------
  const std::vector<uint8_t> original_data(
      pc_info.data.data(), pc_info.data.data() + pc_info.data.size());

  VerifyRoundTrip(encoding_info, original_data, resolution);
}
