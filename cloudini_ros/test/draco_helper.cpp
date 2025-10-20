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

#include "draco_helper.hpp"

#ifdef DRACO_FOUND

#include <draco/compression/encode.h>
#include <draco/compression/expert_encode.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <chrono>

void compressDraco(
    const sensor_msgs::msg::PointCloud2& msg, const Cloudini::EncodingInfo& encoding_info, StatsData& stats) {
  // DRACO doesn't allow NaN points, so we should filter them out first.
  sensor_msgs::msg::PointCloud2 filtered_msg = msg;
  // Remove NAN points
  filtered_msg.width = 0;
  filtered_msg.height = 1;
  filtered_msg.data.clear();
  for (size_t i = 0; i < msg.data.size(); i += encoding_info.point_step) {
    const float* x_ptr = reinterpret_cast<const float*>(msg.data.data() + i + encoding_info.fields[0].offset);
    const float* y_ptr = reinterpret_cast<const float*>(msg.data.data() + i + encoding_info.fields[1].offset);
    const float* z_ptr = reinterpret_cast<const float*>(msg.data.data() + i + encoding_info.fields[2].offset);
    if (!std::isnan(*x_ptr) && !std::isnan(*y_ptr) && !std::isnan(*z_ptr)) {
      filtered_msg.data.insert(
          filtered_msg.data.end(), msg.data.begin() + i, msg.data.begin() + i + encoding_info.point_step);
      filtered_msg.width++;
    }
  }

  const size_t num_points = filtered_msg.width * filtered_msg.height;

  if (encoding_info.fields.size() < 3) {
    return;  // Need at least 3 fields for XYZ
  }

  // Create Draco point cloud
  draco::PointCloudBuilder builder;
  builder.Start(num_points);

  // First 3 fields are always POSITION (x, y, z)
  const int pos_att_id = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);

  // Helper lambda to map Cloudini field type to Draco data type
  auto toDracoType = [](Cloudini::FieldType type) -> draco::DataType {
    switch (type) {
      case Cloudini::FieldType::FLOAT32:
        return draco::DT_FLOAT32;
      case Cloudini::FieldType::FLOAT64:
        return draco::DT_FLOAT64;
      case Cloudini::FieldType::INT8:
        return draco::DT_INT8;
      case Cloudini::FieldType::UINT8:
        return draco::DT_UINT8;
      case Cloudini::FieldType::INT16:
        return draco::DT_INT16;
      case Cloudini::FieldType::UINT16:
        return draco::DT_UINT16;
      case Cloudini::FieldType::INT32:
        return draco::DT_INT32;
      case Cloudini::FieldType::UINT32:
        return draco::DT_UINT32;
      case Cloudini::FieldType::UINT64:
        return draco::DT_UINT64;
      case Cloudini::FieldType::INT64:
        return draco::DT_INT64;
      default:
        return draco::DT_FLOAT32;
    }
  };

  // Store attribute IDs for remaining fields (starting from 4th field)
  std::vector<int> generic_att_ids;

  for (size_t i = 3; i < encoding_info.fields.size(); ++i) {
    const auto& field = encoding_info.fields[i];
    draco::DataType draco_type = toDracoType(field.type);
    int att_id = builder.AddAttribute(draco::GeometryAttribute::GENERIC, 1, draco_type);
    generic_att_ids.push_back(att_id);
  }

  // Use SetAttributeValuesForAllPoints with stride (no need to de-interleave!)
  const size_t stride = encoding_info.point_step;
  const uint8_t* data_ptr = filtered_msg.data.data();

  // Verify we have enough data
  const size_t expected_size = num_points * stride;
  if (filtered_msg.data.size() < expected_size) {
    std::cerr << "Error: Point cloud data size mismatch. Expected at least " << expected_size << " bytes, got "
              << filtered_msg.data.size() << " bytes" << std::endl;
    return;
  }

  // Set position (XYZ) - pointer to first field offset (x, y, z are first 3 fields)
  const uint8_t* xyz_ptr = data_ptr + encoding_info.fields[0].offset;
  builder.SetAttributeValuesForAllPoints(pos_att_id, xyz_ptr, stride);

  // Set remaining fields - pointer to field offset
  for (size_t i = 3; i < encoding_info.fields.size(); ++i) {
    const auto& field = encoding_info.fields[i];
    const size_t attr_idx = i - 3;
    const uint8_t* field_ptr = data_ptr + field.offset;
    builder.SetAttributeValuesForAllPoints(generic_att_ids[attr_idx], field_ptr, stride);
  }

  std::unique_ptr<draco::PointCloud> draco_pc = builder.Finalize(false);
  if (!draco_pc) {
    std::cerr << "Error: Failed to finalize Draco point cloud" << std::endl;
    return;
  }

  // Encode the point cloud with 16-bit quantization and sequential encoding
  draco::ExpertEncoder encoder(*draco_pc);
  encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 16);

  // Set quantization for all generic attributes (if any)
  if (!generic_att_ids.empty()) {
    encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, 16);
  }

  encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);

  static draco::EncoderBuffer buffer;
  buffer.Clear();  // Ensure buffer is initialized

  auto t1 = std::chrono::high_resolution_clock::now();
  encoder.EncodeToBuffer(&buffer);
  auto t2 = std::chrono::high_resolution_clock::now();

  stats.total_time_usec += std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
  stats.total_ratio += double(buffer.size()) / double(filtered_msg.data.size());
}

#endif  // DRACO_FOUND
