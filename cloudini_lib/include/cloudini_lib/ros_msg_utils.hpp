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

#pragma once

#include <map>
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/contrib/nanocdr.hpp"
namespace Cloudini {

struct RosHeader {
  int32_t stamp_sec = 0;    // seconds
  uint32_t stamp_nsec = 0;  // nanoseconds
  std::string frame_id;     // frame ID
};

// This structure mimics the sensor_msgs/msg/PointCloud2 message fields
struct RosPointCloud2 {
  nanocdr::CdrHeader cdr_header;
  RosHeader ros_header;            // ROS header
  uint32_t height = 1;             // default to unorganized point cloud
  uint32_t width = 0;              // number of points when height == 1
  std::vector<PointField> fields;  // point fields
  uint32_t point_step = 0;         // size of a single point in bytes
  uint32_t row_step = 0;           // size of a single row in bytes (not used)
  bool is_bigendian = false;       // endianness (not used)
  ConstBufferView data;
  bool is_dense = true;  // whether all points are valid
};

// Resolutions to be applied to the fields in RosPointCloud2 or EncodingInfo.
// Note that a resolution is 0, the entire field will be removed
using ResolutionProfile = std::map<std::string, float>;

/**
 * @brief Apply a resolution profile to the fields in the point cloud.
 * If a field has a resolution of 0, it will be removed.
 *
 * @param profile The resolution profile to apply.
 * @param field The fields to apply the profile to.
 * @param default_resolution Optional default resolution to apply to FLOAT32 fields not in the profile.
 */
void applyResolutionProfile(
    const ResolutionProfile& profile, std::vector<PointField>& field,
    std::optional<float> default_resolution = std::nullopt);

EncodingInfo toEncodingInfo(const RosPointCloud2& pc_info);

RosPointCloud2 readPointCloud2Message(ConstBufferView raw_dds_msg);

//------------------------------------------------------------------------

// Add all the information from the RosPointCloud2 to the raw_dds_msg buffer, but skip info.data
void writePointCloud2Header(nanocdr::Encoder& encoder, const RosPointCloud2& info);

// Store all the information from the RosPointCloud2 to the raw_dds_msg buffer
void writePointCloud2Message(const RosPointCloud2& pc_info, std::vector<uint8_t>& raw_dds_msg, bool is_compressed_msg);

// Assumining that pc_info contains compressed data, decompress it directly into raw_dds_msg
void decompressAndWritePointCloud2Message(const RosPointCloud2& pc_info, std::vector<uint8_t>& raw_dds_msg);

}  // namespace Cloudini
