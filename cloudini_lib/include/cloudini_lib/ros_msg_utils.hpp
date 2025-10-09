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
namespace cloudini_ros {

using BufferView = Cloudini::BufferView;
using ConstBufferView = Cloudini::ConstBufferView;
using FieldType = Cloudini::FieldType;
using PointField = Cloudini::PointField;
using EncodingOptions = Cloudini::EncodingOptions;
using CompressionOption = Cloudini::CompressionOption;
using EncodingInfo = Cloudini::EncodingInfo;

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

  virtual size_t requiredSize() const;
};

// This structure mimics the point_cloud_interfaces/msg/CompressedPointCloud2 message fields
struct RosCompressedPointCloud2 : public RosPointCloud2 {
  // inherited all the fields from RosPointCloud2
  std::string format;  // format of the compressed data
  size_t requiredSize() const override;
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

// Extract information from a raw DDS message (sensor_msgs/msg/PointCloud2) into a RosPointCloud2 structure
RosPointCloud2 parsePointCloud2Message(ConstBufferView pc2_dds_msg);

// Extract information from a raw DDS message (point_cloud_interfaces/msg/CompressedPointCloud2) into a
// RosCompressedPointCloud2 structure
RosCompressedPointCloud2 parseCompressedPointCloudMessage(ConstBufferView compressed_dds_msg);

//------------------------------------------------------------------------

// Given as input a raw DDS message, containing a sensor_msgs/msg/PointCloud2,
// apply compression and write the result into a point_cloud_interfaces/msg/CompressedPointCloud2
void convertPointCloud2ToCompressedCloud(
    const RosPointCloud2& pc_info, const Cloudini::EncodingInfo& encoding_info,
    std::vector<uint8_t>& compressed_dds_msg);

// Assumining that pc_info contains compressed data, decompress it directly into raw_dds_msg
void convertCompressedCloudToPointCloud2(const RosCompressedPointCloud2& pc_info, std::vector<uint8_t>& pc2_dds_msg);

}  // namespace cloudini_ros
