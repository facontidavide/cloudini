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

struct RosHeader {
  int32_t stamp_sec = 0;    // seconds
  uint32_t stamp_nsec = 0;  // nanoseconds
  std::string frame_id;     // frame ID
};

/**
 * @brief This structure mimics the sensor_msgs/msg/PointCloud2 message fields
 *
 * @details:
 * sensor_msgs/msg/PointCloud2
 *     uint32 height
 *     uint32 width
 *     PointField[] fields
 *     bool    is_bigendian
 *     uint32  point_step
 *     uint32  row_step
 *     uint8[] data
 *     bool is_dense
 *     sensor_msgs/msg/PointField:
 *         string name
 *         uint32 offset
 *         uint8  datatype
 *         uint32 count
 *
 */
struct CloudiniPointCloud {
  nanocdr::CdrHeader cdr_header;
  RosHeader ros_header;                      // ROS header
  uint32_t height = 1;                       // default to unorganized point cloud
  uint32_t width = 0;                        // number of points when height == 1
  std::vector<Cloudini::PointField> fields;  // point fields
  uint32_t point_step = 0;                   // size of a single point in bytes
  uint32_t row_step = 0;                     // size of a single row in bytes (not used)
  bool is_bigendian = false;                 // endianness (not used)
  Cloudini::ConstBufferView data;
  bool is_dense = true;  // whether all points are valid
};

// Resolutions to be applied to the fields in CloudiniPointCloud or EncodingInfo.
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
    const ResolutionProfile& profile, std::vector<Cloudini::PointField>& field,
    std::optional<float> default_resolution = std::nullopt);

Cloudini::EncodingInfo toEncodingInfo(const CloudiniPointCloud& pc_info);

//------------------------------------------------------------------------

/**
 * @brief Get the Deserialized Point Cloud Message object, Extract information from a raw DDS message
 * (sensor_msgs/msg/PointCloud2) or (point_cloud_interfaces/msg/CompressedPointCloud2) into a CloudiniPointCloud
 * structure
 *
 * @param raw_dds_msg [in] serialized msg
 * @param cloudini_point_cloud [out] deserialized cloudini point cloud msg object.
 */
void getDeserializedPointCloudMessage(
    const Cloudini::ConstBufferView raw_dds_msg, CloudiniPointCloud& cloudini_point_cloud);

/**
 * @brief Given as input a raw DDS message, containing a sensor_msgs/msg/PointCloud2, apply compression and write the
 * result into a serialized point_cloud_interfaces/msg/CompressedPointCloud2
 *
 * @param cloudini_point_cloud [in] Point Cloud 2
 * @param encoding_info [in]
 * @param compressed_dds_msg [out] Serialized Compressed Point Cloud
 */
void convertPointCloud2ToCompressedCloud(
    const CloudiniPointCloud& cloudini_point_cloud, const Cloudini::EncodingInfo& encoding_info,
    std::vector<uint8_t>& compressed_dds_msg);

//
/**
 * @brief Assumining that cloudini_point_cloud contains compressed data, decompress it directly into pc2_dds_msg
 *
 * @param cloudini_point_cloud [in] Compressed Point Cloud
 * @param pc2_dds_msg [out] Serialized Point Cloud 2
 */
void convertCompressedCloudToPointCloud2(
    const CloudiniPointCloud& cloudini_point_cloud, std::vector<uint8_t>& pc2_dds_msg);

}  // namespace cloudini_ros
