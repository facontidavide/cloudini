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

#include "cloudini_ros/conversion_utils.hpp"

#include <exception>
#include <optional>

#include "cloudini_lib/ros_msg_utils.hpp"

namespace Cloudini {

EncodingInfo ConvertToEncodingInfo(const sensor_msgs::msg::PointCloud2& msg, float resolution) {
  EncodingInfo info;
  info.width = msg.width;
  info.height = msg.height;
  info.point_step = msg.point_step;
  info.encoding_opt = EncodingOptions::LOSSY;
  info.compression_opt = CompressionOption::ZSTD;

  for (const auto& msg_field : msg.fields) {
    PointField field;
    field.name = msg_field.name;
    field.offset = msg_field.offset;
    field.type = static_cast<FieldType>(msg_field.datatype);
    field.resolution = (field.type == FieldType::FLOAT32) ? std::optional<float>(resolution) : std::nullopt;
    info.fields.push_back(field);
  }
  return info;
}

EncodingInfo ReadEncodingInfo(const point_cloud_interfaces::msg::CompressedPointCloud2& msg) {
  // the encoding info are in the header of the data
  if (msg.format != "cloudini") {
    throw std::runtime_error("Invalid format. Expected 'cloudini'");
  }
  ConstBufferView data(msg.compressed_data.data(), msg.compressed_data.size());
  return DecodeHeader(data);
}

cloudini_ros::RosPointCloud2 ConvertToRosPointCloud2(const sensor_msgs::msg::PointCloud2& msg) {
  cloudini_ros::RosPointCloud2 pc_info;
  pc_info.ros_header.stamp_sec = msg.header.stamp.sec;
  pc_info.ros_header.stamp_nsec = msg.header.stamp.nanosec;
  pc_info.ros_header.frame_id = msg.header.frame_id;
  pc_info.height = msg.height;
  pc_info.width = msg.width;
  pc_info.point_step = msg.point_step;
  pc_info.row_step = msg.row_step;
  pc_info.is_bigendian = msg.is_bigendian;
  pc_info.is_dense = msg.is_dense;
  pc_info.data = ConstBufferView(msg.data.data(), msg.data.size());

  for (const auto& msg_field : msg.fields) {
    PointField field;
    field.name = msg_field.name;
    field.offset = msg_field.offset;
    field.type = static_cast<FieldType>(msg_field.datatype);
    pc_info.fields.push_back(field);
  }
  return pc_info;
}

void SerializeCompressedPointCloud2(
    const sensor_msgs::msg::PointCloud2& msg, float resolution, std::vector<uint8_t>& serialized_dds_msg) {
  auto pc_info = ConvertToRosPointCloud2(msg);
  cloudini_ros::applyResolutionProfile(cloudini_ros::ResolutionProfile{}, pc_info.fields, resolution);
  const auto encoding_info = cloudini_ros::toEncodingInfo(pc_info);
  cloudini_ros::convertPointCloud2ToCompressedCloud(pc_info, encoding_info, serialized_dds_msg);
}

}  // namespace Cloudini
