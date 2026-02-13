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

#include "cloudini_lib/ros_msg_utils.hpp"
#include "cloudini_ros/conversion_utils.hpp"
#include <exception>
#include <optional>

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
  cloudini_ros::RosPointCloud2 result;
  
  // Convert header
  result.ros_header.stamp_sec = msg.header.stamp.sec;
  result.ros_header.stamp_nsec = msg.header.stamp.nanosec;
  result.ros_header.frame_id = msg.header.frame_id;
  
  // Copy point cloud metadata
  result.height = msg.height;
  result.width = msg.width;
  result.point_step = msg.point_step;
  result.row_step = msg.row_step;
  result.is_bigendian = msg.is_bigendian;
  result.is_dense = msg.is_dense;
  
  // Convert fields
  result.fields.reserve(msg.fields.size());
  for (const auto& field : msg.fields) {
    Cloudini::PointField pf;
    pf.name = field.name;
    pf.offset = field.offset;
    pf.type = static_cast<Cloudini::FieldType>(field.datatype);
    result.fields.push_back(std::move(pf));
  }
  
  // Set data view (no copy, just a view)
  result.data = Cloudini::ConstBufferView(msg.data.data(), msg.data.size());
  
  return result;
}

rclcpp::SerializedMessage ConvertToCompressedCloud(const sensor_msgs::msg::PointCloud2& msg, float resolution) {
  // Convert to RosPointCloud2
  auto pc_info = ConvertToRosPointCloud2(msg);
  
  // Apply resolution profile
  cloudini_ros::applyResolutionProfile(
    cloudini_ros::ResolutionProfile{}, pc_info.fields,
    resolution);

  // Create encoding info and compress
  const auto encoding_info = cloudini_ros::toEncodingInfo(pc_info);
  std::vector<uint8_t> compressed_buffer;
  cloudini_ros::convertPointCloud2ToCompressedCloud(pc_info, encoding_info, compressed_buffer);

  // Create serialized message
  rclcpp::SerializedMessage output_message(compressed_buffer.size());
  auto& rcl_msg = output_message.get_rcl_serialized_message();
  std::memcpy(rcl_msg.buffer, compressed_buffer.data(), compressed_buffer.size());
  rcl_msg.buffer_length = compressed_buffer.size();
  
  return output_message;
}


}  // namespace Cloudini
