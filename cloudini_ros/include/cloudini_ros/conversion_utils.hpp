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

#ifndef CLOUDINI_ROS__CONVERSION_UTILS_HPP_
#define CLOUDINI_ROS__CONVERSION_UTILS_HPP_

#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/ros_msg_utils.hpp"

namespace Cloudini {

/**
 * @brief Convert a PointCloud2 message to EncodingInfo
 * Default options (that can be overwitten later) are:
 * - encoding_opt = LOSSY
 * - compression_opt = ZSTD
 *
 * @param msg The PointCloud2 message to convert
 * @param resolution The resolution to use for FLOAT32 fields (XYZ, XYZI).
 * @return The EncodingInfo structure
 */
EncodingInfo ConvertToEncodingInfo(const sensor_msgs::msg::PointCloud2& msg, float resolution);

EncodingInfo ReadEncodingInfo(const point_cloud_interfaces::msg::CompressedPointCloud2& msg);

/**
 * @brief Convert a sensor_msgs::msg::PointCloud2 to a cloudini_ros::RosPointCloud2.
 *
 * The returned RosPointCloud2::data is a non-owning view into the input message's
 * data buffer. The caller must ensure the input message outlives the returned object.
 * The cdr_header field is left at its default value intentionally.
 *
 * @param msg The PointCloud2 message to convert
 * @return A RosPointCloud2 referencing the input message's data
 */
cloudini_ros::RosPointCloud2 ConvertToRosPointCloud2(const sensor_msgs::msg::PointCloud2& msg);

/**
 * @brief Compress a PointCloud2 and serialize as a CDR-encoded CompressedPointCloud2 DDS message.
 *
 * This is a convenience function that combines field conversion, resolution profiling,
 * encoding, and CDR serialization into a single call. The output can be published
 * directly via a generic publisher.
 *
 * @param msg The PointCloud2 message to compress
 * @param resolution The quantization resolution for FLOAT32 fields (e.g. 0.001 for 1mm)
 * @param serialized_dds_msg Output buffer for the CDR-serialized CompressedPointCloud2
 *
 * Example (simple):
 * @code
 *   std::vector<uint8_t> buffer;
 *   Cloudini::SerializeCompressedPointCloud2(pcd_msg, 0.001, buffer);
 * @endcode
 *
 * Example (zero-copy publish):
 * @code
 *   std::vector<uint8_t> buffer;  // keep as member to reuse allocation
 *   Cloudini::SerializeCompressedPointCloud2(pcd_msg, 0.001, buffer);
 *   rclcpp::SerializedMessage ser_msg;
 *   ser_msg.get_rcl_serialized_message().buffer = buffer.data();
 *   ser_msg.get_rcl_serialized_message().buffer_length = buffer.size();
 *   generic_publisher->publish(ser_msg);
 *   // null out buffer pointer before ser_msg destruction to prevent double-free
 *   ser_msg.get_rcl_serialized_message().buffer = nullptr;
 *   ser_msg.get_rcl_serialized_message().buffer_length = 0;
 * @endcode
 */
void SerializeCompressedPointCloud2(
    const sensor_msgs::msg::PointCloud2& msg, float resolution, std::vector<uint8_t>& serialized_dds_msg);

}  // namespace Cloudini
#endif  // CLOUDINI_ROS__CONVERSION_UTILS_HPP_
