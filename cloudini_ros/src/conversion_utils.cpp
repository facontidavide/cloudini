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

#include <cloudini_ros/conversion_utils.hpp>
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

}  // namespace Cloudini
