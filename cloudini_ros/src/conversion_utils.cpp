// Copyright (c) 2025 Davide Faconti
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
