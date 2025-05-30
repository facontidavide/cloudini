// Copyright (c) 2025 Davide Faconti
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <exception>
#include <optional>

#include <cloudini_ros/conversion_utils.hpp>

namespace Cloudini
{

EncodingInfo ConvertToEncodingInfo(const sensor_msgs::msg::PointCloud2 & msg, float resolution)
{
  EncodingInfo info;
  info.width = msg.width;
  info.height = msg.height;
  info.point_step = msg.point_step;
  info.encoding_opt = EncodingOptions::LOSSY;
  info.compression_opt = CompressionOption::ZSTD;

  for (const auto & msg_field : msg.fields) {
    PointField field;
    field.name = msg_field.name;
    field.offset = msg_field.offset;
    field.type = static_cast<FieldType>(msg_field.datatype);
    field.resolution =
      (field.type == FieldType::FLOAT32) ? std::optional<float>(resolution) : std::nullopt;
    info.fields.push_back(field);
  }
  return info;
}

EncodingInfo ReadEncodingInfo(const point_cloud_interfaces::msg::CompressedPointCloud2 & msg)
{
  // the encoding info are in the header of the data
  if (msg.format != "cloudini") {
    throw std::runtime_error("Invalid format. Expected 'cloudini'");
  }
  ConstBufferView data(msg.compressed_data.data(), msg.compressed_data.size());
  return DecodeHeader(data);
}

}  // namespace Cloudini
