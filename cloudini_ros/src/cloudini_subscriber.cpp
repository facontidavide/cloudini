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

#include "cloudini_plugin/cloudini_subscriber.hpp"

#include "cloudini_lib/cloudini.hpp"

namespace cloudini_point_cloud_transport {

CloudiniSubscriber::CloudiniSubscriber() : decoder_(std::make_shared<Cloudini::PointcloudDecoder>()) {
  // Constructor implementation
}

CloudiniSubscriber::DecodeResult CloudiniSubscriber::decodeTyped(
    const point_cloud_interfaces::msg::CompressedPointCloud2& msg) const {
  Cloudini::ConstBufferView input(msg.compressed_data.data(), msg.compressed_data.size());
  auto info = Cloudini::DecodeHeader(input);

  //----------- header sanity check -------------------
  if (msg.width != info.width || msg.height != info.height) {
    throw std::runtime_error("CloudiniSubscriber: wrong point cloud dimensions");
  }
  if (msg.point_step != info.point_step) {
    throw std::runtime_error("CloudiniSubscriber: wrong point step");
  }

  if (msg.fields.size() != info.fields.size()) {
    throw std::runtime_error("CloudiniSubscriber: wrong fields count");
  }

  for (size_t i = 0; i < msg.fields.size(); ++i) {
    const auto& msg_field = msg.fields[i];
    const auto& info_field = info.fields[i];
    if (msg_field.name != info_field.name) {
      throw std::runtime_error("CloudiniSubscriber: wrong field name");
    }
    if (msg_field.offset != info_field.offset) {
      throw std::runtime_error("CloudiniSubscriber: wrong field offset");
    }
    if (msg_field.datatype != static_cast<uint8_t>(info_field.type)) {
      throw std::runtime_error("CloudiniSubscriber: wrong field type");
    }
  }

  //----------- actual decoding -------------------
  auto result = std::make_shared<sensor_msgs::msg::PointCloud2>();

  result->header = msg.header;
  result->height = info.height;
  result->width = info.width;
  result->fields = msg.fields;
  result->is_bigendian = false;
  result->point_step = msg.point_step;
  result->row_step = msg.row_step;
  result->is_dense = msg.is_dense;

  // prepare buffer for decompression
  result->data.resize(msg.point_step * msg.width * msg.height);
  Cloudini::BufferView output(result->data.data(), result->data.size());

  decoder_->decode(info, input, output);

  return result;
}

}  // namespace cloudini_point_cloud_transport
