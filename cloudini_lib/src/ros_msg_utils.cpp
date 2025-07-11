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

#include <algorithm>

namespace Cloudini {

// sensor_msgs/msg/PointCloud2
//
//     uint32 height
//     uint32 width
//     PointField[] fields
//     bool    is_bigendian
//     uint32  point_step
//     uint32  row_step
//     uint8[] data
//     bool is_dense
//
// sensor_msgs/msg/PointField:
//
//     string name
//     uint32 offset
//     uint8  datatype
//     uint32 count

RosPointCloud2 readPointCloud2Message(ConstBufferView raw_dds_msg) {
  nanocdr::Decoder cdr(raw_dds_msg);

  RosPointCloud2 result;

  //----- read the header -----
  cdr.decode(result.ros_header.stamp_sec);
  cdr.decode(result.ros_header.stamp_nsec);
  cdr.decode(result.ros_header.frame_id);

  //----- pointcloud info -----
  cdr.decode(result.height);
  cdr.decode(result.width);

  //----- fields -----
  uint32_t num_fields = 0;
  cdr.decode(num_fields);

  for (uint32_t i = 0; i < num_fields; ++i) {
    PointField field;
    cdr.decode(field.name);
    cdr.decode(field.offset);
    uint8_t type = 0;
    cdr.decode(type);
    field.type = static_cast<FieldType>(type);

    uint32_t count = 0;  // not used
    cdr.decode(count);

    result.fields.push_back(std::move(field));
  }
  bool is_bigendian = false;  // not used
  cdr.decode(is_bigendian);

  cdr.decode(result.point_step);
  cdr.decode(result.row_step);

  cdr.decode(result.data);
  cdr.decode(result.is_dense);

  result.cdr_header = cdr.header();
  return result;
}

RosCompressedPointCloud2 readCompressedPointCloud2Message(ConstBufferView raw_dds_msg) {
  nanocdr::Decoder cdr(raw_dds_msg);

  RosCompressedPointCloud2 result;

  //----- read the header -----
  cdr.decode(result.ros_header.stamp_sec);
  cdr.decode(result.ros_header.stamp_nsec);
  cdr.decode(result.ros_header.frame_id);

  //----- pointcloud info -----
  cdr.decode(result.height);
  cdr.decode(result.width);

  //----- fields -----
  uint32_t num_fields = 0;
  cdr.decode(num_fields);

  for (uint32_t i = 0; i < num_fields; ++i) {
    PointField field;
    cdr.decode(field.name);
    cdr.decode(field.offset);
    uint8_t type = 0;
    cdr.decode(type);
    field.type = static_cast<FieldType>(type);

    uint32_t count = 0;  // not used
    cdr.decode(count);

    result.fields.push_back(std::move(field));
  }
  bool is_bigendian = false;  // not used
  cdr.decode(is_bigendian);

  cdr.decode(result.point_step);
  cdr.decode(result.row_step);

  cdr.decode(result.compressed_data);
  cdr.decode(result.is_dense);
  cdr.decode(result.format);

  result.cdr_header = cdr.header();
  return result;
}

void writePointCloud2Header(nanocdr::Encoder& encoder, const RosPointCloud2& pc_info) {
  //----- write the header -----
  encoder.encode(pc_info.ros_header.stamp_sec);   // header_stamp_sec
  encoder.encode(pc_info.ros_header.stamp_nsec);  // header_stamp_nsec
  encoder.encode(pc_info.ros_header.frame_id);    // frame_id

  //----- pointcloud info -----
  encoder.encode(pc_info.height);
  encoder.encode(pc_info.width);

  //----- fields -----
  encoder.encode(static_cast<uint32_t>(pc_info.fields.size()));
  for (const auto& field : pc_info.fields) {
    encoder.encode(field.name);
    encoder.encode(field.offset);
    encoder.encode(static_cast<uint8_t>(field.type));
    encoder.encode(static_cast<uint32_t>(1));  // count, not used
  }

  encoder.encode(false);  // is_bigendian, not used
  encoder.encode(pc_info.point_step);
  encoder.encode(static_cast<uint32_t>(pc_info.point_step * pc_info.width));
}

void writePointCloud2Message(const RosPointCloud2& pc_info, std::vector<uint8_t>& raw_dds_msg, bool is_compressed_msg) {
  nanocdr::Encoder cdr(pc_info.cdr_header, raw_dds_msg);
  writePointCloud2Header(cdr, pc_info);
  cdr.encode(pc_info.data);
  cdr.encode(pc_info.is_dense);
  if (is_compressed_msg) {
    cdr.encode(std::string("cloudini"));
  }
}

EncodingInfo toEncodingInfo(const RosPointCloud2& pc_info) {
  EncodingInfo info;
  info.height = pc_info.height;
  info.width = pc_info.width;
  info.point_step = pc_info.point_step;
  info.encoding_opt = EncodingOptions::LOSSY;      // default to lossy encoding
  info.compression_opt = CompressionOption::ZSTD;  // default to ZSTD compression
  info.fields = pc_info.fields;
  return info;
}

void decompressAndWritePointCloud2Message(const RosPointCloud2& pc_info, std::vector<uint8_t>& raw_dds_msg) {
  nanocdr::Encoder cdr_encoder(pc_info.cdr_header, raw_dds_msg);
  writePointCloud2Header(cdr_encoder, pc_info);

  const size_t cloud_size = pc_info.width * pc_info.height * pc_info.point_step;
  cdr_encoder.encode(static_cast<uint32_t>(cloud_size));

  // Reserve enough memory for the decode cloud and take a buffer view to that memory
  size_t prev_size = raw_dds_msg.size();
  raw_dds_msg.resize(prev_size + cloud_size);
  BufferView decoded_cloud_view(raw_dds_msg.data() + prev_size, cloud_size);

  Cloudini::PointcloudDecoder cloud_decoder;
  Cloudini::ConstBufferView compressed_data = pc_info.data;
  auto restored_header = Cloudini::DecodeHeader(compressed_data);
  cloud_decoder.decode(restored_header, compressed_data, decoded_cloud_view);

  // add last field
  cdr_encoder.encode(pc_info.is_dense);
}

void applyResolutionProfile(
    const ResolutionProfile& profile, std::vector<PointField>& fields, std::optional<float> default_resolution) {
  // erase-remove idiom to remove fields with profile resolution of 0
  fields.erase(
      std::remove_if(
          fields.begin(), fields.end(),
          [&profile](const auto& field) {
            auto profile_it = profile.find(field.name);
            return (profile_it != profile.end() && profile_it->second == 0);
          }),
      fields.end());

  for (auto& field : fields) {
    auto profile_it = profile.find(field.name);
    if (profile_it != profile.end()) {
      field.resolution = profile_it->second;
    } else if (default_resolution && field.type == Cloudini::FieldType::FLOAT32) {
      field.resolution = *default_resolution;
    }
  }
}

}  // namespace Cloudini
