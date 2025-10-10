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

namespace cloudini_ros {

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

size_t RosPointCloud2::requiredSize() const {
  size_t total_size = 4;  // CDR header size
  total_size += sizeof(RosHeader::stamp_sec) + sizeof(RosHeader::stamp_nsec);
  total_size += sizeof(uint32_t) + ros_header.frame_id.size();  // string length
  total_size += sizeof(height) + sizeof(width);

  total_size += sizeof(uint32_t);  // number of fields
  for (const auto& field : fields) {
    total_size += sizeof(uint32_t) + field.name.size();  // string length + data
    total_size += sizeof(field.offset);                  // offset
    total_size += sizeof(field.type);                    // type
    total_size += sizeof(uint32_t);                      // count
  }
  total_size += sizeof(bool);        // is_bigendian
  total_size += sizeof(point_step);  // point_step
  total_size += sizeof(row_step);    // row_step
  total_size += sizeof(uint32_t);    // data length
  total_size += data.size();         // data
  total_size += sizeof(is_dense);    // is_dense
  return total_size;
}

size_t RosCompressedPointCloud2::requiredSize() const {
  return RosPointCloud2::requiredSize() + sizeof(uint32_t) + format.size();
}

template <typename CloudType>
void readPointCloud2MessageCommon(nanocdr::Decoder& cdr, CloudType& result) {
  static_assert(std::is_same_v<CloudType, RosPointCloud2> || std::is_same_v<CloudType, RosCompressedPointCloud2>);
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
    Cloudini::PointField field;
    cdr.decode(field.name);
    cdr.decode(field.offset);
    uint8_t type = 0;
    cdr.decode(type);
    field.type = static_cast<Cloudini::FieldType>(type);

    uint32_t count = 0;  // not used
    cdr.decode(count);

    result.fields.push_back(std::move(field));
  }

  bool is_bigendian = false;  // not used
  cdr.decode(is_bigendian);

  cdr.decode(result.point_step);
  cdr.decode(result.row_step);
}

RosPointCloud2 parsePointCloud2Message(Cloudini::ConstBufferView raw_dds_msg) {
  RosPointCloud2 result;
  nanocdr::Decoder cdr(raw_dds_msg);
  result.cdr_header = cdr.header();
  readPointCloud2MessageCommon(cdr, result);

  // remaining fields
  cdr.decode(result.data);
  cdr.decode(result.is_dense);
  return result;
}

RosCompressedPointCloud2 parseCompressedPointCloudMessage(Cloudini::ConstBufferView compressed_dds_msg) {
  RosCompressedPointCloud2 result;
  nanocdr::Decoder cdr(compressed_dds_msg);
  result.cdr_header = cdr.header();
  readPointCloud2MessageCommon(cdr, result);

  // remaining fields
  cdr.decode(result.data);
  cdr.decode(result.is_dense);
  cdr.decode(result.format);
  return result;
}

template <typename CloudType>
void writePointCloud2Header(nanocdr::Encoder& encoder, const CloudType& pc_info) {
  static_assert(std::is_same_v<CloudType, RosPointCloud2> || std::is_same_v<CloudType, RosCompressedPointCloud2>);
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

Cloudini::EncodingInfo toEncodingInfo(const RosPointCloud2& pc_info) {
  Cloudini::EncodingInfo info;
  info.height = pc_info.height;
  info.width = pc_info.width;
  info.point_step = pc_info.point_step;
  info.encoding_opt = Cloudini::EncodingOptions::LOSSY;      // default to lossy encoding
  info.compression_opt = Cloudini::CompressionOption::ZSTD;  // default to ZSTD compression
  info.fields = pc_info.fields;
  return info;
}
//-------------------------------------------------------------------

void convertCompressedCloudToPointCloud2(const RosCompressedPointCloud2& pc_info, std::vector<uint8_t>& pc2_dds_msg) {
  // We need to reserve enough space in the output buffer
  const size_t cloud_data_size = pc_info.width * pc_info.height * pc_info.point_step;

  pc2_dds_msg.clear();
  nanocdr::Encoder cdr_encoder(pc_info.cdr_header, pc2_dds_msg);
  writePointCloud2Header(cdr_encoder, pc_info);

  // Start writing the field PointCloud2::data
  cdr_encoder.encode(static_cast<uint32_t>(cloud_data_size));

  // Reserve enough memory for the decode cloud and take a buffer view to that memory
  const auto prev_size = pc2_dds_msg.size();
  pc2_dds_msg.resize(prev_size + cloud_data_size);
  // this buffer view points to a writable memory area after the PointCloud2Header
  Cloudini::BufferView decoded_buffer(pc2_dds_msg.data() + prev_size, cloud_data_size);

  Cloudini::PointcloudDecoder cloud_decoder;
  Cloudini::ConstBufferView compressed_data = pc_info.data;
  auto restored_header = Cloudini::DecodeHeader(compressed_data);
  cloud_decoder.decode(restored_header, compressed_data, decoded_buffer);

  // add last field
  cdr_encoder.encode(pc_info.is_dense);
}

void convertPointCloud2ToCompressedCloud(
    const RosPointCloud2& pc_info, const Cloudini::EncodingInfo& encoding_info, std::vector<uint8_t>& pc2_dds_msg) {
  pc2_dds_msg.clear();
  nanocdr::Encoder cdr_encoder(pc_info.cdr_header, pc2_dds_msg);
  writePointCloud2Header(cdr_encoder, pc_info);

  size_t prev_size = pc2_dds_msg.size();
  // we will write into this area later
  uint8_t* cloud_size_ptr = pc2_dds_msg.data() + prev_size;

  // writing size 0 as a placeholder for now
  cdr_encoder.encode(static_cast<uint32_t>(0));
  prev_size += 4;

  // reserve enough memory for the compressed data. we will resize later to the actual size used
  pc2_dds_msg.resize(prev_size + pc_info.data.size());

  Cloudini::BufferView compressed_data_view(pc2_dds_msg.data() + prev_size, pc2_dds_msg.size() - prev_size);
  Cloudini::PointcloudEncoder cloud_encoder(encoding_info);
  const size_t compressed_size = cloud_encoder.encode(pc_info.data, compressed_data_view, true);

  // we can finally write the actual size of the compressed data
  memcpy(cloud_size_ptr, &compressed_size, sizeof(uint32_t));

  // Resize the output buffer to the actual size used
  pc2_dds_msg.resize(prev_size + compressed_size);

  // Add the remaining fields
  cdr_encoder.encode(pc_info.is_dense);
  cdr_encoder.encode(std::string("cloudini"));  // format
}

//-------------------------------------------------------------------

void applyResolutionProfile(
    const ResolutionProfile& profile, std::vector<Cloudini::PointField>& fields,
    std::optional<float> default_resolution) {
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

}  // namespace cloudini_ros
