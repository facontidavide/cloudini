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

#include "cloudini_lib/wasm_functions.h"

#include <emscripten.h>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/ros_message_definitions.hpp"
#include "cloudini_lib/ros_msg_utils.hpp"

uint32_t cldn_GetHeaderAsYAML(uintptr_t encoded_data_ptr, uint32_t encoded_data_size, uintptr_t output_yaml_ptr) {
  try {
    const uint8_t* encoded_data = reinterpret_cast<const uint8_t*>(encoded_data_ptr);
    Cloudini::ConstBufferView encoded_view(encoded_data, encoded_data_size);

    Cloudini::EncodingInfo info = Cloudini::DecodeHeader(encoded_view);
    std::string json_str = Cloudini::EncodingInfoToYAML(info);

    // Copy the YAML string to the output buffer
    char* output_yaml = reinterpret_cast<char*>(output_yaml_ptr);
    size_t yaml_size = json_str.size();
    std::memcpy(output_yaml, json_str.data(), yaml_size);

    return yaml_size;
  } catch (const std::exception& e) {
    EM_ASM({ console.error('Exception in cldn_GetHeaderAsYAML:', UTF8ToString($0)); }, e.what());
    return 0;
  }
}

uint32_t cldn_ComputeCompressedSize(uintptr_t dds_msg_ptr, uint32_t dds_msg_size, float resolution) {
  try {
    const uint8_t* raw_msg_data = reinterpret_cast<const uint8_t*>(dds_msg_ptr);
    Cloudini::ConstBufferView raw_dds_msg(raw_msg_data, dds_msg_size);

    cloudini_ros::CloudiniPointCloud pc_info;
    cloudini_ros::getDeserializedPointCloudMessage(raw_dds_msg, pc_info);

    Cloudini::EncodingInfo encoding_info = cloudini_ros::toEncodingInfo(pc_info);

    for (auto& field : encoding_info.fields) {
      if (field.type == Cloudini::FieldType::FLOAT32) {
        field.resolution = resolution;
      }
    }

    // Don't use static - it causes memory leaks in WASM
    std::vector<uint8_t> compressed_cloud;
    Cloudini::PointcloudEncoder pc_encoder(encoding_info);

    // Verify data size matches expected size
    const size_t expected_size = pc_info.width * pc_info.height * pc_info.point_step;

    if (pc_info.data.size() != expected_size) {
      // Check if dimensions are valid
      if (pc_info.width == 0 || pc_info.height == 0) {
        return 0;
      }
    }
    const auto compressed_size = pc_encoder.encode(pc_info.data, compressed_cloud);
    return compressed_size;
  } catch (const std::exception& e) {
    EM_ASM({ console.error('Exception in cldn_ComputeCompressedSize:', UTF8ToString($0)); }, e.what());
    return 0;
  }
}

uint32_t cldn_GetDecompressedSize(uintptr_t encoded_dds_ptr, uint32_t encoded_dds_size) {
  const uint8_t* compressed_data = reinterpret_cast<const uint8_t*>(encoded_dds_ptr);
  Cloudini::ConstBufferView raw_dds_msg(compressed_data, encoded_dds_size);
  cloudini_ros::CloudiniPointCloud compressed_cloud;
  cloudini_ros::getDeserializedPointCloudMessage(raw_dds_msg, compressed_cloud);
  return compressed_cloud.height * compressed_cloud.width * compressed_cloud.point_step;
}

uint32_t cldn_ConvertCompressedMsgToPointCloud2Msg(
    uintptr_t compressed_msg_ptr, uint32_t encoded_data_size, uintptr_t output_msg_ptr) {
  const uint8_t* compressed_data = reinterpret_cast<const uint8_t*>(compressed_msg_ptr);
  static std::vector<uint8_t> output_msg_buffer;

  try {
    cloudini_ros::CloudiniPointCloud pc_info;
    cloudini_ros::getDeserializedPointCloudMessage({compressed_data, encoded_data_size}, pc_info);
    cloudini_ros::convertCompressedCloudToPointCloud2(pc_info, output_msg_buffer);

    // Copy the output message to the output buffer
    std::memcpy(reinterpret_cast<void*>(output_msg_ptr), output_msg_buffer.data(), output_msg_buffer.size());
    return output_msg_buffer.size();
  } catch (const std::exception& e) {
    EM_ASM({ console.error('Exception in cldn_ConvertCompressedMsgToPointCloud2Msg:', UTF8ToString($0)); }, e.what());
    return 0;
  }
}

uint32_t cldn_DecodeCompressedMessage(uintptr_t encoded_dds_ptr, uint32_t encoded_dds_size, uintptr_t output_data) {
  const uint8_t* dds_data = reinterpret_cast<const uint8_t*>(encoded_dds_ptr);

  Cloudini::ConstBufferView dds_data_view(dds_data, encoded_dds_size);
  cloudini_ros::CloudiniPointCloud pcl_msg;
  cloudini_ros::getDeserializedPointCloudMessage(dds_data_view, pcl_msg);

  try {
    uintptr_t compressed_data_ptr = reinterpret_cast<uintptr_t>(pcl_msg.data.data());
    uint32_t decoded_size = cldn_DecodeCompressedData(compressed_data_ptr, pcl_msg.data.size(), output_data);

    if (decoded_size == 0) {
      EM_ASM({ console.error('Failed to decode raw message.'); });
      return 0;
    }
    return decoded_size;
  } catch (const std::exception& e) {
    EM_ASM({ console.error('Exception in cldn_DecodeCompressedMessage:', UTF8ToString($0)); }, e.what());
    return 0;
  }
}

uint32_t cldn_DecodeCompressedData(uintptr_t encoded_data_ptr, uint32_t encoded_data_size, uintptr_t output_data) {
  const uint8_t* encoded_data = reinterpret_cast<const uint8_t*>(encoded_data_ptr);
  Cloudini::ConstBufferView encoded_view(encoded_data, encoded_data_size);

  Cloudini::EncodingInfo info;
  try {
    info = Cloudini::DecodeHeader(encoded_view);
  } catch (std::exception& ex) {
    EM_ASM({ console.error('Failed to decode header:', UTF8ToString($0)); }, ex.what());
    return 0;
  }

  size_t decoded_size = info.width * info.height * info.point_step;
  uint8_t* decoded_data = reinterpret_cast<uint8_t*>(output_data);
  Cloudini::BufferView decoded_view(decoded_data, decoded_size);

  try {
    Cloudini::PointcloudDecoder decoder;
    decoder.decode(info, encoded_view, decoded_view);
  } catch (std::exception& ex) {
    EM_ASM({ console.error('Failed to decompress point cloud:', UTF8ToString($0)); }, ex.what());
    return 0;
  }
  return decoded_size;
}

uint32_t cldn_EncodePointcloudMessage(
    const uintptr_t pointcloud_msg_ptr, size_t msg_size, float resolution, uintptr_t output_data_ptr) {
  try {
    const uint8_t* raw_msg_data = reinterpret_cast<const uint8_t*>(pointcloud_msg_ptr);
    Cloudini::ConstBufferView raw_dds_msg(raw_msg_data, msg_size);

    cloudini_ros::CloudiniPointCloud pc_info = cloudini_ros::getDeserializedPointCloudMessage(raw_dds_msg);

    Cloudini::EncodingInfo encoding_info = cloudini_ros::toEncodingInfo(pc_info);

    for (auto& field : encoding_info.fields) {
      if (field.type == Cloudini::FieldType::FLOAT32) {
        field.resolution = resolution;
      }
    }

    // Verify data size matches expected size
    const size_t expected_size = pc_info.width * pc_info.height * pc_info.point_step;

    if (pc_info.data.size() != expected_size) {
      return 0;
    }
    // Assume output buffer is large enough
    Cloudini::BufferView output_view(reinterpret_cast<uint8_t*>(output_data_ptr), msg_size);
    Cloudini::PointcloudEncoder pc_encoder(encoding_info);
    const auto compressed_size = pc_encoder.encode(pc_info.data, output_view, true);
    return compressed_size;
  } catch (const std::exception& e) {
    EM_ASM({ console.error('Exception in cldn_EncodePointcloudMessage:', UTF8ToString($0)); }, e.what());
    return 0;
  }
}

uint32_t cldn_EncodePointcloudData(
    const char* header_as_yaml, const uintptr_t pc_data_ptr, uint32_t pc_data_size, uintptr_t output_data_ptr) {
  try {
    const Cloudini::EncodingInfo encoding_info = Cloudini::EncodingInfoFromYAML(header_as_yaml);

    // Verify data size matches expected size
    uint32_t expected_size = encoding_info.width * encoding_info.height * encoding_info.point_step;

    if (pc_data_size != expected_size) {
      EM_ASM({ console.error('Data size mismatch: expected', $0, 'but got', $1); }, expected_size, pc_data_size);
      return 0;
    }

    Cloudini::ConstBufferView pc_data_view(reinterpret_cast<const uint8_t*>(pc_data_ptr), pc_data_size);
    Cloudini::BufferView output_buffer(reinterpret_cast<uint8_t*>(output_data_ptr), pc_data_size);

    Cloudini::PointcloudEncoder pc_encoder(encoding_info);
    const auto compressed_size = pc_encoder.encode(pc_data_view, output_buffer, true);
    return compressed_size;
  } catch (const std::exception& e) {
    EM_ASM({ console.error('Exception in cldn_EncodePointcloudData:', UTF8ToString($0)); }, e.what());
    return 0;
  }
}
