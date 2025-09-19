#include "cloudini_lib/wasm_functions.h"

#include <emscripten.h>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/ros_message_definitions.hpp"
#include "cloudini_lib/ros_msg_utils.hpp"

size_t cldn_GetHeaderAsJSON(uintptr_t encoded_data_ptr, size_t encoded_data_size, uintptr_t output_json_ptr) {
  try {
    const uint8_t* encoded_data = reinterpret_cast<const uint8_t*>(encoded_data_ptr);
    Cloudini::ConstBufferView encoded_view(encoded_data, encoded_data_size);

    Cloudini::EncodingInfo info = Cloudini::DecodeHeader(encoded_view);
    std::string json_str = Cloudini::EncodingInfoToJSON(info);

    // Copy the JSON string to the output buffer
    char* output_json = reinterpret_cast<char*>(output_json_ptr);
    size_t json_size = json_str.size();
    std::memcpy(output_json, json_str.data(), json_size);

    return json_size;
  } catch (const std::exception& e) {
    EM_ASM({ console.error('Exception in cldn_GetHeaderAsJSON:', UTF8ToString($0)); }, e.what());
    return 0;
  }
}

size_t cldn_ComputeCompressedSize(uintptr_t dds_msg_ptr, size_t dds_msg_size, float resolution) {
  try {
    const uint8_t* raw_msg_data = reinterpret_cast<const uint8_t*>(dds_msg_ptr);
    Cloudini::ConstBufferView raw_dds_msg(raw_msg_data, dds_msg_size);

    auto pc_info = Cloudini::readPointCloud2Message(raw_dds_msg);

    Cloudini::EncodingInfo encoding_info = Cloudini::toEncodingInfo(pc_info);

    for (auto& field : encoding_info.fields) {
      if (field.type == Cloudini::FieldType::FLOAT32) {
        field.resolution = resolution;
      }
    }

    // Don't use static - it causes memory leaks in WASM
    std::vector<uint8_t> compressed_cloud;
    Cloudini::PointcloudEncoder pc_encoder(encoding_info);

    // Verify data size matches expected size
    size_t expected_size = pc_info.width * pc_info.height * pc_info.point_step;

    if (pc_info.data.size() != expected_size) {
      // Check if dimensions are valid
      if (pc_info.width == 0 || pc_info.height == 0) {
        return 0;
      }
    }

    auto compressed_size = pc_encoder.encode(pc_info.data, compressed_cloud);
    return compressed_size;
  } catch (const std::exception& e) {
    EM_ASM({ console.error('Exception in cldn_ComputeCompressedSize:', UTF8ToString($0)); }, e.what());
    return 0;
  }
}

size_t cldn_GetDecompressedSize(uintptr_t encoded_dds_ptr, size_t encoded_dds_size) {
  const uint8_t* compressed_data = reinterpret_cast<const uint8_t*>(encoded_dds_ptr);
  Cloudini::ConstBufferView raw_dds_msg(compressed_data, encoded_dds_size);
  auto compressed_cloud = Cloudini::readCompressedPointCloud2Message(raw_dds_msg);
  return compressed_cloud.height * compressed_cloud.width * compressed_cloud.point_step;
}

size_t cldn_DecodeCompressedMessage(uintptr_t encoded_dds_ptr, size_t encoded_dds_size, uintptr_t output_data) {
  const uint8_t* dds_data = reinterpret_cast<const uint8_t*>(encoded_dds_ptr);

  Cloudini::ConstBufferView dds_data_view(dds_data, encoded_dds_size);
  auto pcl_msg = Cloudini::readCompressedPointCloud2Message(dds_data_view);

  try {
    uintptr_t compressed_data_ptr = reinterpret_cast<uintptr_t>(pcl_msg.compressed_data.data());
    size_t decoded_size = cldn_DecodeCompressedData(compressed_data_ptr, pcl_msg.compressed_data.size(), output_data);

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

size_t cldn_DecodeCompressedData(uintptr_t encoded_data_ptr, size_t encoded_data_size, uintptr_t output_data) {
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
