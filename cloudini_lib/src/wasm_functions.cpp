#include <emscripten.h>

#include "cloudini_lib/wasm_functions.h"

#include "cloudini_lib/ros_message_definitions.hpp"
#include "cloudini_lib/ros_msg_utils.hpp"
#include "cloudini_lib/cloudini.hpp"

size_t ComputeCompressedSize(uintptr_t dataPtr, size_t size) {
  const uint8_t* raw_msg_data = reinterpret_cast<const uint8_t*>(dataPtr);
  Cloudini::ConstBufferView raw_dds_msg(raw_msg_data, size);

  auto pc_info = Cloudini::readPointCloud2Message(raw_dds_msg);

  Cloudini::EncodingInfo encoding_info = Cloudini::toEncodingInfo(pc_info);
  for (auto& field : encoding_info.fields) {
    if (field.type == Cloudini::FieldType::FLOAT32) {
      field.resolution = 0.001;
    }
  }

  static std::vector<uint8_t> compressed_cloud;

  Cloudini::PointcloudEncoder pc_encoder(encoding_info);
  auto new_size = pc_encoder.encode(pc_info.data, compressed_cloud);

  return new_size;
}

size_t DecompressPointCloudBuffer(uintptr_t compressedPtr, size_t compressedSize, uintptr_t outputPtr) {
  const uint8_t* compressed_data = reinterpret_cast<const uint8_t*>(compressedPtr);

  Cloudini::ConstBufferView raw_dds_msg(compressed_data, compressedSize);
  auto compressed_cloud = Cloudini::readCompressedPointCloud2Message(raw_dds_msg);

  
  Cloudini::EncodingInfo info;
  try {
    info = Cloudini::DecodeHeader(compressed_cloud. compressed_data);
  } catch (...) {
    EM_ASM({
      console.error('Failed to decode header.');
    });
    return 0;
  }

  size_t decompressed_size = info.width * info.height * info.point_step;
  uint8_t* output_data = reinterpret_cast<uint8_t*>(outputPtr);
  Cloudini::BufferView output_view(output_data, decompressed_size);

  try {
    Cloudini::PointcloudDecoder decoder;
    Cloudini::ConstBufferView compressed_view(compressed_cloud.compressed_data.data(), compressed_cloud.compressed_data.size());
    decoder.decode(info, compressed_view, output_view);
  } catch (...) {
    EM_ASM({
      console.error('Failed to decompress point cloud.');
    });
    return 0;
  }
  return decompressed_size;
}

size_t GetDecompressedSize(uintptr_t compressedPtr, size_t compressedSize) {
  const uint8_t* compressed_data = reinterpret_cast<const uint8_t*>(compressedPtr);
  Cloudini::ConstBufferView raw_dds_msg(compressed_data, compressedSize);
  auto compressed_cloud = Cloudini::readCompressedPointCloud2Message(raw_dds_msg);
  return compressed_cloud.height * compressed_cloud.width * compressed_cloud.point_step;
}
