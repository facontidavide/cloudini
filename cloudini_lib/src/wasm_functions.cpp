#include "cloudini_lib/wasm_functions.h"

#include "cloudini_lib/ros_message_definitions.hpp"
#include "cloudini_lib/ros_msg_utils.hpp"

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
