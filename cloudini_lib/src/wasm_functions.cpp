#include "cloudini_lib/wasm_functions.h"

#include <emscripten.h>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/ros_message_definitions.hpp"
#include "cloudini_lib/ros_msg_utils.hpp"

size_t ComputeCompressedSize(uintptr_t data_ptr, size_t size) {
  const uint8_t* raw_msg_data = reinterpret_cast<const uint8_t*>(data_ptr);
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

size_t DecodePointCloudMessage(uintptr_t msg_ptr, size_t msg_size, uintptr_t output_ptr) {
  const uint8_t* msg_data = reinterpret_cast<const uint8_t*>(msg_ptr);

  Cloudini::ConstBufferView raw_dds_msg(msg_data, msg_size);
  auto pcl_msg = Cloudini::readCompressedPointCloud2Message(raw_dds_msg);

  try{
    size_t decoded_size = DecodePointCloudBuffer(reinterpret_cast<const uintptr_t>(pcl_msg.compressed_data.data()), pcl_msg.compressed_data.size(), output_ptr);
    if (decoded_size == 0){
      EM_ASM({ console.error('Failed to decode raw message.'); });
      return 0;  
    }
    return decoded_size;
  } catch (...) {
    EM_ASM({ console.error('Failed to decode raw message.'); });
    return 0;
  }
}

size_t DecodePointCloudBuffer(uintptr_t data_ptr, size_t data_size, uintptr_t output_ptr) {
  const uint8_t* encoded_data = reinterpret_cast<const uint8_t*>(data_ptr);

  Cloudini::ConstBufferView encoded_view(encoded_data, data_size);

  Cloudini::EncodingInfo info;
  try {
    info = Cloudini::DecodeHeader(encoded_view);
  } catch (...) {
    EM_ASM({ console.error('Failed to decode header.'); });
    return 0;
  }

  size_t decoded_size = info.width * info.height * info.point_step;
  uint8_t* decoded_data = reinterpret_cast<uint8_t*>(output_ptr);
  Cloudini::BufferView decoded_view(decoded_data, decoded_size);

  try {
    Cloudini::PointcloudDecoder decoder;
    decoder.decode(info, encoded_view, decoded_view);
  } catch (...) {
    EM_ASM({ console.error('Failed to decompress point cloud.'); });
    return 0;
  }
  return decoded_size;
}

size_t GetDecompressedSize(uintptr_t compressedPtr, size_t compressedSize) {
  const uint8_t* compressed_data = reinterpret_cast<const uint8_t*>(compressedPtr);
  Cloudini::ConstBufferView raw_dds_msg(compressed_data, compressedSize);
  auto compressed_cloud = Cloudini::readCompressedPointCloud2Message(raw_dds_msg);
  return compressed_cloud.height * compressed_cloud.width * compressed_cloud.point_step;
}
