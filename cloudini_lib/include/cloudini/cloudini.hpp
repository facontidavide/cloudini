#pragma once

#include <iostream>

#include "cloudini/encoding_utils.hpp"

namespace Cloudini {

enum class FirstStageOpt : uint8_t {
  // No compression
  UNDEFINED = 0,
  // May apply lossy compression to some fields
  // (e.g. position, normals, doubles)
  LOSSY = 1,
  // Lossless compression (e.g. XoR for doubles)
  LOSSLES = 2
};

enum class SecondStageOpt : uint8_t {
  NONE = 0,  // No compression
  LZ4 = 1,   // LZ4 compression
  ZSTD = 2   // ZSTD compression
};

struct EncodingInfo {
  // Fields in the point cloud
  std::vector<PointField> fields;

  uint32_t width = 0;

  // clouds that are not organized have height equal to 1
  uint32_t height = 1;

  // the size in bytes of a single point
  uint32_t point_step = 0;

  // the fist step of the encoding
  FirstStageOpt firts_stage = FirstStageOpt::LOSSY;

  // the second step of the encoding (general purpose compression)
  SecondStageOpt second_stage = SecondStageOpt::ZSTD;

  uint32_t decoded_size = 0;
};

constexpr const char* magic_header = "CLOUDINI_V01";

// pre-compute the size of the header, to allocate memory
size_t ComputeHeaderSize(const std::vector<PointField>& fields);

/**
 * @brief Encode the header into the output buffer. Called by PointcloudEncode (exposed here for testing)
 *
 * @param header The header information to encode.
 * @param output The output buffer to write the encoded header data. Will be modified to point to the next data after
 * the header.
 */
void EncodeHeader(const EncodingInfo& header, BufferView& output);

/**
 * @brief Decode the header from the input buffer.
 *
 * @param info The encoding information.
 * @return HeaderInfo The decoded header information.
 */
EncodingInfo DecodeHeader(ConstBufferView& input);

size_t PointcloudEncode(const EncodingInfo& info, ConstBufferView cloud_data, BufferView serialized_cloud);

size_t PointcloudDecode(ConstBufferView serialized_cloud, BufferView cloud_data);

}  // namespace Cloudini
