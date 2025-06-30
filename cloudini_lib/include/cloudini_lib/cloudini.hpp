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

#pragma once

#include <memory>

#include "cloudini_lib/encoding_utils.hpp"
#include "cloudini_lib/field_decoder.hpp"
#include "cloudini_lib/field_encoder.hpp"

namespace Cloudini {

/// First stage of the encoding, using custom encoding
enum class EncodingOptions : uint8_t {
  // Do nothing. Compression is done in the second stage only.
  // We kee pthis only for benchmarking purposes.
  NONE = 0,
  // Will apply lossy compression to FLOAT32 fields
  LOSSY = 1,
  // Lossless compression (e.g. XoR for FLOAT32).
  // Currently it has very poor performance, so it is not recommended.
  // Keeping it for future improvemnts
  LOSSLES = 2,
};

/// Second stage of the encoding, using general purpose compression
enum class CompressionOption : uint8_t {
  // No compression
  NONE = 0,
  // LZ4 compression
  LZ4 = 1,
  // ZSTD compression
  ZSTD = 2
};

struct EncodingInfo {
  // Fields in the point cloud
  std::vector<PointField> fields;

  // equal to number of points when (height == 1)
  uint32_t width = 0;

  // clouds that are not organized have height equal to 1
  uint32_t height = 1;

  // the size in bytes of a single point
  uint32_t point_step = 0;

  // the fist step of the encoding
  EncodingOptions encoding_opt = EncodingOptions::LOSSY;

  // the second step is a general purpose compression
  CompressionOption compression_opt = CompressionOption::ZSTD;

  bool operator==(const EncodingInfo& other) const {
    if (fields.size() != other.fields.size()) {
      return false;
    }
    for (size_t i = 0; i < fields.size(); ++i) {
      if (fields[i] != other.fields[i]) {
        return false;
      }
    }
    return width == other.width && height == other.height && point_step == other.point_step &&
           encoding_opt == other.encoding_opt && compression_opt == other.compression_opt;
  }
  bool operator!=(const EncodingInfo& other) const {
    return !(*this == other);
  }
};

constexpr const char* kMagicHeader = "CLOUDINI_V01";

// pre-compute the size of the header, to allocate memory
size_t ComputeHeaderSize(const std::vector<PointField>& fields);

/**
 * @brief Encode the header into the output buffer. Called by PointcloudEncode (exposed here for testing)
 *
 * @param header The header information to encode.
 * @param output The output buffer to write the encoded header data. Will be modified to point to the next data after
 * the header.
 * @return The size of the encoded header.
 */
size_t EncodeHeader(const EncodingInfo& header, BufferView& output);

/**
 * @brief Decode the header from the input buffer.
 *
 * @param info The encoding information.
 * @return HeaderInfo The decoded header information.
 */
EncodingInfo DecodeHeader(ConstBufferView& input);

/**
 * @brief PointcloudEncoder is used to encode a point cloud into a compressed format.
 *
 * The encoder uses two stages of compression:
 * 1. First stage: applies lossy or lossless compression to the fields of the point cloud.
 * 2. Second stage: applies general-purpose compression (LZ4 or ZSTD) to the entire point cloud data.
 */
class PointcloudEncoder {
 public:
  PointcloudEncoder(const EncodingInfo& info);

  /**
   * @brief Encode the point cloud data into a compressed format.
   *
   * @param cloud_data The input point cloud data to be encoded.
   * @param output The output buffer to store the compressed data. It will be resized to fit the encoded data.
   * @return The size of the encoded data.
   */
  size_t encode(ConstBufferView cloud_data, std::vector<uint8_t>& output);

  // version that will not allocate any memory in output. Use it at your own risk
  size_t encode(ConstBufferView cloud_data, BufferView& output);

  const EncodingInfo& getEncodingInfo() const {
    return info_;
  }

 private:
  EncodingInfo info_;
  std::vector<std::unique_ptr<FieldEncoder>> encoders_;
  std::vector<uint8_t> buffer_;
  std::vector<uint8_t> header_;
};

/**
 * @brief PointcloudDecoder is used to decode a compressed point cloud into its original format.
 */
class PointcloudDecoder {
 public:
  PointcloudDecoder() {}

  /**
   * @brief Decode the compressed point cloud data into its original format.
   *
   * @param info The encoding information for the point cloud.
   * @param compressed_data The input compressed data to be decoded. It should NOT contain the header.
   * @param output The output buffer to store the decoded point cloud data. Memory should be already allocated
   */
  void decode(const EncodingInfo& info, ConstBufferView compressed_data, BufferView output);

  void decode(const EncodingInfo& info, ConstBufferView compressed_data, std::vector<uint8_t>& output) {
    output.resize(info.width * info.height * info.point_step);
    BufferView output_view(output.data(), output.size());
    decode(info, compressed_data, output_view);
  }

 private:
  void updateDecoders(const EncodingInfo& info);

  std::vector<std::unique_ptr<FieldDecoder>> decoders_;
  std::vector<uint8_t> buffer_;
};
}  // namespace Cloudini
