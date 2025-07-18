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

#include "cloudini_lib/cloudini.hpp"

#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

#include "cloudini_lib/encoding_utils.hpp"
#include "cloudini_lib/field_decoder.hpp"
#include "cloudini_lib/field_encoder.hpp"
#include "lz4.h"
#include "zstd.h"

namespace Cloudini {

const uint8_t kMagicLength = strlen(kMagicHeader);

size_t ComputeHeaderSize(const std::vector<PointField>& fields) {
  size_t header_size = kMagicLength;
  header_size += sizeof(uint32_t);                   // width
  header_size += sizeof(uint32_t);                   // height
  header_size += sizeof(uint32_t);                   // point_step
  header_size += sizeof(uint8_t) + sizeof(uint8_t);  // encoding and compression stage options
  header_size += sizeof(uint16_t);                   // fields count

  for (const auto& field : fields) {
    header_size += field.name.size() + sizeof(uint16_t);  // name
    header_size += sizeof(uint32_t);                      // offset
    header_size += sizeof(uint8_t);                       // type
    header_size += sizeof(float);                         // resolution
  }
  return header_size;
}

size_t EncodeHeader(const EncodingInfo& header, BufferView& output) {
  const size_t prev_size = output.size();

  memcpy(output.data(), kMagicHeader, kMagicLength);
  output.trim_front(kMagicLength);

  encode(header.width, output);
  encode(header.height, output);
  encode(header.point_step, output);

  encode(static_cast<uint8_t>(header.encoding_opt), output);
  encode(static_cast<uint8_t>(header.compression_opt), output);
  encode(static_cast<uint16_t>(header.fields.size()), output);

  for (const auto& field : header.fields) {
    encode(field.name, output);
    encode(field.offset, output);
    encode(static_cast<uint8_t>(field.type), output);
    if (field.resolution) {
      encode(*field.resolution, output);
    } else {
      const float res = -1.0;
      encode(res, output);
    }
  }
  return prev_size - output.size();
}

EncodingInfo DecodeHeader(ConstBufferView& input) {
  EncodingInfo header;
  const uint8_t* buff = input.data();

  // check the magic header
  if (memcmp(buff, kMagicHeader, kMagicLength) != 0) {
    std::string fist_bytes = std::string(reinterpret_cast<const char*>(buff), kMagicLength);
    throw std::runtime_error(std::string("Invalid magic header. got: ") + fist_bytes);
  }
  input.trim_front(kMagicLength);

  decode(input, header.width);
  decode(input, header.height);
  decode(input, header.point_step);

  uint8_t stage;
  decode(input, stage);
  header.encoding_opt = static_cast<EncodingOptions>(stage);

  decode(input, stage);
  header.compression_opt = static_cast<CompressionOption>(stage);

  uint16_t fields_count = 0;
  decode(input, fields_count);

  for (int i = 0; i < fields_count; ++i) {
    PointField field;
    decode(input, field.name);
    decode(input, field.offset);
    uint8_t type = 0;
    decode(input, type);
    field.type = static_cast<FieldType>(type);
    float res = 0.0;
    decode(input, res);
    if (res > 0) {
      field.resolution = res;
    }
    header.fields.push_back(std::move(field));
  }
  return header;
}

PointcloudEncoder::PointcloudEncoder(const EncodingInfo& info) : info_(info) {
  header_.resize(ComputeHeaderSize(info_.fields));
  BufferView buffer_view(header_.data(), header_.size());
  //-------------------------------------------------------------------------------------------
  EncodeHeader(info_, buffer_view);

  if (info_.encoding_opt == EncodingOptions::NONE) {
    for (const auto& field : info_.fields) {
      encoders_.push_back(std::make_unique<FieldEncoderCopy>(field.offset, field.type));
    }
    return;
  }
  //-------------------------------------------------------------------------------------------
  // special case: first 3 or 4 fields are consecutive FLOAT32 fields
  size_t start_index = 0;

  if (info_.encoding_opt == EncodingOptions::LOSSY) {
    size_t floats_count = 0;
    for (size_t i = 0; i < info_.fields.size(); ++i) {
      if (info_.fields[i].type != FieldType::FLOAT32 || !info_.fields[i].resolution.has_value()) {
        break;
      }
      floats_count++;
    }
    if (floats_count == 3 || floats_count == 4) {
      start_index = floats_count;
      std::vector<FieldEncoderFloatN_Lossy::FieldData> field_data;
      field_data.reserve(floats_count);
      for (size_t i = 0; i < floats_count; ++i) {
        field_data.emplace_back(info_.fields[i].offset, info_.fields[i].resolution.value());
      }
      encoders_.push_back(std::make_unique<FieldEncoderFloatN_Lossy>(field_data));
    }
  }
  //-------------------------------------------------------------------------------------------
  // do remaining fields
  for (size_t index = start_index; index < info_.fields.size(); ++index) {
    const auto& field = info_.fields[index];
    const auto offset = field.offset;

    switch (field.type) {
      case FieldType::FLOAT32: {
        if (info_.encoding_opt == EncodingOptions::LOSSY && field.resolution.has_value()) {
          encoders_.push_back(std::make_unique<FieldEncoderFloat_Lossy<float>>(offset, *field.resolution));
        } else {
          encoders_.push_back(std::make_unique<FieldEncoderCopy>(offset, field.type));
        }
      } break;

      case FieldType::FLOAT64: {
        if (info_.encoding_opt == EncodingOptions::LOSSY && field.resolution.has_value()) {
          encoders_.push_back(std::make_unique<FieldEncoderFloat_Lossy<double>>(offset, *field.resolution));
        } else {
          encoders_.push_back(std::make_unique<FieldEncoderFloat_XOR<double>>(offset));
        }
      } break;

      case FieldType::INT16:
        encoders_.push_back(std::make_unique<FieldEncoderInt<int16_t>>(offset));
        break;
      case FieldType::INT32:
        encoders_.push_back(std::make_unique<FieldEncoderInt<int32_t>>(offset));
        break;
      case FieldType::UINT16:
        encoders_.push_back(std::make_unique<FieldEncoderInt<uint16_t>>(offset));
        break;
      case FieldType::UINT32:
        encoders_.push_back(std::make_unique<FieldEncoderInt<uint32_t>>(offset));
        break;
      case FieldType::UINT64:
        encoders_.push_back(std::make_unique<FieldEncoderInt<uint64_t>>(offset));
        break;
      case FieldType::INT64:
        encoders_.push_back(std::make_unique<FieldEncoderInt<int64_t>>(offset));
        break;
      case FieldType::INT8:
      case FieldType::UINT8:
        encoders_.push_back(std::make_unique<FieldEncoderCopy>(offset, field.type));
        break;
      default:
        throw std::runtime_error("Unsupported field type:" + std::to_string(static_cast<int>(field.type)));
    }
  }
}

size_t PointcloudEncoder::encode(ConstBufferView cloud_data, std::vector<uint8_t>& output) {
  // maximum compressed size in worst case single-pass scenario
  size_t max_compressed_size = ZSTD_compressBound(cloud_data.size());
  output.resize(header_.size() + max_compressed_size);
  BufferView output_view(output.data(), output.size());
  auto new_size = encode(cloud_data, output_view);
  output.resize(new_size);
  return new_size;
}

size_t PointcloudEncoder::encode(ConstBufferView cloud_data, BufferView& output_view) {
  buffer_.resize(cloud_data.size());
  const BufferView buffer_view(buffer_.data(), buffer_.size());

  // copy the header at the beginning of the output
  memcpy(output_view.data(), header_.data(), header_.size());
  output_view.trim_front(header_.size());
  //----------------------------------------------
  // reset the state of the encoders
  for (auto& encoder : encoders_) {
    encoder->reset();
  }
  //----------------------------------------------
  // first stage compression. Result is stored in buffer_
  size_t serialized_size = 0;
  {
    BufferView encoding_view = (info_.compression_opt == CompressionOption::NONE) ? output_view : buffer_view;
    while (cloud_data.size() > 0) {
      for (auto& encoder : encoders_) {
        serialized_size += encoder->encode(cloud_data, encoding_view);
      }
      cloud_data.trim_front(info_.point_step);
    }

    // Flush any remaining buffered data from encoders
    for (auto& encoder : encoders_) {
      serialized_size += encoder->flush(encoding_view);
    }

    // if there is no 2nd stage, we have done already
    if (info_.compression_opt == CompressionOption::NONE) {
      return serialized_size + header_.size();
    }
  }

  //----------------------------------------------

  const char* src_ptr = reinterpret_cast<const char*>(buffer_view.data());
  const size_t src_size = serialized_size;
  char* dest_ptr = reinterpret_cast<char*>(output_view.data());
  const size_t dest_capacity = output_view.size();
  //----------------------------------------------
  // second stage compression. Data in _buffer will be compressed into output
  switch (info_.compression_opt) {
    case CompressionOption::LZ4: {
      int compressed_size = LZ4_compress_default(src_ptr, dest_ptr, src_size, dest_capacity);
      if (compressed_size <= 0) {
        throw std::runtime_error("LZ4 compression failed");
      }
      return (compressed_size + header_.size());
    } break;

    case CompressionOption::ZSTD: {
      size_t const max_compressed_size = ZSTD_compressBound(src_size);
      if (dest_capacity < max_compressed_size) {
        throw std::runtime_error(
            "Destination buffer too small for ZSTD compression. Required: " + std::to_string(max_compressed_size) +
            ", Available: " + std::to_string(dest_capacity));
      }
      size_t compressed_size = ZSTD_compress(dest_ptr, dest_capacity, src_ptr, src_size, 1);
      if (ZSTD_isError(compressed_size)) {
        throw std::runtime_error("ZSTD compression failed");
      }
      return (compressed_size + header_.size());
    } break;

    default:
      throw std::runtime_error("Unsupported second stage compression");
  }
  return 0;  // should never reach here
}

//------------------------------------------------------------------------------------------

void PointcloudDecoder::updateDecoders(const EncodingInfo& info) {
  auto create_decoder = [](const PointField& field) -> std::unique_ptr<FieldDecoder> {
    const auto offset = field.offset;
    switch (field.type) {
      case FieldType::FLOAT32:
        if (field.resolution) {
          return std::make_unique<FieldDecoderFloat_Lossy<float>>(offset, *field.resolution);
        } else {
          return std::make_unique<FieldDecoderCopy>(field.offset, field.type);
        }
        break;
      case FieldType::FLOAT64:
        if (field.resolution) {
          return std::make_unique<FieldDecoderFloat_Lossy<double>>(offset, *field.resolution);
        } else {
          return std::make_unique<FieldDecoderFloat_XOR<double>>(offset);
        }
        break;
      case FieldType::INT16:
        return std::make_unique<FieldDecoderInt<int16_t>>(offset);
      case FieldType::INT32:
        return std::make_unique<FieldDecoderInt<int32_t>>(offset);
      case FieldType::UINT16:
        return std::make_unique<FieldDecoderInt<uint16_t>>(offset);
      case FieldType::UINT32:
        return std::make_unique<FieldDecoderInt<uint32_t>>(offset);
      case FieldType::UINT64:
        return std::make_unique<FieldDecoderInt<uint64_t>>(offset);
      case FieldType::INT64:
        return std::make_unique<FieldDecoderInt<int64_t>>(offset);
      case FieldType::INT8:
      case FieldType::UINT8:
        return std::make_unique<FieldDecoderCopy>(field.offset, field.type);
      default:
        throw std::runtime_error("Unsupported field type");
    }
  };

  decoders_.clear();

  // special case: first 3 or 4 fields are consecutive FLOAT32 fields
  size_t start_index = 0;

  if (info.encoding_opt == EncodingOptions::LOSSY) {
    size_t floats_count = 0;
    for (size_t i = 0; i < info.fields.size(); ++i) {
      if (info.fields[i].type != FieldType::FLOAT32 || !info.fields[i].resolution.has_value()) {
        break;
      }
      floats_count++;
    }
    if (floats_count == 3 || floats_count == 4) {
      start_index = floats_count;
      std::vector<FieldDecoderFloatN_Lossy::FieldData> field_data;
      field_data.reserve(floats_count);
      for (size_t i = 0; i < floats_count; ++i) {
        field_data.emplace_back(info.fields[i].offset, info.fields[i].resolution.value());
      }
      decoders_.push_back(std::make_unique<FieldDecoderFloatN_Lossy>(field_data));
    }
  }

  // do remaining fields
  for (size_t index = start_index; index < info.fields.size(); ++index) {
    decoders_.push_back(create_decoder(info.fields[index]));
  }
}

void PointcloudDecoder::decode(const EncodingInfo& info, ConstBufferView compressed_data, BufferView output) {
  // read the header
  updateDecoders(info);

  // check if the first bytes are the magic header. if they are, skip them
  if (memcmp(compressed_data.data(), kMagicHeader, kMagicLength) == 0) {
    throw std::runtime_error("compressed_data contains the header. You should use DecodeHeader first");
  }

  // allocate sufficient space in the buffer
  buffer_.resize(info.width * info.height * info.point_step);

  //----------------------------------------------------------------------
  // start decompressing using "compression_opt" param.
  // Note that compressed_data doesn't contan the header anymore.
  // Decompressed data will be stored in buffer_
  switch (info.compression_opt) {
    case CompressionOption::LZ4: {
      const auto* src_ptr = reinterpret_cast<const char*>(compressed_data.data());
      auto* buffer_ptr = reinterpret_cast<char*>(buffer_.data());
      const int decompressed_size = LZ4_decompress_safe(src_ptr, buffer_ptr, compressed_data.size(), buffer_.size());
      if (decompressed_size < 0) {
        throw std::runtime_error("LZ4 decompression failed");
      }
      buffer_.resize(decompressed_size);
    } break;

    case CompressionOption::ZSTD: {
      const size_t decompressed_size =
          ZSTD_decompress(buffer_.data(), buffer_.size(), compressed_data.data(), compressed_data.size());
      if (ZSTD_isError(decompressed_size)) {
        throw std::runtime_error("ZSTD decompression failed");
      }
      buffer_.resize(decompressed_size);
    } break;

    default:
      break;  // do nothing
  }

  //----------------------------------------------------------------------
  // decode the data (first stage).
  auto encoded_view = (info.compression_opt == CompressionOption::NONE)
                          ? ConstBufferView(compressed_data)
                          : ConstBufferView(buffer_.data(), buffer_.size());

  for (size_t i = 0; i < info.width * info.height; ++i) {
    BufferView point_view(output.data() + i * info.point_step, info.point_step);
    for (auto& decoder : decoders_) {
      decoder->decode(encoded_view, point_view);
    }
  }
}

}  // namespace Cloudini
