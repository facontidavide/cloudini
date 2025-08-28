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

size_t ComputeHeaderSize(const std::vector<PointField>& fields) {
  size_t header_size = kMagicHeaderLength + 2;       // 2 bytes for version number
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

  memcpy(output.data(), kMagicHeader, kMagicHeaderLength);
  output.trim_front(kMagicHeaderLength);

  output.data()[0] = '0' + (header.version / 10);
  output.data()[1] = '0' + (header.version % 10);
  output.trim_front(2);

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

auto char_to_num = [](char c) -> uint8_t {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  return 0;
};

EncodingInfo DecodeHeader(ConstBufferView& input) {
  const uint8_t* buff = input.data();

  // check the magic header
  if (memcmp(buff, kMagicHeader, kMagicHeaderLength) != 0) {
    std::string fist_bytes = std::string(reinterpret_cast<const char*>(buff), kMagicHeaderLength);
    throw std::runtime_error(std::string("Invalid magic header. Expecter 'CLOUDINI_V', got: ") + fist_bytes);
  }
  input.trim_front(kMagicHeaderLength);

  // next 2 bytes contain the version number as string
  uint8_t version = char_to_num(input.data()[0]) * 10 + char_to_num(input.data()[1]);
  if (version < 2 || version > kEncodingVersion) {
    throw std::runtime_error(
        "Unsupported encoding version. Current is:" + std::to_string(kEncodingVersion) +
        ", got: " + std::to_string(version));
  }
  EncodingInfo header;
  header.version = version;
  input.trim_front(2);

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
  BufferView header_view(header_.data(), header_.size());
  //-------------------------------------------------------------------------------------------
  EncodeHeader(info_, header_view);

  // Start the compression worker thread if we're using compression
  compressing_thread_ = std::thread(&PointcloudEncoder::compressionWorker, this);

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

PointcloudEncoder::~PointcloudEncoder() {
  if (compressing_thread_.joinable()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      should_exit_ = true;
    }
    cv_ready_to_compress_.notify_one();
    compressing_thread_.join();
  }
}

void PointcloudEncoder::compressionWorker() {
  while (true) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_ready_to_compress_.wait(lock, [this] {  //
        return has_data_to_compress_ || should_exit_;
      });

      if (should_exit_) {
        break;
      }
      if (!has_data_to_compress_) {
        continue;
      }
      has_data_to_compress_ = false;
    }

    // this is the 4 bytes area where the size of the chunk will be written later
    uint8_t* compressed_chunk_size_ptr = output_view_.data();
    output_view_.trim_front(4);

    const char* src_ptr = reinterpret_cast<const char*>(buffer_compressing_.data());
    const size_t src_size = buffer_compressing_.size();

    char* dest_ptr = reinterpret_cast<char*>(output_view_.data());
    const size_t dest_capacity = output_view_.size();

    uint32_t chunk_size = 0;
    switch (info_.compression_opt) {
      case CompressionOption::LZ4: {
        int comp_size = LZ4_compress_default(src_ptr, dest_ptr, src_size, dest_capacity);
        if (comp_size <= 0) {
          throw std::runtime_error("LZ4 compression failed in worker thread");
        }
        chunk_size = static_cast<uint32_t>(comp_size);
      } break;

      case CompressionOption::ZSTD: {
        size_t comp_size = ZSTD_compress(dest_ptr, dest_capacity, src_ptr, src_size, 1);
        if (ZSTD_isError(comp_size)) {
          throw std::runtime_error("ZSTD compression failed in worker thread");
        }
        chunk_size = static_cast<uint32_t>(comp_size);
      } break;
      default:
        break;
    }

    output_view_.trim_front(chunk_size);

    // write the size of the chunk
    memcpy(compressed_chunk_size_ptr, &chunk_size, sizeof(uint32_t));
    compressed_size_ += chunk_size + sizeof(uint32_t);
    compression_done_ = true;

    cv_done_compressing_.notify_one();
  }
}

void PointcloudEncoder::waitForCompressionComplete() {
  std::unique_lock<std::mutex> lock(mutex_);
  cv_done_compressing_.wait(lock, [this] { return compression_done_; });
}

size_t PointcloudEncoder::encode(ConstBufferView cloud_data, std::vector<uint8_t>& output) {
  // maximum compressed size in worst case single-pass scenario
  const size_t max_compressed_size = ZSTD_compressBound(cloud_data.size());
  // compute the number of chunks. each will require 4 extra bytes that will contain the size of the chunk
  const size_t points_count = cloud_data.size() / info_.point_step;
  const size_t chunks_count = (points_count / POINTS_PER_CHUNK) + ((points_count % POINTS_PER_CHUNK) ? 1 : 0);
  const size_t chunk_size_bytes = 4 * chunks_count;

  output.resize(header_.size() + max_compressed_size + chunk_size_bytes);
  BufferView output_view(output.data(), output.size());
  auto new_size = encode(cloud_data, output_view);
  output.resize(new_size);
  return new_size;
}

size_t PointcloudEncoder::encode(ConstBufferView cloud_data, BufferView& output) {
  // Reset the state of the encoders and the class attributes
  for (auto& encoder : encoders_) {
    encoder->reset();
  }
  compressed_size_ = 0;
  output_view_ = output;
  should_exit_ = false;
  has_data_to_compress_ = false;
  compression_done_ = true;

  // Copy the header at the beginning of the output
  memcpy(output_view_.data(), header_.data(), header_.size());
  compressed_size_ += header_.size();
  output_view_.trim_front(header_.size());

  const size_t kChunkSize = POINTS_PER_CHUNK * info_.point_step;

  buffer_.resize(kChunkSize);
  BufferView buffer_view(buffer_);

  size_t points_in_current_chunk = 0;
  size_t serialized_size = 0;
  size_t chunks_count = 0;

  while (cloud_data.size() > 0) {
    for (auto& encoder : encoders_) {
      serialized_size += encoder->encode(cloud_data, buffer_view);
    }
    cloud_data.trim_front(info_.point_step);
    points_in_current_chunk++;
    // end of chunk ?
    if (points_in_current_chunk >= POINTS_PER_CHUNK || cloud_data.empty()) {
      // simple case: no compression. Execute in the same thread
      if (info_.compression_opt == CompressionOption::NONE) {
        Cloudini::encode(static_cast<uint32_t>(serialized_size), output_view_);
        memcpy(output_view_.data(), buffer_.data(), serialized_size);
        output_view_.trim_front(serialized_size);
        compressed_size_ += serialized_size + sizeof(uint32_t);
      } else {
        waitForCompressionComplete();
        // swap buffers and start compressing in the other thread
        {
          std::unique_lock<std::mutex> lock(mutex_);
          buffer_.resize(serialized_size);
          std::swap(buffer_, buffer_compressing_);
          has_data_to_compress_ = true;
          compression_done_ = false;
        }
        cv_ready_to_compress_.notify_one();
      }
      // clean up current buffer and buffer_view
      buffer_.resize(kChunkSize);
      buffer_view = BufferView(buffer_);
      points_in_current_chunk = 0;
      serialized_size = 0;
      chunks_count++;
    }
  }

  waitForCompressionComplete();

  // Return 0 as the actual size is handled by the vector version
  return compressed_size_;
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

  if (info.encoding_opt == EncodingOptions::NONE) {
    for (const auto& field : info.fields) {
      decoders_.push_back(std::make_unique<FieldDecoderCopy>(field.offset, field.type));
    }
    return;
  }

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
  if (memcmp(compressed_data.data(), kMagicHeader, kMagicHeaderLength) == 0) {
    throw std::runtime_error("compressed_data contains the header. You should use DecodeHeader first");
  }

  if (info.version >= 3) {
    size_t chunk_index = 0;
    while (!compressed_data.empty()) {
      uint32_t chunk_size = 0;
      Cloudini::decode(compressed_data, chunk_size);
      if (chunk_size > compressed_data.size()) {
        throw std::runtime_error("Invalid chunk size found while decoding");
      }
      ConstBufferView chunk_view(compressed_data.data(), chunk_size);
      decodeChunk(info, chunk_view, output);
      compressed_data.trim_front(chunk_size);
      chunk_index++;
    }
  } else {
    decodeChunk(info, compressed_data, output);
  }
}

void PointcloudDecoder::decodeChunk(const EncodingInfo& info, ConstBufferView chunk_data, BufferView& output_buffer) {
  // allocate sufficient space in the buffer
  decompressed_buffer_.resize(info.width * info.height * info.point_step);

  // start decompressing using "compression_opt" param.
  // Note that compressed_data doesn't contan the header anymore.
  // Decompressed data will be stored in buffer_
  switch (info.compression_opt) {
    case CompressionOption::LZ4: {
      const auto* src_ptr = reinterpret_cast<const char*>(chunk_data.data());
      auto* buffer_ptr = reinterpret_cast<char*>(decompressed_buffer_.data());
      const int decompressed_size =
          LZ4_decompress_safe(src_ptr, buffer_ptr, chunk_data.size(), decompressed_buffer_.size());
      if (decompressed_size < 0) {
        throw std::runtime_error("LZ4 decompression failed");
      }
      decompressed_buffer_.resize(decompressed_size);
    } break;

    case CompressionOption::ZSTD: {
      const size_t decompressed_size = ZSTD_decompress(
          decompressed_buffer_.data(), decompressed_buffer_.size(), chunk_data.data(), chunk_data.size());
      if (ZSTD_isError(decompressed_size)) {
        throw std::runtime_error("ZSTD decompression failed: " + std::string(ZSTD_getErrorName(decompressed_size)));
      }
      decompressed_buffer_.resize(decompressed_size);
    } break;

    default:
      break;  // do nothing
  }

  //----------------------------------------------------------------------
  // decode the data (first stage).
  auto encoded_view = (info.compression_opt == CompressionOption::NONE) ? ConstBufferView(chunk_data)
                                                                        : ConstBufferView(decompressed_buffer_);

  size_t decoded_points = 0;
  while (encoded_view.size() > 0) {
    if (output_buffer.size() < info.point_step) {
      throw std::runtime_error("Output buffer is too small to hold the decoded data");
    }
    BufferView point_view(output_buffer.data(), info.point_step);
    for (auto& decoder : decoders_) {
      decoder->decode(encoded_view, point_view);
    }
    output_buffer.trim_front(info.point_step);
    decoded_points++;
  }
}

}  // namespace Cloudini
