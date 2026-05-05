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

#include "codec_common.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "lz4.h"
#include "zstd.h"

namespace Cloudini::detail {

size_t MaxSerializedFieldSize(const PointField& field, EncodingOptions encoding_opt) {
  switch (field.type) {
    case FieldType::INT16:
    case FieldType::UINT16:
    case FieldType::INT32:
    case FieldType::UINT32:
    case FieldType::INT64:
    case FieldType::UINT64:
      return 10;  // worst-case signed varint64 size
    case FieldType::FLOAT32:
      if (encoding_opt == EncodingOptions::LOSSY && field.resolution.has_value()) {
        return 10;  // quantized int64 delta as varint
      }
      // Gorilla worst-case bits: 1 (flag) + 1 (control) + 5 (leading) + 6 (length) + 32 (bits) = 45 bits.
      // With byte-alignment slop from other fields + final flush, 7 bytes is a safe upper bound per value.
      return 7;
    case FieldType::FLOAT64:
      if (encoding_opt == EncodingOptions::LOSSY && field.resolution.has_value()) {
        return 10;  // quantized int64 delta as varint
      }
      // Gorilla worst-case bits: 1 + 1 + 5 + 6 + 64 = 77 bits.
      return 11;  // XOR residual or Gorilla bit-pack, rounded up
    case FieldType::INT8:
    case FieldType::UINT8:
      return 1;
    default:
      throw std::runtime_error(
          "Unsupported field type '" + field.name + "' (type=" + std::to_string(static_cast<int>(field.type)) +
          ") in MaxSerializedFieldSize");
  }
}

size_t MaxSerializedPointSize(const EncodingInfo& info) {
  size_t max_point_size = 0;
  for (const auto& field : info.fields) {
    max_point_size += MaxSerializedFieldSize(field, info.encoding_opt);
  }
  return max_point_size;
}

size_t LeadingLossyFloatFieldCount(const EncodingInfo& info) {
  if (info.encoding_opt != EncodingOptions::LOSSY) {
    return 0;
  }

  size_t floats_count = 0;
  for (const auto& field : info.fields) {
    if (field.type != FieldType::FLOAT32 || !field.resolution.has_value()) {
      break;
    }
    ++floats_count;
  }
  return (floats_count == 3 || floats_count == 4) ? floats_count : 0;
}

size_t AppendLeadingLossyFloatEncoder(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldEncoder>>& encoders) {
  const size_t floats_count = LeadingLossyFloatFieldCount(info);
  if (floats_count == 0) {
    return 0;
  }

  std::vector<FieldEncoderFloatN_Lossy::FieldData> field_data;
  field_data.reserve(floats_count);
  for (size_t i = 0; i < floats_count; ++i) {
    field_data.emplace_back(info.fields[i].offset, info.fields[i].resolution.value());
  }
  encoders.push_back(std::make_unique<FieldEncoderFloatN_Lossy>(field_data));
  return floats_count;
}

size_t AppendLeadingLossyFloatDecoder(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders) {
  const size_t floats_count = LeadingLossyFloatFieldCount(info);
  if (floats_count == 0) {
    return 0;
  }

  std::vector<FieldDecoderFloatN_Lossy::FieldData> field_data;
  field_data.reserve(floats_count);
  for (size_t i = 0; i < floats_count; ++i) {
    field_data.emplace_back(info.fields[i].offset, info.fields[i].resolution.value());
  }
  decoders.push_back(std::make_unique<FieldDecoderFloatN_Lossy>(field_data));
  return floats_count;
}

std::unique_ptr<FieldEncoder> CreateCompatibleEncoder(const EncodingInfo& info, const PointField& field) {
  const auto offset = field.offset;
  switch (field.type) {
    case FieldType::FLOAT32:
      if (info.encoding_opt == EncodingOptions::LOSSY && field.resolution.has_value()) {
        return std::make_unique<FieldEncoderFloat_Lossy<float>>(offset, *field.resolution);
      } else if (info.encoding_opt == EncodingOptions::LOSSLESS) {
        return std::make_unique<FieldEncoderFloat_XOR<float>>(offset);
      }
      return std::make_unique<FieldEncoderCopy>(offset, field.type);

    case FieldType::FLOAT64:
      if (info.encoding_opt == EncodingOptions::LOSSY && field.resolution.has_value()) {
        return std::make_unique<FieldEncoderFloat_Lossy<double>>(offset, *field.resolution);
      } else if (!field.resolution.has_value() && info.version >= 4) {
        return std::make_unique<FieldEncoderFloat_Gorilla<double>>(offset);
      }
      return std::make_unique<FieldEncoderFloat_XOR<double>>(offset);

    case FieldType::INT16:
      return std::make_unique<FieldEncoderInt<int16_t>>(offset);
    case FieldType::INT32:
      return std::make_unique<FieldEncoderInt<int32_t>>(offset);
    case FieldType::UINT16:
      return std::make_unique<FieldEncoderInt<uint16_t>>(offset);
    case FieldType::UINT32:
      return std::make_unique<FieldEncoderInt<uint32_t>>(offset);
    case FieldType::UINT64:
      return std::make_unique<FieldEncoderInt<uint64_t>>(offset);
    case FieldType::INT64:
      return std::make_unique<FieldEncoderInt<int64_t>>(offset);
    case FieldType::INT8:
    case FieldType::UINT8:
      return std::make_unique<FieldEncoderCopy>(offset, field.type);
    default:
      throw std::runtime_error("Unsupported field type:" + std::to_string(static_cast<int>(field.type)));
  }
}

std::unique_ptr<FieldDecoder> CreateCompatibleDecoder(const EncodingInfo& info, const PointField& field) {
  const auto offset = field.offset;
  switch (field.type) {
    case FieldType::FLOAT32:
      if (info.encoding_opt == EncodingOptions::LOSSY && field.resolution) {
        return std::make_unique<FieldDecoderFloat_Lossy<float>>(offset, *field.resolution);
      } else if (info.encoding_opt == EncodingOptions::LOSSLESS) {
        return std::make_unique<FieldDecoderFloat_XOR<float>>(offset);
      } else if (field.resolution) {
        // Legacy compatibility: if resolution is set but encoding_opt is not LOSSY.
        return std::make_unique<FieldDecoderFloat_Lossy<float>>(offset, *field.resolution);
      }
      return std::make_unique<FieldDecoderCopy>(field.offset, field.type);

    case FieldType::FLOAT64:
      if (info.encoding_opt == EncodingOptions::LOSSY && field.resolution) {
        return std::make_unique<FieldDecoderFloat_Lossy<double>>(offset, *field.resolution);
      } else if (field.resolution && info.encoding_opt != EncodingOptions::LOSSLESS) {
        return std::make_unique<FieldDecoderFloat_Lossy<double>>(offset, *field.resolution);
      } else if (!field.resolution && info.version >= 4) {
        // FLOAT64 without a resolution is treated as lossless regardless of encoding_opt.
        return std::make_unique<FieldDecoderFloat_Gorilla<double>>(offset);
      }
      return std::make_unique<FieldDecoderFloat_XOR<double>>(offset);

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
}

void ResetEncoders(std::vector<std::unique_ptr<FieldEncoder>>& encoders) {
  for (auto& encoder : encoders) {
    encoder->reset();
  }
}

void ResetDecoders(std::vector<std::unique_ptr<FieldDecoder>>& decoders) {
  for (auto& decoder : decoders) {
    decoder->reset();
  }
}

size_t FlushEncoders(std::vector<std::unique_ptr<FieldEncoder>>& encoders, BufferView& output) {
  size_t serialized_size = 0;
  for (auto& encoder : encoders) {
    serialized_size += encoder->flush(output);
  }
  return serialized_size;
}

uint32_t CompressChunk(CompressionOption compression, ConstBufferView input, BufferView& output) {
  if (input.size() > std::numeric_limits<uint32_t>::max()) {
    throw std::runtime_error("Chunk too large");
  }

  uint32_t chunk_size = 0;
  switch (compression) {
    case CompressionOption::LZ4: {
      if (input.size() > static_cast<size_t>(std::numeric_limits<int>::max()) ||
          output.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("Chunk size too large for LZ4");
      }
      const int cs = LZ4_compress_default(
          reinterpret_cast<const char*>(input.data()), reinterpret_cast<char*>(output.data()),
          static_cast<int>(input.size()), static_cast<int>(output.size()));
      if (cs <= 0) {
        throw std::runtime_error("LZ4 compression failed");
      }
      chunk_size = static_cast<uint32_t>(cs);
    } break;

    case CompressionOption::ZSTD: {
      const size_t cs = ZSTD_compress(output.data(), output.size(), input.data(), input.size(), 1);
      if (ZSTD_isError(cs)) {
        throw std::runtime_error("ZSTD compression failed");
      }
      if (cs > std::numeric_limits<uint32_t>::max()) {
        throw std::runtime_error("Compressed chunk too large");
      }
      chunk_size = static_cast<uint32_t>(cs);
    } break;

    default:
      throw std::runtime_error("Unsupported compression option");
  }

  output.trim_front(chunk_size);
  return chunk_size;
}

ConstBufferView DecompressChunk(
    CompressionOption compression, ConstBufferView chunk_data,
    std::vector<uint8_t>& decompressed_buffer, size_t max_decompressed_size) {
  switch (compression) {
    case CompressionOption::NONE:
      return chunk_data;

    case CompressionOption::LZ4: {
      if (chunk_data.size() > static_cast<size_t>(std::numeric_limits<int>::max()) ||
          max_decompressed_size > static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("Chunk size too large for LZ4");
      }
      if (decompressed_buffer.size() < max_decompressed_size) {
        decompressed_buffer.resize(max_decompressed_size);
      }
      const int decompressed_size = LZ4_decompress_safe(
          reinterpret_cast<const char*>(chunk_data.data()),
          reinterpret_cast<char*>(decompressed_buffer.data()), static_cast<int>(chunk_data.size()),
          static_cast<int>(max_decompressed_size));
      if (decompressed_size < 0) {
        throw std::runtime_error("LZ4 decompression failed");
      }
      return ConstBufferView(decompressed_buffer.data(), static_cast<size_t>(decompressed_size));
    }

    case CompressionOption::ZSTD: {
      if (decompressed_buffer.size() < max_decompressed_size) {
        decompressed_buffer.resize(max_decompressed_size);
      }
      const size_t decompressed_size = ZSTD_decompress(
          decompressed_buffer.data(), max_decompressed_size, chunk_data.data(), chunk_data.size());
      if (ZSTD_isError(decompressed_size)) {
        throw std::runtime_error("ZSTD decompression failed: " + std::string(ZSTD_getErrorName(decompressed_size)));
      }
      return ConstBufferView(decompressed_buffer.data(), decompressed_size);
    }
  }

  throw std::runtime_error("Unsupported compression option");
}

}  // namespace Cloudini::detail
