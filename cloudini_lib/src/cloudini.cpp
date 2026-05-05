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

#include <algorithm>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "chunk_writer.hpp"
#include "codec_common.hpp"
#include "cloudini_lib/encoding_utils.hpp"
#include "cloudini_lib/yaml_parser.hpp"
#include "v4_codec.hpp"
#include "v5_codec.hpp"
#include "lz4.h"
#include "zstd.h"

namespace Cloudini {

namespace {

void ensureScratchBuffer(std::unique_ptr<uint8_t[]>& buffer, size_t& capacity, size_t required_capacity) {
  if (capacity >= required_capacity) {
    return;
  }
  buffer.reset(new uint8_t[required_capacity]);
  capacity = required_capacity;
}

}  // namespace

const char* ToString(const FieldType& type) {
  switch (type) {
    case FieldType::INT8:
      return "INT8";
    case FieldType::UINT8:
      return "UINT8";
    case FieldType::INT16:
      return "INT16";
    case FieldType::UINT16:
      return "UINT16";
    case FieldType::INT32:
      return "INT32";
    case FieldType::UINT32:
      return "UINT32";
    case FieldType::FLOAT32:
      return "FLOAT32";
    case FieldType::FLOAT64:
      return "FLOAT64";
    case FieldType::INT64:
      return "INT64";
    case FieldType::UINT64:
      return "UINT64";

    case FieldType::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

const char* ToString(const EncodingOptions& opt) {
  switch (opt) {
    case EncodingOptions::NONE:
      return "NONE";
    case EncodingOptions::LOSSY:
      return "LOSSY";
    case EncodingOptions::LOSSLESS:
      return "LOSSLESS";
    default:
      return "UNKNOWN";
  }
}

const char* ToString(const CompressionOption& opt) {
  switch (opt) {
    case CompressionOption::NONE:
      return "NONE";
    case CompressionOption::LZ4:
      return "LZ4";
    case CompressionOption::ZSTD:
      return "ZSTD";
    default:
      return "UNKNOWN";
  }
}

EncodingOptions EncodingOptionsFromString(std::string_view str) {
  if (str == "NONE") {
    return EncodingOptions::NONE;
  } else if (str == "LOSSY") {
    return EncodingOptions::LOSSY;
  } else if (str == "LOSSLESS") {
    return EncodingOptions::LOSSLESS;
  } else {
    int val = std::stoi(std::string(str));
    if (val >= static_cast<int>(EncodingOptions::NONE) && val <= static_cast<int>(EncodingOptions::LOSSLESS)) {
      return static_cast<EncodingOptions>(val);
    }
  }
  throw std::runtime_error("Invalid EncodingOptions string: " + std::string(str));
}

FieldType FieldTypeFromString(std::string_view str) {
  if (str == "INT8") {
    return FieldType::INT8;
  } else if (str == "UINT8") {
    return FieldType::UINT8;
  } else if (str == "INT16") {
    return FieldType::INT16;
  } else if (str == "UINT16") {
    return FieldType::UINT16;
  } else if (str == "INT32") {
    return FieldType::INT32;
  } else if (str == "UINT32") {
    return FieldType::UINT32;
  } else if (str == "FLOAT32") {
    return FieldType::FLOAT32;
  } else if (str == "FLOAT64") {
    return FieldType::FLOAT64;
  } else if (str == "INT64") {
    return FieldType::INT64;
  } else if (str == "UINT64") {
    return FieldType::UINT64;
  } else {
    int val = std::stoi(std::string(str));
    if (val >= static_cast<int>(FieldType::UNKNOWN) && val <= static_cast<int>(FieldType::UINT64)) {
      return static_cast<FieldType>(val);
    }
  }
  throw std::runtime_error("Invalid FieldType string: " + std::string(str));
}

CompressionOption CompressionOptionFromString(std::string_view str) {
  if (str == "NONE") {
    return CompressionOption::NONE;
  } else if (str == "LZ4") {
    return CompressionOption::LZ4;
  } else if (str == "ZSTD") {
    return CompressionOption::ZSTD;
  } else {
    int val = std::stoi(std::string(str));
    if (val >= static_cast<int>(CompressionOption::NONE) && val <= static_cast<int>(CompressionOption::ZSTD)) {
      return static_cast<CompressionOption>(val);
    }
  }
  throw std::runtime_error("Invalid CompressionOption string: " + std::string(str));
}

std::string EncodingInfoToYAML(const EncodingInfo& info) {
  std::ostringstream yaml;
  yaml << "version: " << static_cast<int>(info.version) << "\n";
  yaml << "width: " << info.width << "\n";
  yaml << "height: " << info.height << "\n";
  yaml << "point_step: " << info.point_step << "\n";
  yaml << "encoding_opt: " << ToString(info.encoding_opt) << "\n";
  yaml << "compression_opt: " << ToString(info.compression_opt) << "\n";
  if (!info.encoding_config.empty()) {
    yaml << "encoding_config: " << info.encoding_config << "\n";
  }

  yaml << "fields:\n";

  for (const auto& field : info.fields) {
    yaml << "  - name: " << field.name << "\n";
    yaml << "    offset: " << field.offset << "\n";
    yaml << "    type: " << ToString(field.type) << "\n";
    if (field.resolution.has_value()) {
      yaml << "    resolution: " << field.resolution.value() << "\n";
    } else {
      yaml << "    resolution: null\n";
    }
  }
  return yaml.str();
}

EncodingInfo EncodingInfoFromYAML(std::string_view yaml) {
  EncodingInfo info;

  // Parse YAML using the new parser
  auto root = YAML::parse(yaml);

  // Read top-level fields
  info.version = root["version"].as<uint8_t>();
  info.width = root["width"].as<uint32_t>();
  info.height = root["height"].as<uint32_t>();
  info.point_step = root["point_step"].as<uint32_t>();
  info.encoding_opt = EncodingOptionsFromString(root["encoding_opt"].as<std::string_view>());
  info.compression_opt = CompressionOptionFromString(root["compression_opt"].as<std::string_view>());

  // encoding_config might be empty in older versions
  if (!root["encoding_config"].isNull() && root["encoding_config"].isString()) {
    info.encoding_config = root["encoding_config"].as<std::string>();
  }

  // Parse fields array
  const auto& fields_node = root["fields"];
  if (fields_node.isSequence()) {
    for (size_t i = 0; i < fields_node.size(); ++i) {
      const auto& field_node = fields_node[i];
      PointField field;
      field.name = field_node["name"].as<std::string>();
      field.offset = field_node["offset"].as<uint32_t>();
      field.type = FieldTypeFromString(field_node["type"].as<std::string_view>());

      std::string res_str = field_node["resolution"].as<std::string>();
      if (res_str != "null") {
        field.resolution = std::stof(res_str);
      }
      info.fields.push_back(field);
    }
  }

  return info;
}

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

size_t MaxCompressedSize(const EncodingInfo& info, size_t points_count, bool include_header) {
  if (info.point_step == 0) {
    throw std::runtime_error("point_step cannot be 0");
  }

  constexpr size_t chunk_points = detail::kPointsPerChunk;
  const size_t chunks_count = (points_count / chunk_points) + ((points_count % chunk_points) ? 1 : 0);

  const size_t max_serialized_point_size = detail::MaxSerializedPointSize(info);
  size_t total_size = include_header ? (kMagicHeaderLength + 2 + 1 + EncodingInfoToYAML(info).size() + 1) : 0;

  size_t points_left = points_count;
  for (size_t chunk_idx = 0; chunk_idx < chunks_count; ++chunk_idx) {
    const size_t points_in_chunk = std::min(points_left, chunk_points);
    points_left -= points_in_chunk;
    size_t max_chunk_input_size = points_in_chunk * max_serialized_point_size;
    if (detail::UsesV5Codec(info)) {
      // V5 adaptive integer sections add mode/header bytes. Adaptive sections
      // compete using their full encoded size, so any selected mode remains
      // bounded by the delta-varint section plus this fixed slack.
      max_chunk_input_size += info.fields.size() * 32u + 1024u;
    }

    total_size += sizeof(uint32_t);  // chunk size prefix
    switch (info.compression_opt) {
      case CompressionOption::NONE:
        total_size += max_chunk_input_size;
        break;
      case CompressionOption::LZ4:
        if (max_chunk_input_size > static_cast<size_t>(std::numeric_limits<int>::max())) {
          throw std::runtime_error("Chunk size too large for LZ4");
        }
        total_size += static_cast<size_t>(LZ4_compressBound(static_cast<int>(max_chunk_input_size)));
        break;
      case CompressionOption::ZSTD:
        total_size += ZSTD_compressBound(max_chunk_input_size);
        break;
      default:
        throw std::runtime_error("Unsupported compression option in MaxCompressedSize");
    }
  }

  return total_size;
}

void EncodeHeader(const EncodingInfo& header, std::vector<uint8_t>& output, HeaderEncoding encoding) {
  output.clear();

  auto write_magic = [&header](BufferView& output_buffer) {
    memcpy(output_buffer.data(), kMagicHeader, kMagicHeaderLength);
    output_buffer.trim_front(kMagicHeaderLength);
    // version as two ASCII digits. Respects header.version so callers can request
    // an older wire format (e.g. v3) for backward compatibility with old readers.
    const uint8_t v = header.version;
    encode<char>('0' + (v / 10), output_buffer);
    encode<char>('0' + (v % 10), output_buffer);
  };

  if (encoding == HeaderEncoding::YAML) {
    const auto yaml_str = EncodingInfoToYAML(header);
    // magic + \n + yaml + \0
    output.resize(yaml_str.size() + 2 + kMagicHeaderLength + 2);
    BufferView output_buffer(output.data(), output.size());

    write_magic(output_buffer);
    encode('\n', output_buffer);  // newline
    memcpy(output_buffer.data(), yaml_str.data(), yaml_str.size());
    output_buffer.trim_front(yaml_str.size());
    encode('\0', output_buffer);  // null terminator
  } else {
    // Binary encoding
    output.resize(ComputeHeaderSize(header.fields));
    BufferView output_buffer(output.data(), output.size());
    write_magic(output_buffer);

    encode(header.width, output_buffer);
    encode(header.height, output_buffer);
    encode(header.point_step, output_buffer);

    encode(static_cast<uint8_t>(header.encoding_opt), output_buffer);
    encode(static_cast<uint8_t>(header.compression_opt), output_buffer);
    encode(static_cast<uint16_t>(header.fields.size()), output_buffer);

    for (const auto& field : header.fields) {
      encode(field.name, output_buffer);
      encode(field.offset, output_buffer);
      encode(static_cast<uint8_t>(field.type), output_buffer);
      if (field.resolution) {
        encode(*field.resolution, output_buffer);
      } else {
        const float res = -1.0;
        encode(res, output_buffer);
      }
    }
  }
}

auto char_to_num = [](char c) -> uint8_t {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  return 0;
};

EncodingInfo DecodeHeader(ConstBufferView& input) {
  if (input.size() < static_cast<size_t>(kMagicHeaderLength + 2)) {
    throw std::runtime_error("Input too small to contain Cloudini header");
  }
  const uint8_t* buff = input.data();

  // check the magic header
  if (memcmp(buff, kMagicHeader, kMagicHeaderLength) != 0) {
    std::string fist_bytes = std::string(reinterpret_cast<const char*>(buff), kMagicHeaderLength);
    throw std::runtime_error(std::string("Invalid magic header. Expecter 'CLOUDINI_V', got: ") + fist_bytes);
  }
  input.trim_front(kMagicHeaderLength);

  // next 2 bytes contain the version number as string
  const uint8_t version = char_to_num(input.data()[0]) * 10 + char_to_num(input.data()[1]);
  input.trim_front(2);

  if (version < 2 || version > kEncodingVersion) {
    throw std::runtime_error(
        "Unsupported encoding version. Current is:" + std::to_string(kEncodingVersion) +
        ", got: " + std::to_string(version));
  }
  // Note: version 4 adds Gorilla bit-packing for lossless FLOAT32/FLOAT64 XOR residuals.
  // Versions 2 and 3 keep the raw-XOR path (8 bytes per double, 4 bytes per float).

  // check if encoded as YAML (starts with newline after version, then non-brace character)
  if (input.size() >= 2 && input.data()[0] == '\n' && input.data()[1] != '{') {
    // YAML encoded header
    input.trim_front(1);  // consume newline
    std::string_view yaml_str(reinterpret_cast<const char*>(input.data()), input.size());
    size_t null_pos = yaml_str.find('\0');
    if (null_pos == std::string::npos) {
      throw std::runtime_error("Malformed YAML header: missing null terminator");
    }
    yaml_str = yaml_str.substr(0, null_pos);
    input.trim_front(null_pos + 1);  // consume header + null terminator
    EncodingInfo yaml_header = EncodingInfoFromYAML(yaml_str);
    // The magic-header version is authoritative. YAML's parseScalar<uint8_t> reads a
    // single character (e.g. "3" -> 51), so the YAML-parsed info.version is unreliable.
    yaml_header.version = version;
    return yaml_header;
  }

  // Binary encoded header
  EncodingInfo header;
  header.version = version;

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
  EncodeHeader(info_, header_);

  if (!detail::UsesV5Codec(info_)) {
    detail::BuildV4Encoders(info_, encoders_);
  }

  if (info_.compression_opt != CompressionOption::NONE && info_.use_threads) {
    compressing_thread_ = std::thread(&PointcloudEncoder::compressionWorker, this);
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
  try {
    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_ready_to_compress_.wait(lock, [this] { return has_data_to_compress_ || should_exit_; });

        if (should_exit_) {
          break;
        }
        has_data_to_compress_ = false;
      }

      uint8_t* compressed_chunk_size_ptr = output_view_.data();
      output_view_.trim_front(sizeof(uint32_t));

      ConstBufferView stage1_data(buffer_compressing_.get(), buffer_compressing_size_);
      BufferView compressed_output(output_view_.data(), output_view_.size());
      const uint32_t chunk_size = detail::CompressChunk(info_.compression_opt, stage1_data, compressed_output);
      output_view_ = compressed_output;
      memcpy(compressed_chunk_size_ptr, &chunk_size, sizeof(uint32_t));

      {
        std::lock_guard<std::mutex> lock(mutex_);
        compressed_size_ += chunk_size + sizeof(uint32_t);
        compression_done_ = true;
      }

      cv_done_compressing_.notify_one();
    }
  } catch (...) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      worker_failed_ = true;
      worker_exception_ = std::current_exception();
    }
    cv_done_compressing_.notify_all();
  }
}

void PointcloudEncoder::waitForCompressionComplete() {
  std::unique_lock<std::mutex> lock(mutex_);
  cv_done_compressing_.wait(lock, [this] { return compression_done_ || worker_failed_; });
  if (worker_failed_) {
    std::rethrow_exception(worker_exception_);
  }
}

size_t PointcloudEncoder::encode(ConstBufferView cloud_data, std::vector<uint8_t>& output) {
  if (info_.point_step == 0) {
    throw std::runtime_error("point_step cannot be 0");
  }
  if (cloud_data.size() % info_.point_step != 0) {
    throw std::runtime_error("Input cloud_data size is not a multiple of point_step");
  }

  const size_t points_count = cloud_data.size() / info_.point_step;
  output.resize(MaxCompressedSize(info_, points_count, true));
  // write the header
  BufferView output_view(output.data(), output.size());
  memcpy(output_view.data(), header_.data(), header_.size());
  output_view.trim_front(header_.size());

  const size_t added_bytes = encode(cloud_data, output_view, false);
  const size_t new_size = header_.size() + added_bytes;
  output.resize(new_size);
  return new_size;
}

size_t PointcloudEncoder::encode(ConstBufferView cloud_data, BufferView& output, bool write_header) {
  if (info_.point_step == 0) {
    throw std::runtime_error("point_step cannot be 0");
  }
  if (cloud_data.size() % info_.point_step != 0) {
    throw std::runtime_error("Input cloud_data size is not a multiple of point_step");
  }
  const size_t points_count = cloud_data.size() / info_.point_step;
  // Use header_.size() directly to avoid redundant YAML serialization inside MaxCompressedSize
  const size_t required_capacity = MaxCompressedSize(info_, points_count, false) + (write_header ? header_.size() : 0);
  if (output.size() < required_capacity) {
    throw std::runtime_error("Output buffer too small for worst-case compressed size");
  }

  if (info_.compression_opt != CompressionOption::NONE && info_.use_threads) {
    bool need_respawn = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      need_respawn = worker_failed_;
    }
    if (need_respawn) {
      if (compressing_thread_.joinable()) {
        compressing_thread_.join();
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        worker_failed_ = false;
        worker_exception_ = nullptr;
      }
      compressing_thread_ = std::thread(&PointcloudEncoder::compressionWorker, this);
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    compressed_size_ = 0;
    should_exit_ = false;
    has_data_to_compress_ = false;
    compression_done_ = true;
    buffer_compressing_size_ = 0;
  }
  output_view_ = output;

  // Copy the header at the beginning of the output
  if (write_header) {
    memcpy(output_view_.data(), header_.data(), header_.size());
    compressed_size_ += header_.size();
    output_view_.trim_front(header_.size());
  }

  auto write_stage1_chunk = [&](size_t serialized_size) {
    ConstBufferView stage1_data(buffer_.get(), serialized_size);
    if (info_.compression_opt == CompressionOption::NONE || !info_.use_threads) {
      compressed_size_ += detail::WriteStage1Chunk(info_, stage1_data, output_view_);
      return;
    }
    waitForCompressionComplete();
    {
      std::unique_lock<std::mutex> lock(mutex_);
      buffer_compressing_size_ = serialized_size;
      std::swap(buffer_, buffer_compressing_);
      std::swap(buffer_capacity_, buffer_compressing_capacity_);
      has_data_to_compress_ = true;
      compression_done_ = false;
    }
    cv_ready_to_compress_.notify_one();
  };

  if (detail::UsesV5Codec(info_)) {
    const size_t stage_capacity = detail::V5StageBufferSize(info_, detail::kPointsPerChunk);
    ensureScratchBuffer(buffer_, buffer_capacity_, stage_capacity);
    if (info_.compression_opt != CompressionOption::NONE && info_.use_threads) {
      ensureScratchBuffer(buffer_compressing_, buffer_compressing_capacity_, stage_capacity);
    }
    auto get_stage_buffer = [this] { return BufferView(buffer_.get(), buffer_capacity_); };
    detail::EncodeV5Stage1(
        info_, cloud_data, points_count, detail::kPointsPerChunk, get_stage_buffer, write_stage1_chunk);
  } else {
    const size_t max_per_point = detail::MaxSerializedPointSize(info_);
    const size_t stage_capacity =
        detail::kPointsPerChunk * std::max<size_t>(info_.point_step, max_per_point);
    ensureScratchBuffer(buffer_, buffer_capacity_, stage_capacity);
    if (info_.compression_opt != CompressionOption::NONE && info_.use_threads) {
      ensureScratchBuffer(buffer_compressing_, buffer_compressing_capacity_, stage_capacity);
    }

    ConstBufferView remaining = cloud_data;
    while (!remaining.empty()) {
      BufferView stage_view(buffer_.get(), buffer_capacity_);
      const size_t serialized_size = detail::EncodeV4Stage1Chunk(
          info_, encoders_, remaining, detail::kPointsPerChunk, stage_view);
      write_stage1_chunk(serialized_size);
    }
  }

  if (info_.use_threads && info_.compression_opt != CompressionOption::NONE) {
    waitForCompressionComplete();
  }

  // Return 0 as the actual size is handled by the vector version
  return compressed_size_;
}

//------------------------------------------------------------------------------------------

void PointcloudDecoder::updateDecoders(const EncodingInfo& info) {
  if (detail::UsesV5Codec(info)) {
    detail::BuildV5Decoders(info, decoders_, min_encoded_point_bytes_);
  } else {
    detail::BuildV4Decoders(info, decoders_, min_encoded_point_bytes_);
  }
}

void PointcloudDecoder::decode(const EncodingInfo& info, ConstBufferView compressed_data, BufferView output) {
  // read the header
  updateDecoders(info);

  // check if the first bytes are the magic header. if they are, skip them
  if (compressed_data.size() >= static_cast<size_t>(kMagicHeaderLength) &&
      memcmp(compressed_data.data(), kMagicHeader, kMagicHeaderLength) == 0) {
    throw std::runtime_error("compressed_data contains the header. You should use DecodeHeader first");
  }

  if (info.version >= 3) {
    size_t points_remaining = static_cast<size_t>(info.width) * static_cast<size_t>(info.height);
    while (!compressed_data.empty()) {
      if (points_remaining == 0) {
        throw std::runtime_error("Encoded data contains more chunks than declared points");
      }
      uint32_t chunk_size = 0;
      Cloudini::decode(compressed_data, chunk_size);
      if (chunk_size > compressed_data.size()) {
        throw std::runtime_error("Invalid chunk size found while decoding");
      }
      ConstBufferView chunk_view(compressed_data.data(), chunk_size);
      const size_t points_in_chunk = std::min(points_remaining, detail::kPointsPerChunk);
      decodeChunk(info, chunk_view, output, points_in_chunk);
      compressed_data.trim_front(chunk_size);
      points_remaining -= points_in_chunk;
    }
    if (points_remaining != 0) {
      throw std::runtime_error("Encoded data ended before all declared points were decoded");
    }
  } else {
    decodeChunk(info, compressed_data, output, /*expected_points=*/0);
  }
}

void PointcloudDecoder::decodeChunk(
    const EncodingInfo& info, ConstBufferView chunk_data, BufferView& output_buffer, size_t expected_points) {
  const size_t max_decompressed_size =
      static_cast<size_t>(info.width) * static_cast<size_t>(info.height) * info.point_step;
  ConstBufferView encoded_view = detail::DecompressChunk(
      info.compression_opt, chunk_data, decompressed_buffer_, max_decompressed_size);

  if (detail::UsesV5Codec(info)) {
    detail::DecodeV5Stage1Chunk(info, decoders_, encoded_view, output_buffer, expected_points);
  } else {
    detail::DecodeV4Stage1Chunk(
        decoders_, min_encoded_point_bytes_, encoded_view, output_buffer,
        info.point_step, expected_points);
  }
}

}  // namespace Cloudini
