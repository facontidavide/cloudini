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
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <type_traits>

#include "cloudini_lib/encoding_utils.hpp"
#include "cloudini_lib/field_decoder.hpp"
#include "cloudini_lib/field_encoder.hpp"
#include "cloudini_lib/yaml_parser.hpp"
#include "lz4.h"
#include "zstd.h"

namespace Cloudini {

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
      return sizeof(float);
    case FieldType::FLOAT64:
      if (encoding_opt == EncodingOptions::LOSSY && field.resolution.has_value()) {
        return 10;  // quantized int64 delta as varint
      }
      return sizeof(double);  // XOR residual
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

size_t MaxCompressedSize(const EncodingInfo& info, size_t points_count, bool include_header) {
  if (info.point_step == 0) {
    throw std::runtime_error("point_step cannot be 0");
  }

  constexpr size_t chunk_points = 32 * 1024;
  const size_t chunks_count = (points_count / chunk_points) + ((points_count % chunk_points) ? 1 : 0);

  const size_t max_serialized_point_size = MaxSerializedPointSize(info);
  size_t total_size = include_header ? (kMagicHeaderLength + 2 + 1 + EncodingInfoToYAML(info).size() + 1) : 0;

  size_t points_left = points_count;
  for (size_t chunk_idx = 0; chunk_idx < chunks_count; ++chunk_idx) {
    const size_t points_in_chunk = std::min(points_left, chunk_points);
    points_left -= points_in_chunk;
    const size_t max_chunk_input_size = points_in_chunk * max_serialized_point_size;

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

  auto write_magic = [](BufferView& output_buffer) {
    memcpy(output_buffer.data(), kMagicHeader, kMagicHeaderLength);
    output_buffer.trim_front(kMagicHeaderLength);
    // version as two ASCII digits
    encode<char>('0' + (kEncodingVersion / 10), output_buffer);
    encode<char>('0' + (kEncodingVersion % 10), output_buffer);
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
    return EncodingInfoFromYAML(yaml_str);
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

  if (info_.encoding_opt == EncodingOptions::NONE) {
    for (const auto& field : info_.fields) {
      encoders_.push_back(std::make_unique<FieldEncoderCopy>(field.offset, field.type));
    }
    // Start the compression worker thread if we're using compression
    compressing_thread_ = std::thread(&PointcloudEncoder::compressionWorker, this);
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
  // Start the compression worker thread if we're using compression
  compressing_thread_ = std::thread(&PointcloudEncoder::compressionWorker, this);
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
          throw std::runtime_error("Unsupported compression option in worker thread");
      }

      output_view_.trim_front(chunk_size);

      // write the size of the chunk
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

  // If the worker thread died from a previous failure, join it and re-spawn
  if (info_.compression_opt != CompressionOption::NONE) {
    bool need_respawn = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      need_respawn = worker_failed_;
    }
    if (need_respawn) {
      if (compressing_thread_.joinable()) {
        compressing_thread_.join();
      }
      // Reset failure state and re-spawn
      {
        std::lock_guard<std::mutex> lock(mutex_);
        worker_failed_ = false;
        worker_exception_ = nullptr;
      }
      compressing_thread_ = std::thread(&PointcloudEncoder::compressionWorker, this);
    }
  }

  // Reset the state of the encoders and the class attributes
  for (auto& encoder : encoders_) {
    encoder->reset();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    compressed_size_ = 0;
    should_exit_ = false;
    has_data_to_compress_ = false;
    compression_done_ = true;
  }
  output_view_ = output;

  // Copy the header at the beginning of the output
  if (write_header) {
    memcpy(output_view_.data(), header_.data(), header_.size());
    compressed_size_ += header_.size();
    output_view_.trim_front(header_.size());
  }
  const size_t kChunkSize = POINTS_PER_CHUNK * info_.point_step;

  buffer_.resize(kChunkSize);
  BufferView buffer_view(buffer_);

  size_t points_in_current_chunk = 0;
  size_t serialized_size = 0;

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
      for (auto& encoder : encoders_) {
        encoder->reset();
      }
      buffer_.resize(kChunkSize);
      buffer_view = BufferView(buffer_);
      points_in_current_chunk = 0;
      serialized_size = 0;
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
    min_encoded_point_bytes_ = 0;
    for (const auto& field : info.fields) {
      decoders_.push_back(std::make_unique<FieldDecoderCopy>(field.offset, field.type));
      min_encoded_point_bytes_ += SizeOf(field.type);
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

  // Compute once: minimum encoded bytes per point (sum of each decoder's minimum)
  min_encoded_point_bytes_ = 0;
  for (const auto& decoder : decoders_) {
    min_encoded_point_bytes_ += decoder->minInputBytes();
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
    while (!compressed_data.empty()) {
      uint32_t chunk_size = 0;
      Cloudini::decode(compressed_data, chunk_size);
      if (chunk_size > compressed_data.size()) {
        throw std::runtime_error("Invalid chunk size found while decoding");
      }
      ConstBufferView chunk_view(compressed_data.data(), chunk_size);
      decodeChunk(info, chunk_view, output);
      compressed_data.trim_front(chunk_size);
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
  for (auto& decoder : decoders_) {
    decoder->reset();
  }

  while (encoded_view.size() > 0) {
    if (encoded_view.size() < min_encoded_point_bytes_) {
      throw std::runtime_error("Truncated encoded data: not enough bytes for a complete point");
    }
    if (output_buffer.size() < info.point_step) {
      throw std::runtime_error("Output buffer is too small to hold the decoded data");
    }
    BufferView point_view(output_buffer.data(), info.point_step);
    for (auto& decoder : decoders_) {
      decoder->decode(encoded_view, point_view);
    }
    output_buffer.trim_front(info.point_step);
  }
}

}  // namespace Cloudini
