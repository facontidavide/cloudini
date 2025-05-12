#include "cloudini/cloudini.hpp"

#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

#include "cloudini/encoding_utils.hpp"
#include "cloudini/field_decoder.hpp"
#include "cloudini/field_encoder.hpp"
#include "lz4.h"
#include "zstd.h"

namespace Cloudini {

size_t ComputeHeaderSize(const std::vector<PointField>& fields) {
  size_t header_size = strlen(magic_header);
  header_size += sizeof(uint32_t);                   // width
  header_size += sizeof(uint32_t);                   // height
  header_size += sizeof(uint32_t);                   // point_step
  header_size += sizeof(uint64_t);                   // decoded size
  header_size += sizeof(uint8_t) + sizeof(uint8_t);  // first and second stage options
  header_size += sizeof(uint16_t);                   // fields count

  for (const auto& field : fields) {
    header_size += field.name.size() + sizeof(uint16_t);  // name
    header_size += sizeof(uint32_t);                      // offset
    header_size += sizeof(uint8_t);                       // type
    header_size += sizeof(double);                        // resolution
  }
  return header_size;
}

size_t EncodeHeader(const EncodingInfo& header, BufferView& output) {
  static uint8_t magic_length = strlen(magic_header);
  uint8_t* buff = output.data;

  const size_t prev_size = output.size;

  memcpy(buff, magic_header, magic_length);
  buff += magic_length;

  encode(header.width, output);
  encode(header.height, output);
  encode(header.point_step, output);

  encode(static_cast<uint8_t>(header.firts_stage), output);
  encode(static_cast<uint8_t>(header.second_stage), output);
  encode(header.decoded_size, output);

  encode(static_cast<uint16_t>(header.fields.size()), output);

  for (const auto& field : header.fields) {
    encode(field.name, output);
    encode(field.offset, output);
    encode(static_cast<uint8_t>(field.type), output);
    if (field.resolution) {
      encode(*field.resolution, output);
    } else {
      const double res = 0.0;
      encode(res, output);
    }
  }
  return output.size - prev_size;
}

EncodingInfo DecodeHeader(ConstBufferView& input) {
  EncodingInfo header;
  const uint8_t* buff = input.data;

  // check the magic header
  if (memcmp(buff, magic_header, strlen(magic_header)) != 0) {
    throw std::runtime_error("Invalid magic header");
  }
  buff += strlen(magic_header);

  decode(input, header.width);
  decode(input, header.height);
  decode(input, header.point_step);

  uint8_t stage;
  decode(input, stage);
  header.firts_stage = static_cast<FirstStageOpt>(stage);

  decode(input, stage);
  header.second_stage = static_cast<SecondStageOpt>(stage);

  decode(input, header.decoded_size);

  uint16_t fields_count = 0;
  decode(input, fields_count);

  for (int i = 0; i < fields_count; ++i) {
    PointField field;
    decode(input, field.name);
    decode(input, field.offset);
    uint8_t type = 0;
    decode(input, type);
    field.type = static_cast<FieldType>(type);
    double res = 0.0;
    decode(input, res);
    field.resolution = res;
    header.fields.push_back(std::move(field));
  }
  return header;
}

PointcloudEncoder::PointcloudEncoder(const EncodingInfo& info) : info_(info) {
  auto next_is_float = [this](size_t index) -> bool {
    return (index + 1) < info_.fields.size() && info_.fields[index + 1].type == FieldType::FLOAT32;
  };

  for (size_t i = 0; i < info_.fields.size(); ++i) {
    const auto& field = info_.fields[i];

    if (field.resolution && *field.resolution <= 0.0) {
      throw std::runtime_error("Field resolution must be greater than 0");
    }
    // special case: consecutive FLOAT32 fields
    if (field.type == FieldType::FLOAT32 && next_is_float(i)) {
      std::vector<FieldEncoderFloatN_Lossy::FieldData> field_data;
      field_data.emplace_back(field.offset, *field.resolution);
      while (next_is_float(i) && field_data.size() < 4) {
        ++i;
        field_data.emplace_back(info_.fields[i].offset, *info_.fields[i].resolution);
      }
      encoders_.push_back(std::make_unique<FieldEncoderFloatN_Lossy>(field_data));
      continue;
    }

    if (field.type == FieldType::FLOAT32) {
      encoders_.push_back(std::make_unique<FieldEncoderFloat_Lossy>(field.offset, *field.resolution));
    } else if (field.type == FieldType::INT16) {
      encoders_.push_back(std::make_unique<FieldEncoderInt<uint16_t>>(field.offset));
    } else if (field.type == FieldType::INT32) {
      encoders_.push_back(std::make_unique<FieldEncoderInt<uint32_t>>(field.offset));
    } else if (field.type == FieldType::UINT16) {
      encoders_.push_back(std::make_unique<FieldEncoderInt<uint16_t>>(field.offset));
    } else if (field.type == FieldType::UINT32) {
      encoders_.push_back(std::make_unique<FieldEncoderInt<uint32_t>>(field.offset));
    } else if (field.type == FieldType::INT8 || field.type == FieldType::UINT8) {
      encoders_.push_back(std::make_unique<FieldEncoderCopy>(field.offset, field.type));
    } else {
      throw std::runtime_error("Unsupported field type");
    }
  }

  header_.resize(ComputeHeaderSize(info_.fields));
  BufferView buffer_view(header_.data(), header_.size());
  EncodeHeader(info_, buffer_view);
}

size_t PointcloudEncoder::encode(ConstBufferView cloud_data, std::vector<uint8_t>& output) {
  buffer_.resize(cloud_data.size);
  output.resize(header_.size() + cloud_data.size);

  //----------------------------------------------
  // reset the state of the encoders
  for (auto& encoder : encoders_) {
    encoder->clear();
  }
  //----------------------------------------------
  // first stage compression. Result is stored in buffer_
  if (info_.firts_stage != FirstStageOpt::NONE) {
    BufferView buffer_view(buffer_.data(), buffer_.size());
    size_t serialized_size = 0;
    while (cloud_data.size > 0) {
      for (auto& encoder : encoders_) {
        serialized_size += encoder->encode(cloud_data, buffer_view);
      }
      cloud_data.advance(info_.point_step);
    }
    buffer_.resize(serialized_size);
  }

  //----------------------------------------------
  // copy the header at the beginning of the output
  memcpy(output.data(), header_.data(), header_.size());

  const bool has_fist_stage = info_.firts_stage != FirstStageOpt::NONE;
  const auto buffer_view = has_fist_stage ? ConstBufferView{buffer_.data(), buffer_.size()} : cloud_data;

  const char* src_ptr = reinterpret_cast<const char*>(buffer_view.data);
  const size_t src_size = buffer_view.size;
  char* dest_ptr = reinterpret_cast<char*>(output.data() + header_.size());
  const size_t dest_capacity = output.size() - header_.size();
  //----------------------------------------------
  // second stage compression. Data in _buffer will be compressed into output
  switch (info_.second_stage) {
    case SecondStageOpt::LZ4: {
      int compressed_size = LZ4_compress_default(src_ptr, dest_ptr, src_size, dest_capacity);
      if (compressed_size <= 0) {
        throw std::runtime_error("LZ4 compression failed");
      }
      output.resize(compressed_size + header_.size());
    } break;

    case SecondStageOpt::ZSTD: {
      size_t compressed_size = ZSTD_compress(dest_ptr, dest_capacity, src_ptr, src_size, 1);
      if (ZSTD_isError(compressed_size)) {
        throw std::runtime_error("ZSTD compression failed");
      }
      output.resize(compressed_size + header_.size());
    } break;

    case SecondStageOpt::NONE: {
      memcpy(dest_ptr, src_ptr, src_size);
      output.resize(src_size + header_.size());
    } break;
  }

  return output.size();
}

}  // namespace Cloudini