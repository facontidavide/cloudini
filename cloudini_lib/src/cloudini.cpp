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

constexpr static uint8_t kMagicLength = strlen(magic_header);

size_t ComputeHeaderSize(const std::vector<PointField>& fields) {
  size_t header_size = kMagicLength;
  header_size += sizeof(uint32_t);                   // width
  header_size += sizeof(uint32_t);                   // height
  header_size += sizeof(uint32_t);                   // point_step
  header_size += sizeof(uint8_t) + sizeof(uint8_t);  // first and second stage options
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
  const size_t prev_size = output.size;

  memcpy(output.data, magic_header, kMagicLength);
  output.advance(kMagicLength);

  encode(header.width, output);
  encode(header.height, output);
  encode(header.point_step, output);

  encode(static_cast<uint8_t>(header.firts_stage), output);
  encode(static_cast<uint8_t>(header.second_stage), output);
  encode(static_cast<uint16_t>(header.fields.size()), output);

  for (const auto& field : header.fields) {
    encode(field.name, output);
    encode(field.offset, output);
    encode(static_cast<uint8_t>(field.type), output);
    if (field.resolution) {
      encode(*field.resolution, output);
    } else {
      const float res = 0.0;
      encode(res, output);
    }
  }
  return prev_size - output.size;
}

EncodingInfo DecodeHeader(ConstBufferView& input) {
  EncodingInfo header;
  const uint8_t* buff = input.data;

  // check the magic header
  if (memcmp(buff, magic_header, kMagicLength) != 0) {
    std::string fist_bytes = std::string(reinterpret_cast<const char*>(buff), kMagicLength);
    throw std::runtime_error(std::string("Invalid magic header. got: ") + fist_bytes);
  }
  input.advance(kMagicLength);

  decode(input, header.width);
  decode(input, header.height);
  decode(input, header.point_step);

  uint8_t stage;
  decode(input, stage);
  header.firts_stage = static_cast<FirstStageOpt>(stage);

  decode(input, stage);
  header.second_stage = static_cast<SecondStageOpt>(stage);

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
    field.resolution = res;
    header.fields.push_back(std::move(field));
  }
  return header;
}

PointcloudEncoder::PointcloudEncoder(const EncodingInfo& info) : info_(info) {
  for (size_t index = 0; index < info_.fields.size(); ++index) {
    auto resolution = info_.fields[index].resolution.value_or(1.0f);
    auto offset = info_.fields[index].offset;
    auto field_type = info_.fields[index].type;

    // special case: consecutive FLOAT32 fields
    auto next_is_float = [this](size_t i) -> bool {
      return (i + 1) < info_.fields.size() && info_.fields[i + 1].type == FieldType::FLOAT32;
    };

    if (field_type == FieldType::FLOAT32 && next_is_float(index)) {
      std::vector<FieldEncoderFloatN_Lossy::FieldData> field_data;

      while (field_data.size() < 4) {
        field_data.emplace_back(offset, resolution);
        if (!next_is_float(index)) {
          break;
        }
        index++;
        resolution = info_.fields[index].resolution.value_or(1.0f);
        offset = info_.fields[index].offset;
      }
      encoders_.push_back(std::make_unique<FieldEncoderFloatN_Lossy>(field_data));
      continue;
    }

    switch (field_type) {
      case FieldType::FLOAT32:
        encoders_.push_back(std::make_unique<FieldEncoderFloat_Lossy>(offset, resolution));
        break;
      case FieldType::INT16:
        encoders_.push_back(std::make_unique<FieldEncoderInt<uint16_t>>(offset));
        break;
      case FieldType::INT32:
        encoders_.push_back(std::make_unique<FieldEncoderInt<uint32_t>>(offset));
        break;
      case FieldType::UINT16:
        encoders_.push_back(std::make_unique<FieldEncoderInt<uint16_t>>(offset));
        break;
      case FieldType::UINT32:
        encoders_.push_back(std::make_unique<FieldEncoderInt<uint32_t>>(offset));
        break;
      case FieldType::INT8:
      case FieldType::UINT8:
        encoders_.push_back(std::make_unique<FieldEncoderCopy>(offset, field_type));
        break;
      default:
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
    encoder->reset();
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

//------------------------------------------------------------------------------------------

void PointcloudDecoder::updateDecoders(const EncodingInfo& info) {
  auto create_decoder = [this](const PointField& field) -> std::unique_ptr<FieldDecoder> {
    if (field.type == FieldType::FLOAT32 && field.resolution) {
      return (std::make_unique<FieldDecoderFloat_Lossy>(field.offset, *field.resolution));
    } else if (field.type == FieldType::INT16) {
      return (std::make_unique<FieldDecoderInt<uint16_t>>(field.offset));
    } else if (field.type == FieldType::INT32) {
      return (std::make_unique<FieldDecoderInt<uint32_t>>(field.offset));
    } else if (field.type == FieldType::UINT16) {
      return (std::make_unique<FieldDecoderInt<uint16_t>>(field.offset));
    } else if (field.type == FieldType::UINT32) {
      return (std::make_unique<FieldDecoderInt<uint32_t>>(field.offset));
    } else if (field.type == FieldType::INT8 || field.type == FieldType::UINT8) {
      return (std::make_unique<FieldDecoderCopy>(field.offset, field.type));
    } else {
      throw std::runtime_error("Unsupported field type");
    }
  };

  decoders_.resize(info.fields.size());
  for (size_t i = 0; i < info.fields.size(); ++i) {
    decoders_[i] = create_decoder(info.fields[i]);
  }
}

void PointcloudDecoder::decode(const EncodingInfo& info, ConstBufferView compressed_data, BufferView output) {
  // read the header
  updateDecoders(info);

  // check if the first bytes are the magic header. if they are, skip them
  if (memcmp(compressed_data.data, magic_header, kMagicLength) == 0) {
    throw std::runtime_error("compressed_data contains the header. You should use DecodeHeader first");
  }

  // allocate sufficient space in the buffer
  buffer_.resize(info.width * info.height * info.point_step);

  //----------------------------------------------------------------------
  // start decompressing using "second_stage" param.
  // Note that compressed_data doesn't contan the header anymore.
  // Decompressed data will be stored in buffer_
  switch (info.second_stage) {
    case SecondStageOpt::LZ4: {
      const auto* src_ptr = reinterpret_cast<const char*>(compressed_data.data);
      auto* buffer_ptr = reinterpret_cast<char*>(buffer_.data());
      const int decompressed_size = LZ4_decompress_safe(src_ptr, buffer_ptr, compressed_data.size, buffer_.size());
      if (decompressed_size < 0) {
        throw std::runtime_error("LZ4 decompression failed");
      }
      buffer_.resize(decompressed_size);
    } break;

    case SecondStageOpt::ZSTD: {
      const size_t decompressed_size =
          ZSTD_decompress(buffer_.data(), buffer_.size(), compressed_data.data, compressed_data.size);
      if (ZSTD_isError(decompressed_size)) {
        throw std::runtime_error("ZSTD decompression failed");
      }
      buffer_.resize(decompressed_size);
    } break;

    case SecondStageOpt::NONE: {
      // TODO: fixme, no need to copy
      memcpy(buffer_.data(), compressed_data.data, compressed_data.size);
    } break;
  }

  //----------------------------------------------------------------------
  // decode the data (first stage)
  ConstBufferView buffer_view(buffer_.data(), buffer_.size());
  for (size_t i = 0; i < info.width * info.height; ++i) {
    for (auto& decoder : decoders_) {
      decoder->decode(buffer_view, output);
    }
    output.advance(info.point_step);
  }
}

}  // namespace Cloudini