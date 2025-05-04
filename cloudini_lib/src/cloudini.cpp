#include "cloudini/cloudini.hpp"

#include <stdexcept>

#include "cloudini/encoding_utils.hpp"
#include "lz4.h"

namespace Cloudini {

size_t ComputeHeaderSize(const std::vector<PointField>& fields) {
  size_t header_size = strlen(magic_header);
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

void EncodeHeader(const EncodingInfo& header, BufferView& output) {
  static uint8_t magic_length = strlen(magic_header);
  uint8_t* buff = output.data;

  memcpy(buff, magic_header, magic_length);
  buff += magic_length;

  encode(header.decoded_size, output);

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
      const double res = 0.0;
      encode(res, output);
    }
  }
}

EncodingInfo DecodeHeader(BufferView& input) {
  EncodingInfo header;
  uint8_t* buff = input.data;

  // check the magic header
  if (memcmp(buff, magic_header, strlen(magic_header)) != 0) {
    throw std::runtime_error("Invalid magic header");
  }
  buff += strlen(magic_header);

  decode(input, header.decoded_size);

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
    double res = 0.0;
    decode(input, res);
    field.resolution = res;
    header.fields.push_back(std::move(field));
  }
  return header;
}

}  // namespace Cloudini