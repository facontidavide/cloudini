#include "cloudini/field_decoder.hpp"

namespace Cloudini {

FieldDecoderFloatLossy::FieldDecoderFloatLossy(PointField field_info) : FieldDecoder(field_info) {
  static_assert(std::is_floating_point<float>::value, "FieldDecoderFloat requires a floating point type");
  if (field_info.type != FieldType::FLOAT32) {
    throw std::runtime_error("FieldDecoder(Float) requires a FLOAT32 type");
  }
  if (field_info.resolution.has_value() && field_info.resolution.value() > 0.0) {
    resolution_ = static_cast<float>(field_info.resolution.value());
  } else {
    throw std::runtime_error("FieldDecoder(Float/Lossy) requires a resolution with value > 0.0");
  }
}

void FieldDecoderFloatLossy::decode(ConstBufferView& input, BufferView& output) {
  int64_t diff = 0;
  auto offset = decodeVarint(input.data, diff);
  int64_t value = prev_value_ + diff;
  float value_real = static_cast<float>(value) * resolution_;
  prev_value_ = value;

  memcpy(output.data, &value_real, sizeof(float));
  input.advance(offset);
  output.advance(sizeof(float));
}

//------------------------------------------------------------------------------------------

FieldDecoderFloatXOR::FieldDecoderFloatXOR(PointField field_info) : FieldDecoder(field_info) {
  static_assert(std::is_floating_point<float>::value, "FieldDecoderFloat requires a floating point type");
  if (field_info.type != FieldType::FLOAT32) {
    throw std::runtime_error("FieldDecoder(Float) requires a FLOAT32 type");
  }
}

void FieldDecoderFloatXOR::decode(ConstBufferView& input, BufferView& output) {
  // TODO
}

}  // namespace Cloudini