#include "cloudini/field_decoder.hpp"

#include <bit>

namespace Cloudini {

FieldDecoderFloatLossy::FieldDecoderFloatLossy(PointField field_info) : FieldDecoder(field_info) {
  static_assert(std::is_floating_point<float>::value, "FieldDecoderFloat requires a floating point type");
  if (field_info.type != FieldType::FLOAT32) {
    throw std::runtime_error("FieldDecoder(Float) requires a FLOAT32 type");
  }
  if (field_info.resolution.has_value() && field_info.resolution.value() > 0.0) {
    resolution_inv_ = 1.0 / static_cast<float>(field_info.resolution.value());
  } else {
    throw std::runtime_error("FieldDecoder(Float/Lossy) requires a resolution with value > 0.0");
  }
}

void FieldDecoderFloatLossy::decode(ConstBufferView& input, BufferView& output) {
  float value = *(reinterpret_cast<const float*>(input.data));
  float diff = value - prev_value_;
  prev_value_ = value;
  auto offset = encodeVarint(static_cast<int64_t>(diff * resolution_inv_), output.data);

  input.advance(offset);
  output.advance(4);
}

//------------------------------------------------------------------------------------------

FieldDecoderFloatXOR::FieldDecoderFloatXOR(PointField field_info) : FieldDecoder(field_info) {
  static_assert(std::is_floating_point<float>::value, "FieldDecoderFloat requires a floating point type");
  if (field_info.type != FieldType::FLOAT32) {
    throw std::runtime_error("FieldDecoder(Float) requires a FLOAT32 type");
  }
}

void FieldDecoderFloatXOR::decode(ConstBufferView& input, BufferView& output) {
  const float current_value = *(reinterpret_cast<const float*>(input.data));
  const uint32_t current_value_bits = std::bit_cast<uint32_t>(current_value);

  const uint32_t residual = current_value_bits ^ prev_value_bits_;
  prev_value_bits_ = current_value_bits;

  memcpy(output.data, &residual, sizeof(residual));
  output.advance(4);
}

}  // namespace Cloudini