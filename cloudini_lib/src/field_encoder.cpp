#include "cloudini/field_encoder.hpp"

#include <bit>

namespace Cloudini {

FieldEncoderFloatLossy::FieldEncoderFloatLossy(PointField field_info) : FieldEncoder(field_info) {
  static_assert(std::is_floating_point<float>::value, "FieldEncoderFloat requires a floating point type");
  if (field_info.type != FieldType::FLOAT32) {
    throw std::runtime_error("FieldEncoder(Float) requires a FLOAT32 type");
  }
  if (field_info.resolution.has_value() && field_info.resolution.value() > 0.0) {
    resolution_inv_ = 1.0 / static_cast<float>(field_info.resolution.value());
  } else {
    throw std::runtime_error("FieldEncoder(Float/Lossy) requires a resolution with value > 0.0");
  }
}

size_t FieldEncoderFloatLossy::encode(const ConstBufferView& input, BufferView& output) {
  float value_real = *(reinterpret_cast<const float*>(input.data));
  int64_t value = static_cast<int64_t>(value_real * resolution_inv_);

  int64_t delta = value - prev_value_;
  prev_value_ = value;
  auto offset = encodeVarint(delta, output.data);

  output.advance(offset);
  return offset;
}

//------------------------------------------------------------------------------------------

FieldEncoderFloatXOR::FieldEncoderFloatXOR(PointField field_info) : FieldEncoder(field_info) {
  static_assert(std::is_floating_point<float>::value, "FieldEncoderFloat requires a floating point type");
  if (field_info.type != FieldType::FLOAT32) {
    throw std::runtime_error("FieldEncoder(Float) requires a FLOAT32 type");
  }
}

size_t FieldEncoderFloatXOR::encode(const ConstBufferView& input, BufferView& output) {
  const float current_value = *(reinterpret_cast<const float*>(input.data));
  const uint32_t current_value_bits = std::bit_cast<uint32_t>(current_value);

  const uint32_t residual = current_value_bits ^ prev_value_bits_;
  prev_value_bits_ = current_value_bits;

  memcpy(output.data, &residual, sizeof(residual));
  output.advance(4);
  return 4;
}

}  // namespace Cloudini