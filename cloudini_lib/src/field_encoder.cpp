#include "cloudini/field_encoder.hpp"

#include <bit>

namespace Cloudini {

size_t FieldEncoderFloatLossy::encode(ConstBufferView& input, BufferView& output) {
  float value_real = *(reinterpret_cast<const float*>(input.data));
  int64_t value = static_cast<int64_t>(value_real * resolution_inv_);

  int64_t delta = value - prev_value_;
  prev_value_ = value;
  auto offset = encodeVarint(delta, output.data);

  input.advance(input_advance_);
  output.advance(offset);
  return offset;
}

//------------------------------------------------------------------------------------------

size_t FieldEncoderFloatXOR::encode(ConstBufferView& input, BufferView& output) {
  const float current_value = *(reinterpret_cast<const float*>(input.data));
  const uint32_t current_value_bits = std::bit_cast<uint32_t>(current_value);

  const uint32_t residual = current_value_bits ^ prev_value_bits_;
  prev_value_bits_ = current_value_bits;

  memcpy(output.data, &residual, sizeof(residual));

  input.advance(input_advance_);
  output.advance(sizeof(residual));
  return sizeof(residual);
}

}  // namespace Cloudini