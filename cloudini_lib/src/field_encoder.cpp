#include "cloudini/field_encoder.hpp"

#include <bit>

namespace Cloudini {

size_t FieldEncoderFloat_Lossy::encode(ConstBufferView& input, BufferView& output) {
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

size_t FieldEncoderXYZ_Lossy::encode(ConstBufferView& input, BufferView& output) {
  Vector4f vect_real;
  memcpy(vect_real.data.u, input.data, 12);

  Vector4f normalized_vect = vect_real * multiplier_;

  Vector4l vect_int = cast_vector4f_to_Vector4l(normalized_vect);
  Vector4l delta = vect_int - prev_vect_;
  prev_vect_ = vect_int;

  auto ptr_out = output.data;
  ptr_out += encodeVarint(delta[0], ptr_out);
  ptr_out += encodeVarint(delta[1], ptr_out);
  ptr_out += encodeVarint(delta[2], ptr_out);

  input.advance(input_advance_);
  const size_t offset = ptr_out - output.data;
  output.advance(offset);
  return offset;
}

//------------------------------------------------------------------------------------------

size_t FieldEncoderXYZI_Lossy::encode(ConstBufferView& input, BufferView& output) {
  Vector4f vect_real;
  memcpy(vect_real.data.u, input.data, 16);

  Vector4l vect_int = cast_vector4f_to_Vector4l(vect_real * multiplier_);
  Vector4l delta = vect_int - prev_vect_;
  prev_vect_ = vect_int;

  auto ptr_out = output.data;
  ptr_out += encodeVarint(delta[0], ptr_out);
  ptr_out += encodeVarint(delta[1], ptr_out);
  ptr_out += encodeVarint(delta[2], ptr_out);
  ptr_out += encodeVarint(delta[3], ptr_out);

  input.advance(input_advance_);
  const size_t offset = ptr_out - output.data;
  output.advance(offset);
  return offset;
}

//------------------------------------------------------------------------------------------

size_t FieldEncoderFloat_XOR::encode(ConstBufferView& input, BufferView& output) {
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