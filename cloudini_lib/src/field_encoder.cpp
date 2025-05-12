#include "cloudini/field_encoder.hpp"

#include <bit>

namespace Cloudini {

size_t FieldEncoderFloat_Lossy::encode(const ConstBufferView& point_view, BufferView& output) {
  float value_real = *(reinterpret_cast<const float*>(point_view.data + field_offset_));
  int64_t value = static_cast<int64_t>(value_real * resolution_inv_);

  int64_t delta = value - prev_value_;
  prev_value_ = value;
  auto offset = encodeVarint(delta, output.data);

  output.advance(offset);
  return offset;
}

//------------------------------------------------------------------------------------------

size_t FieldEncoderFloat_XOR::encode(const ConstBufferView& point_view, BufferView& output) {
  const float current_value = *(reinterpret_cast<const float*>(point_view.data + field_offset_));
  const uint32_t current_value_bits = std::bit_cast<uint32_t>(current_value);

  const uint32_t residual = current_value_bits ^ prev_value_bits_;
  prev_value_bits_ = current_value_bits;

  memcpy(output.data, &residual, sizeof(residual));

  output.advance(sizeof(residual));
  return sizeof(residual);
}

//------------------------------------------------------------------------------------------

size_t FieldEncoderFloatN_Lossy::encode(const ConstBufferView& point_view, BufferView& output) {
  const Vector4f vect_real(
      *(reinterpret_cast<const float*>(point_view.data + offset_[0])),
      *(reinterpret_cast<const float*>(point_view.data + offset_[1])),
      *(reinterpret_cast<const float*>(point_view.data + offset_[2])),
      *(reinterpret_cast<const float*>(point_view.data + offset_[3])));

  const Vector4f normalized_vect = vect_real * multiplier_;
  const Vector4i vect_int = cast_vector4f_to_Vector4i(normalized_vect);
  const Vector4i delta = vect_int - prev_vect_;
  prev_vect_ = vect_int;

  auto ptr_out = output.data;
  for (int i = 0; i < 4; ++i) {
    ptr_out += encodeVarint(delta[i], ptr_out);
  }

  const size_t offset = ptr_out - output.data;
  output.advance(offset);
  return offset;
}

//------------------------------------------------------------------------------------------

}  // namespace Cloudini