#include "cloudini/field_decoder.hpp"

namespace Cloudini {

void FieldDecoderFloat_Lossy::decode(ConstBufferView& input, BufferView& output) {
  int32_t diff = 0;
  auto offset = decodeVarint(input.data, diff);
  int32_t value = prev_value_ + diff;
  float value_real = static_cast<float>(value) * resolution_;
  prev_value_ = value;

  memcpy(output.data, &value_real, sizeof(float));
  input.advance(offset);
  output.advance(output_advance_);
}
//------------------------------------------------------------------------------------------

void FieldDecoderXYZ_Lossy::decode(ConstBufferView& input, BufferView& output) {
  Vector4i diff_vect(0, 0, 0, 0);
  auto ptr_out = input.data;
  ptr_out += decodeVarint(ptr_out, diff_vect[0]);
  ptr_out += decodeVarint(ptr_out, diff_vect[1]);
  ptr_out += decodeVarint(ptr_out, diff_vect[2]);

  Vector4i vect_int = diff_vect + prev_vect_;
  prev_vect_ = vect_int;

  Vector4f vect_real(vect_int[0], vect_int[1], vect_int[2], 0);
  vect_real = vect_real * multiplier_;

  memcpy(output.data, vect_real.data.u, 12);

  input.advance(ptr_out - input.data);
  output.advance(output_advance_);
}

//------------------------------------------------------------------------------------------

void FieldDecoderXYZI_Lossy::decode(ConstBufferView& input, BufferView& output) {
  Vector4i diff_vect;
  auto ptr_out = input.data;
  ptr_out += decodeVarint(ptr_out, diff_vect[0]);
  ptr_out += decodeVarint(ptr_out, diff_vect[1]);
  ptr_out += decodeVarint(ptr_out, diff_vect[2]);
  ptr_out += decodeVarint(ptr_out, diff_vect[3]);

  Vector4i vect_int = diff_vect + prev_vect_;
  prev_vect_ = vect_int;

  Vector4f vect_real(vect_int[0], vect_int[1], vect_int[2], vect_int[3]);
  vect_real = vect_real * multiplier_;

  memcpy(output.data, vect_real.data.u, 16);

  input.advance(ptr_out - input.data);
  output.advance(output_advance_);
}

//------------------------------------------------------------------------------------------

void FieldDecoderFloat_XOR::decode(ConstBufferView& input, BufferView& output) {
  input.advance(sizeof(uint32_t));
  output.advance(output_advance_);
}

}  // namespace Cloudini