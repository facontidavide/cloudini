#include "cloudini/field_encoder.hpp"

#include <bit>
#include <cmath>

namespace Cloudini {

size_t FieldEncoderFloat_Lossy::encode(const ConstBufferView& point_view, BufferView& output) {
  float value_real = *(reinterpret_cast<const float*>(point_view.data + offset_));

  if (std::isnan(value_real)) {
    output.data[0] = 0;  // value 0 is reserved for NaN
    prev_value_ = 0;
    output.advance(1);
    return 1;
  }

  const auto value = static_cast<int32_t>(value_real * resolution_inv_);
  const auto delta = value - prev_value_;
  prev_value_ = value;
  auto offset = encodeVarint(delta, output.data);

  output.advance(offset);
  return offset;
}

//------------------------------------------------------------------------------------------

size_t FieldEncoderFloat_XOR::encode(const ConstBufferView& point_view, BufferView& output) {
  const uint32_t current_value_bits = *(reinterpret_cast<const uint32_t*>(point_view.data + offset_));
  const uint32_t residual = current_value_bits ^ prev_value_bits_;
  prev_value_bits_ = current_value_bits;
  memcpy(output.data, &residual, sizeof(uint32_t));
  output.advance(sizeof(uint32_t));
  return sizeof(uint32_t);
}

//------------------------------------------------------------------------------------------

FieldEncoderFloatN_Lossy::FieldEncoderFloatN_Lossy(const std::vector<FieldData>& field_data)
    : fields_count_(field_data.size()) {
  if (fields_count_ < 2) {
    throw std::runtime_error("FieldEncoderFloatN_Lossy requires at least one field");
  }
  if (fields_count_ > 4) {
    throw std::runtime_error("FieldEncoderFloatN_Lossy can have at most 4 fields");
  }

  for (size_t i = 0; i < fields_count_; ++i) {
    multiplier_[i] = 1.0F / field_data[i].resolution;
    if (multiplier_[i] <= 0.0) {
      throw std::runtime_error("FieldEncoderFloatN_Lossy requires a resolution with value > 0.0");
    }
    offset_[i] = field_data[i].offset;
  }
}

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
  for (size_t i = 0; i < fields_count_; ++i) {
    if (std::isnan(vect_real[i])) {
      *ptr_out = 0;
      prev_vect_[i] = 0;
      ptr_out++;
    } else {
      ptr_out += encodeVarint(delta[i], ptr_out);
    }
  }

  const auto count = static_cast<size_t>(ptr_out - output.data);
  output.advance(count);
  return count;
}

//------------------------------------------------------------------------------------------

}  // namespace Cloudini