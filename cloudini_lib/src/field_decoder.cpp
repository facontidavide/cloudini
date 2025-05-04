#include "cloudini/field_decoder.hpp"

namespace Cloudini {

void FieldDecoderFloatLossy::decode(ConstBufferView& input, BufferView& output) {
  int64_t diff = 0;
  auto offset = decodeVarint(input.data, diff);
  int64_t value = prev_value_ + diff;
  float value_real = static_cast<float>(value) * resolution_;
  prev_value_ = value;

  memcpy(output.data, &value_real, sizeof(float));
  input.advance(offset);
  output.advance(output_advance_);
}

//------------------------------------------------------------------------------------------

void FieldDecoderFloatXOR::decode(ConstBufferView& input, BufferView& output) {
  input.advance(sizeof(uint32_t));
  output.advance(output_advance_);
}

}  // namespace Cloudini