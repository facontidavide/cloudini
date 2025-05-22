#include "cloudini_lib/field_decoder.hpp"

namespace Cloudini {

void FieldDecoderFloat_Lossy::decode(ConstBufferView& input, BufferView dest_point_view) {
  if (input.data[0] == 0) {
    constexpr float nan_value = std::numeric_limits<float>::quiet_NaN();
    memcpy(dest_point_view.data + offset_, &nan_value, sizeof(float));
    input.advance(1);
    reset();
    return;
  }

  int64_t diff = 0;
  const auto count = decodeVarint(input.data, diff);
  const int64_t value = prev_value_ + diff;
  const float value_real = static_cast<float>(value) * multiplier_;
  prev_value_ = value;

  memcpy(dest_point_view.data + offset_, &value_real, sizeof(float));
  input.advance(count);
}

}  // namespace Cloudini