/*
 * Copyright 2025 Davide Faconti
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cloudini_lib/field_decoder.hpp"

#include <cmath>
#include <limits>

namespace Cloudini {

FieldDecoderFloatN_Lossy::FieldDecoderFloatN_Lossy(const std::vector<FieldData>& field_data)
    : fields_count_(field_data.size()) {
  if (fields_count_ < 2) {
    throw std::runtime_error("FieldDecoderFloatN_Lossy requires at least 2 fields");
  }
  if (fields_count_ > 4) {
    throw std::runtime_error("FieldDecoderFloatN_Lossy can have at most 4 fields");
  }

  for (size_t i = 0; i < fields_count_; ++i) {
    multiplier_[i] = field_data[i].resolution;
    if (multiplier_[i] <= 0.0) {
      throw std::runtime_error("FieldDecoderFloatN_Lossy requires a resolution with value > 0.0");
    }
    offset_[i] = field_data[i].offset;
  }
  min_input_bytes_ = fields_count_;  // 1 byte per field minimum (NaN marker or smallest varint)
}

void FieldDecoderFloatN_Lossy::decode(ConstBufferView& input, BufferView dest_point_view) {
  if (input.empty()) {
    throw std::runtime_error("FieldDecoderFloatN_Lossy::decode: empty input buffer");
  }
  const uint8_t* ptr_in = input.data();
  const uint8_t* const ptr_end = input.data() + input.size();

  Vector4i new_vect{};
  Vector4f float_vect;

  // Decode deltas for each field
  for (size_t i = 0; i < fields_count_; ++i) {
    if (ptr_in >= ptr_end) {
      throw std::runtime_error("FieldDecoderFloatN_Lossy::decode: truncated input");
    }
    if (ptr_in[0] == 0) {
      // NaN case
      new_vect[i] = 0;
      float_vect[i] = std::numeric_limits<float>::quiet_NaN();
      ptr_in++;
    } else {
      // Normal case: decode varint delta
      int64_t diff = 0;
      const auto remaining = static_cast<size_t>((input.data() + input.size()) - ptr_in);
      const auto count = decodeVarint(ptr_in, remaining, diff);
      new_vect[i] = static_cast<int32_t>(diff) + prev_vect_[i];
      float_vect[i] = static_cast<float>(new_vect[i]) * multiplier_[i];
      ptr_in += count;
    }
  }

  prev_vect_ = new_vect;

  // Store results, handling NaN cases
  for (size_t i = 0; i < fields_count_; ++i) {
    if (offset_[i] != kDecodeButSkipStore) {
      memcpy(dest_point_view.data() + offset_[i], &float_vect[i], sizeof(float));
    }
  }

  // Update input buffer to point past consumed data
  const auto consumed = static_cast<size_t>(ptr_in - input.data());
  input.trim_front(consumed);
}

}  // namespace Cloudini
