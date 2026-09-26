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

void FieldDecoderFloatN_Lossy::decodePoints(ConstBufferView& input, uint8_t* output, size_t point_step, size_t count) {
  switch (fields_count_) {
    case 2:
      decodePointsImpl<2>(input, output, point_step, count);
      break;
    case 3:
      decodePointsImpl<3>(input, output, point_step, count);
      break;
    default:
      decodePointsImpl<4>(input, output, point_step, count);
      break;
  }
}

template <size_t N>
void FieldDecoderFloatN_Lossy::decodePointsImpl(
    ConstBufferView& input, uint8_t* output, size_t point_step, size_t count) {
  // A point takes at most N * kMaxVarintBytes bytes: while that many are left, its varints are read
  // without per-byte bounds checks. The last few points go through the checked decode().
  constexpr size_t kMaxPointBytes = N * kMaxVarintBytes;
  const uint8_t* ptr = input.data();
  const uint8_t* const end = input.data() + input.size();

  int32_t prev[N];
  float multiplier[N];
  size_t offset[N];
  for (size_t k = 0; k < N; ++k) {
    prev[k] = prev_vect_[k];
    multiplier[k] = multiplier_[k];
    offset[k] = offset_[k];
  }

  size_t i = 0;
  for (; i < count && static_cast<size_t>(end - ptr) >= kMaxPointBytes; ++i) {
    uint8_t* point = output + i * point_step;
    for (size_t k = 0; k < N; ++k) {
      float value;
      if (*ptr == 0) {
        // NaN marker
        ++ptr;
        prev[k] = 0;
        value = std::numeric_limits<float>::quiet_NaN();
      } else {
        int64_t diff = 0;
        ptr += decodeVarintUnchecked(ptr, diff);
        // same wrap-around as decode(): int32 addition of the truncated delta
        prev[k] = static_cast<int32_t>(static_cast<uint32_t>(prev[k]) + static_cast<uint32_t>(diff));
        value = static_cast<float>(prev[k]) * multiplier[k];
      }
      if (offset[k] != kDecodeButSkipStore) {
        memcpy(point + offset[k], &value, sizeof(float));
      }
    }
  }

  for (size_t k = 0; k < N; ++k) {
    prev_vect_[k] = prev[k];
  }
  input.trim_front(static_cast<size_t>(ptr - input.data()));

  // the last few points, through the checked per-point decode()
  FieldDecoder::decodePoints(input, output + i * point_step, point_step, count - i);
}

}  // namespace Cloudini
