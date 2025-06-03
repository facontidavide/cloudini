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

#include "cloudini_lib/field_encoder.hpp"

#include <bit>
#include <cmath>

namespace Cloudini {

FieldEncoderFloatN_Lossy::FieldEncoderFloatN_Lossy(const std::vector<FieldData>& field_data)
    : fields_count_(field_data.size()) {
  if (fields_count_ < 2) {
    throw std::runtime_error("FieldEncoderFloatN_Lossy requires at least one field");
  }
  if (fields_count_ > 4) {
    throw std::runtime_error("FieldEncoderFloatN_Lossy can have at most 4 fields");
  }

  for (size_t i = 0; i < fields_count_; ++i) {
    multiplier_[i] = 1.0F / (field_data[i].resolution);
    if (multiplier_[i] <= 0.0) {
      throw std::runtime_error("FieldEncoderFloatN_Lossy requires a resolution with value > 0.0");
    }
    offset_[i] = field_data[i].offset;
  }
}

size_t FieldEncoderFloatN_Lossy::encode(const ConstBufferView& point_view, BufferView& output) {
  const Vector4f vect_real(
      *(reinterpret_cast<const float*>(point_view.data() + offset_[0])),
      *(reinterpret_cast<const float*>(point_view.data() + offset_[1])),
      *(reinterpret_cast<const float*>(point_view.data() + offset_[2])),
      *(reinterpret_cast<const float*>(point_view.data() + offset_[3])));

  const Vector4f normalized_vect = vect_real * multiplier_;
  const Vector4i vect_int = cast_vector4f_to_vector4i(normalized_vect);
  const Vector4i delta = vect_int - prev_vect_;
  prev_vect_ = vect_int;

  auto* ptr_out = output.data();

  for (size_t i = 0; i < fields_count_; ++i) {
    if (std::isnan(vect_real[i])) {
      *ptr_out = 0;
      prev_vect_[i] = 0;
      ptr_out++;
    } else {
      ptr_out += encodeVarint64(delta[i], ptr_out);
    }
  }

  const auto count = static_cast<size_t>(ptr_out - output.data());
  output.trim_front(count);
  return count;
}

//------------------------------------------------------------------------------------------

}  // namespace Cloudini
