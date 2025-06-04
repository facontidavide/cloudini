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

#pragma once

#include <array>
#include <stdexcept>

#include "cloudini_lib/encoding_utils.hpp"
#include "cloudini_lib/intrinsics.hpp"

namespace Cloudini {

class FieldDecoder {
 public:
  FieldDecoder() = default;

  virtual ~FieldDecoder() = default;

  /**
   * @brief Decode the field data from the input buffer to the output buffer.
   * The input buffer will be advanced, while the dest_point_view must be advanced by the user.BufferView
   *
   * @param input The input buffer containing the pointer to the current point. Advanced automatically.
   * @param dest_point_view The output buffer to write the decoded data. Must point at the point.
   */
  virtual void decode(ConstBufferView& input, BufferView dest_point_view) = 0;

  virtual void reset() = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for copying the field data
class FieldDecoderCopy : public FieldDecoder {
 public:
  FieldDecoderCopy(size_t field_offset, FieldType field_type)
      : offset_(field_offset), field_size_(SizeOf(field_type)) {}

  void decode(ConstBufferView& input, BufferView dest_point_view) override {
    if (offset_ != kDecodeButSkipStore) {
      memcpy(dest_point_view.data() + offset_, input.data(), field_size_);
    }
    input.trim_front(field_size_);
  }

  void reset() override {}

 private:
  size_t offset_ = 0;
  size_t field_size_ = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for all the integer types
template <typename IntType>
class FieldDecoderInt : public FieldDecoder {
 public:
  FieldDecoderInt(size_t field_offset) : offset_(field_offset) {
    static_assert(std::is_integral<IntType>::value, "FieldDecoderInt requires an integral type");
  }

  void decode(ConstBufferView& input, BufferView dest_point_view) override {
    int64_t diff = 0;
    auto count = decodeVarint(input.data(), diff);

    int64_t value = prev_value_ + diff;
    prev_value_ = value;
    if (offset_ != kDecodeButSkipStore) {
      memcpy(dest_point_view.data() + offset_, &value, sizeof(IntType));
    }
    input.trim_front(count);
  }

  void reset() override {
    prev_value_ = 0;
  }

 private:
  int64_t prev_value_ = 0;
  size_t offset_;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossy compression
template <typename FloatType>
class FieldDecoderFloat_Lossy : public FieldDecoder {
 public:
  FieldDecoderFloat_Lossy(size_t field_offset, FloatType resolution) : offset_(field_offset), multiplier_(resolution) {
    if (resolution <= 0.0) {
      throw std::runtime_error("FieldDecoder(Float/Lossy) requires a resolution with value > 0.0");
    }
  }

  void decode(ConstBufferView& input, BufferView dest_point_view) override;

  void reset() override {
    prev_value_ = 0;
  }

 private:
  size_t offset_ = 0;
  FloatType multiplier_ = 0.0;
  int64_t prev_value_ = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossless compression
class FieldDecoderFloat_XOR : public FieldDecoder {
 public:
  FieldDecoderFloat_XOR(size_t field_offset) : offset_(field_offset) {}

  void decode(ConstBufferView& input, BufferView dest_point_view) override;

  void reset() override {
    prev_value_bits_ = 0;
  }

 private:
  size_t offset_ = 0;
  uint32_t prev_value_bits_ = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for multiple consecutive floats
class FieldDecoderFloatN_Lossy : public FieldDecoder {
 public:
  struct FieldData {
    size_t offset;
    float resolution;
  };

  FieldDecoderFloatN_Lossy(const std::vector<FieldData>& field_data);

  void decode(ConstBufferView& input, BufferView dest_point_view) override;

  void reset() override {
    prev_vect_ = Vector4i(0, 0, 0, 0);
    multiplier_ = Vector4f(0, 0, 0, 0);
  }

 private:
  std::array<size_t, 4> offset_ = {0, 0, 0, 0};
  size_t fields_count_ = 0;

  Vector4i prev_vect_ = Vector4i(0, 0, 0, 0);
  Vector4f multiplier_ = Vector4f(0, 0, 0, 0);
};

//------------------------------------------------------------------------------------------
template <typename FloatType>
inline void FieldDecoderFloat_Lossy<FloatType>::decode(ConstBufferView& input, BufferView dest_point_view) {
  if (input.data()[0] == 0) {
    constexpr auto nan_value = std::numeric_limits<FloatType>::quiet_NaN();
    if (offset_ != kDecodeButSkipStore) {
      memcpy(dest_point_view.data() + offset_, &nan_value, sizeof(FloatType));
    }
    input.trim_front(1);
    reset();
    return;
  }

  int64_t diff = 0;
  const auto count = decodeVarint(input.data(), diff);
  const int64_t value = prev_value_ + diff;
  const FloatType value_real = static_cast<FloatType>(value) * multiplier_;
  prev_value_ = value;

  if (offset_ != kDecodeButSkipStore) {
    memcpy(dest_point_view.data() + offset_, &value_real, sizeof(value_real));
  }
  input.trim_front(count);
}

}  // namespace Cloudini
