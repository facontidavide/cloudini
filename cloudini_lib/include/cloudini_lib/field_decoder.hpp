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

  /// Minimum number of input bytes this decoder needs per call.
  /// Used by PointcloudDecoder for a single per-point bounds check.
  size_t minInputBytes() const {
    return min_input_bytes_;
  }

 protected:
  size_t min_input_bytes_ = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for copying the field data
class FieldDecoderCopy : public FieldDecoder {
 public:
  FieldDecoderCopy(size_t field_offset, FieldType field_type) : offset_(field_offset), field_size_(SizeOf(field_type)) {
    min_input_bytes_ = field_size_;
  }

  void decode(ConstBufferView& input, BufferView dest_point_view) override {
    // Bounds validated by PointcloudDecoder's per-point check + trim_front's own check
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
    min_input_bytes_ = 1;  // smallest varint is 1 byte
  }

  void decode(ConstBufferView& input, BufferView dest_point_view) override {
    int64_t diff = 0;
    auto count = decodeVarint(input.data(), input.size(), diff);

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
    min_input_bytes_ = 1;  // NaN marker or smallest varint
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
template <typename FloatType>
class FieldDecoderFloat_XOR : public FieldDecoder {
 public:
  FieldDecoderFloat_XOR(size_t field_offset) : offset_(field_offset) {
    static_assert(std::is_floating_point<FloatType>::value, "FieldDecoderFloat_XOR requires a floating point type");
    min_input_bytes_ = sizeof(IntType);
  }

  void decode(ConstBufferView& input, BufferView dest_point_view) override;

  void reset() override {
    prev_bits_ = 0;
  }

 private:
  using IntType = std::conditional_t<std::is_same<FloatType, float>::value, uint32_t, uint64_t>;
  size_t offset_;
  IntType prev_bits_ = 0;
};

//------------------------------------------------------------------------------------------
// Gorilla/Chimp-style bit-packed XOR decoder. Mirrors FieldEncoderFloat_Gorilla.
// Reads bits LSB-first within each byte; bytes are pulled from `input` on demand.
template <typename FloatType>
class FieldDecoderFloat_Gorilla : public FieldDecoder {
 public:
  FieldDecoderFloat_Gorilla(size_t field_offset) : offset_(field_offset) {
    static_assert(std::is_floating_point<FloatType>::value, "FieldDecoderFloat_Gorilla requires a floating point type");
    // Bit-packed: bytes are consumed nondeterministically across calls, so the per-point
    // minInputBytes() check cannot be meaningful. Returning 0 disables that check for this
    // field; getBits() throws on actual truncation.
    min_input_bytes_ = 0;
  }

  void decode(ConstBufferView& input, BufferView dest_point_view) override;

  void reset() override {
    prev_bits_ = 0;
    prev_leading_ = kLeadingSentinel;
    prev_trailing_ = 0;
    bit_buf_ = 0;
    bit_count_ = 0;
    first_ = true;
  }

 private:
  using IntType = std::conditional_t<std::is_same<FloatType, float>::value, uint32_t, uint64_t>;
  static constexpr size_t kTypeBits = sizeof(IntType) * 8;
  static constexpr uint8_t kLeadingSentinel = 255;

  inline uint64_t getBits(uint8_t nbits, ConstBufferView& input);

  size_t offset_;
  IntType prev_bits_ = 0;
  uint8_t prev_leading_ = kLeadingSentinel;
  uint8_t prev_trailing_ = 0;

  uint64_t bit_buf_ = 0;
  uint8_t bit_count_ = 0;
  bool first_ = true;
};

template <typename FloatType>
inline uint64_t FieldDecoderFloat_Gorilla<FloatType>::getBits(uint8_t nbits, ConstBufferView& input) {
  // Split into two halves to avoid shifting a full byte into a near-full bit buffer
  // (which would drop high-order bits). When we'd refill with a byte but bit_count_ > 56,
  // we must first extract bits already in the buffer.
  auto fillOne = [&]() {
    if (input.size() == 0) {
      throw std::runtime_error("FieldDecoderFloat_Gorilla: truncated input");
    }
    const uint64_t byte = static_cast<uint64_t>(input.data()[0]);
    input.trim_front(1);
    bit_buf_ |= (byte << bit_count_);
    bit_count_ += 8;
  };

  // If we'd need more than 56 bits combined (bit_count_ + future fill) and nbits is large,
  // process in two phases: first consume what's in the buffer, then refill.
  if (nbits > 56 || bit_count_ > 56) {
    // Phase 1: extract up to min(nbits, bit_count_) from current bit_buf_ without adding more.
    const uint8_t phase1_bits = std::min<uint8_t>(nbits, bit_count_);
    const uint64_t mask1 = (phase1_bits < 64) ? ((uint64_t{1} << phase1_bits) - 1) : ~uint64_t{0};
    const uint64_t low = bit_buf_ & mask1;
    if (phase1_bits < 64) {
      bit_buf_ >>= phase1_bits;
    } else {
      bit_buf_ = 0;
    }
    bit_count_ -= phase1_bits;

    const uint8_t remaining = nbits - phase1_bits;
    if (remaining == 0) {
      return low;
    }
    // Now bit_count_ is small; safe to refill.
    while (bit_count_ < remaining) {
      fillOne();
    }
    const uint64_t mask2 = (remaining < 64) ? ((uint64_t{1} << remaining) - 1) : ~uint64_t{0};
    const uint64_t high = bit_buf_ & mask2;
    if (remaining < 64) {
      bit_buf_ >>= remaining;
    } else {
      bit_buf_ = 0;
    }
    bit_count_ -= remaining;
    return low | (high << phase1_bits);
  }

  // Fast path: small nbits and small bit_count_ — can't overflow even if we add 8 more.
  while (bit_count_ < nbits) {
    fillOne();
  }
  const uint64_t mask = (nbits < 64) ? ((uint64_t{1} << nbits) - 1) : ~uint64_t{0};
  const uint64_t result = bit_buf_ & mask;
  bit_buf_ >>= nbits;
  bit_count_ -= nbits;
  return result;
}

template <typename FloatType>
inline void FieldDecoderFloat_Gorilla<FloatType>::decode(ConstBufferView& input, BufferView dest_point_view) {
  IntType value_bits;

  if (first_) {
    first_ = false;
    const uint64_t raw = getBits(static_cast<uint8_t>(kTypeBits), input);
    value_bits = static_cast<IntType>(raw);
    prev_bits_ = value_bits;
  } else {
    const uint64_t flag = getBits(1, input);
    if (flag == 0) {
      // Same value as previous.
      value_bits = prev_bits_;
    } else {
      const uint64_t control = getBits(1, input);
      IntType xor_val;
      if (control == 0) {
        // Reuse previous window.
        const uint8_t meaningful = static_cast<uint8_t>(kTypeBits - prev_leading_ - prev_trailing_);
        const uint64_t bits = getBits(meaningful, input);
        xor_val = static_cast<IntType>(bits << prev_trailing_);
      } else {
        // New window: read leading(5), meaningful-1(6), meaningful bits.
        const uint8_t stored_leading = static_cast<uint8_t>(getBits(5, input));
        const uint8_t meaningful = static_cast<uint8_t>(getBits(6, input) + 1);
        const uint64_t bits = getBits(meaningful, input);
        const uint8_t trailing = static_cast<uint8_t>(kTypeBits - stored_leading - meaningful);
        xor_val = static_cast<IntType>(bits << trailing);
        prev_leading_ = stored_leading;
        prev_trailing_ = trailing;
      }
      value_bits = static_cast<IntType>(xor_val ^ prev_bits_);
      prev_bits_ = value_bits;
    }
  }

  if (offset_ != kDecodeButSkipStore) {
    memcpy(dest_point_view.data() + offset_, &value_bits, sizeof(FloatType));
  }

  // Discard any padding bits remaining in the bit buffer so the next decode() starts on
  // a byte boundary. Matches the per-call byte-alignment in FieldEncoderFloat_Gorilla.
  bit_buf_ = 0;
  bit_count_ = 0;
}

//------------------------------------------------------------------------------------------
// Specialization for multiple consecutive floats
class FieldDecoderFloatN_Lossy : public FieldDecoder {
 public:
  struct FieldData {
    size_t offset = 0;
    float resolution = 0.001f;
    FieldData() = default;
    FieldData(size_t o, float r) : offset(o), resolution(r) {}
  };

  FieldDecoderFloatN_Lossy(const std::vector<FieldData>& field_data);

  void decode(ConstBufferView& input, BufferView dest_point_view) override;

  void reset() override {
    prev_vect_ = Vector4i(0, 0, 0, 0);
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
  // Bounds validated by PointcloudDecoder's per-point min_encoded_point_bytes_ check
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
  const auto count = decodeVarint(input.data(), input.size(), diff);
  const int64_t value = prev_value_ + diff;
  const FloatType value_real = static_cast<FloatType>(value) * multiplier_;
  prev_value_ = value;

  if (offset_ != kDecodeButSkipStore) {
    memcpy(dest_point_view.data() + offset_, &value_real, sizeof(value_real));
  }
  input.trim_front(count);
}

template <typename FloatType>
inline void FieldDecoderFloat_XOR<FloatType>::decode(ConstBufferView& input, BufferView dest_point_view) {
  // Bounds validated by PointcloudDecoder's per-point check + trim_front's own check
  IntType residual = 0;
  memcpy(&residual, input.data(), sizeof(IntType));
  input.trim_front(sizeof(IntType));

  // XOR the residual with the previous bits to recover the current value
  const IntType current_bits = residual ^ prev_bits_;
  prev_bits_ = current_bits;

  // Convert back to float and store in destination
  if (offset_ != kDecodeButSkipStore) {
    memcpy(dest_point_view.data() + offset_, &current_bits, sizeof(FloatType));
  }
}
}  // namespace Cloudini
