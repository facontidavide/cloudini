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

class FieldEncoder {
 public:
  FieldEncoder() = default;

  virtual ~FieldEncoder() = default;

  /**
   * @brief Encode the field data from the input buffer to the output buffer.
   *
   * @param point_view The input buffer containing the pointer to the current point.
   * @param output The output buffer to write the encoded data. It will be advanced.
   * @return The number of bytes written to the output buffer.
   */
  virtual size_t encode(const ConstBufferView& point_view, BufferView& output) = 0;

  virtual void reset() = 0;

  // Flush any remaining buffered data. Default implementation does nothing.
  virtual size_t flush(BufferView& /*output*/) {
    return 0;
  }
};

//------------------------------------------------------------------------------------------
class FieldEncoderCopy : public FieldEncoder {
 public:
  FieldEncoderCopy(size_t field_offset, FieldType field_type)
      : offset_(field_offset), field_size_(SizeOf(field_type)) {}

  size_t encode(const ConstBufferView& point_view, BufferView& output) override {
    memcpy(output.data(), point_view.data() + offset_, field_size_);
    output.trim_front(field_size_);
    return field_size_;
  }

  void reset() override {}

 private:
  size_t offset_;
  size_t field_size_;
};

//------------------------------------------------------------------------------------------
// Specialization for all the integer types
template <typename IntType>
class FieldEncoderInt : public FieldEncoder {
 public:
  FieldEncoderInt(size_t field_offset) : offset_(field_offset) {
    static_assert(std::is_integral<IntType>::value, "FieldEncoderInt requires an integral type");
  }

  size_t encode(const ConstBufferView& point_view, BufferView& output) override {
    int64_t value = ToInt64<IntType>(point_view.data() + offset_);
    int64_t diff = value - prev_value_;
    prev_value_ = value;
    int64_t var_size = encodeVarint64(diff, output.data());
    output.trim_front(var_size);
    return var_size;
  }

  void reset() override {
    prev_value_ = 0;
  }

 private:
  int64_t prev_value_ = 0;
  size_t offset_ = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossy compression
template <typename FloatType>
class FieldEncoderFloat_Lossy : public FieldEncoder {
 public:
  FieldEncoderFloat_Lossy(size_t field_offset, FloatType resolution)
      : offset_(field_offset), multiplier_(1.0 / resolution) {
    if (resolution <= 0.0) {
      throw std::runtime_error("FieldEncoder(Float/Lossy) requires a resolution with value > 0.0");
    }
  }

  size_t encode(const ConstBufferView& point_view, BufferView& output) override;

  void reset() override {
    prev_value_ = 0.0;
  }

 private:
  int64_t prev_value_ = 0.0;
  size_t offset_;
  FloatType multiplier_;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossless compression
template <typename FloatType>
class FieldEncoderFloat_XOR : public FieldEncoder {
 public:
  FieldEncoderFloat_XOR(size_t field_offset) : offset_(field_offset) {
    static_assert(std::is_floating_point<FloatType>::value, "FieldEncoderFloat_XOR requires a floating point type");
  }

  size_t encode(const ConstBufferView& point_view, BufferView& output) override;

  void reset() override {
    prev_bits_ = 0;
  }

 private:
  using IntType = std::conditional_t<std::is_same<FloatType, float>::value, uint32_t, uint64_t>;
  size_t offset_;
  IntType prev_bits_ = 0;
};

//------------------------------------------------------------------------------------------
// Gorilla/Chimp-style bit-packed XOR for lossless float compression.
// Encoding (per value after the first):
//   - If XOR(curr, prev) == 0: emit 1 bit '0'.
//   - Else emit '1'; if the non-zero window (leading_zeros, trailing_zeros) fits
//     INSIDE the previously-emitted window (curr_leading >= prev_leading AND
//     curr_trailing >= prev_trailing), emit '0' + meaningful bits (prev window size).
//     Otherwise emit '1' + leading_zeros(5b) + meaningful_bit_count-1 (6b) + meaningful bits.
// The first value is emitted raw (32 or 64 bits).
//
// IMPORTANT: to make the encoder composable with other byte-oriented encoders in the same
// per-point loop (each field encoder writes to a shared output buffer), encode() flushes
// its internal bit buffer to a byte boundary at the END of each call. This costs up to 7
// padding bits per call but keeps each point's output contiguous per encoder.
// flush() is retained for API symmetry but is a no-op for this class.
template <typename FloatType>
class FieldEncoderFloat_Gorilla : public FieldEncoder {
 public:
  FieldEncoderFloat_Gorilla(size_t field_offset) : offset_(field_offset) {
    static_assert(std::is_floating_point<FloatType>::value, "FieldEncoderFloat_Gorilla requires a floating point type");
  }

  size_t encode(const ConstBufferView& point_view, BufferView& output) override;

  // Flush partial bits to the next byte boundary. Returns number of bytes written.
  size_t flush(BufferView& output) override;

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
  static constexpr uint8_t kLeadingSentinel = 255;  // impossible-leading marker

  // Writes `nbits` low bits of `bits` into the internal bit buffer, draining full bytes to `output`.
  // Returns number of bytes written to `output`.
  inline size_t putBits(uint64_t bits, uint8_t nbits, BufferView& output);

  size_t offset_;
  IntType prev_bits_ = 0;
  uint8_t prev_leading_ = kLeadingSentinel;
  uint8_t prev_trailing_ = 0;

  // Internal bit accumulator: low-order bits are the earliest emitted.
  // `bit_count_` is the number of valid bits currently held.
  uint64_t bit_buf_ = 0;
  uint8_t bit_count_ = 0;

  bool first_ = true;
};

template <typename FloatType>
inline size_t FieldEncoderFloat_Gorilla<FloatType>::putBits(uint64_t bits, uint8_t nbits, BufferView& output) {
  // Mask to avoid polluting upper bits (for nbits < 64).
  if (nbits < 64) {
    bits &= (uint64_t{1} << nbits) - 1;
  }
  size_t bytes_written = 0;

  // If inserting `nbits` at position `bit_count_` would overflow the 64-bit accumulator,
  // split into two halves: emit the low `space = 64 - bit_count_` bits first, drain bytes,
  // then emit the remaining high bits.
  uint8_t space = static_cast<uint8_t>(64 - bit_count_);
  if (nbits > space) {
    // Low `space` bits first.
    const uint64_t low_mask = (space == 64) ? ~uint64_t{0} : ((uint64_t{1} << space) - 1);
    const uint64_t low_bits = bits & low_mask;
    bit_buf_ |= (low_bits << bit_count_);
    bit_count_ += space;
    // Drain full bytes (bit_count_ is now 64).
    while (bit_count_ >= 8) {
      output.data()[0] = static_cast<uint8_t>(bit_buf_ & 0xFF);
      output.trim_front(1);
      bit_buf_ >>= 8;
      bit_count_ -= 8;
      ++bytes_written;
    }
    // Remaining high bits.
    const uint8_t remaining = static_cast<uint8_t>(nbits - space);
    const uint64_t high_bits = bits >> space;
    bit_buf_ |= (high_bits << bit_count_);
    bit_count_ += remaining;
  } else {
    bit_buf_ |= (bits << bit_count_);
    bit_count_ += nbits;
  }

  while (bit_count_ >= 8) {
    output.data()[0] = static_cast<uint8_t>(bit_buf_ & 0xFF);
    output.trim_front(1);
    bit_buf_ >>= 8;
    bit_count_ -= 8;
    ++bytes_written;
  }
  return bytes_written;
}

template <typename FloatType>
inline size_t FieldEncoderFloat_Gorilla<FloatType>::encode(const ConstBufferView& point_view, BufferView& output) {
  IntType current_val_uint;
  memcpy(&current_val_uint, point_view.data() + offset_, sizeof(IntType));

  size_t bytes_written = 0;

  if (first_) {
    // Emit the first value raw, 32 or 64 bits.
    first_ = false;
    prev_bits_ = current_val_uint;
    bytes_written += putBits(static_cast<uint64_t>(current_val_uint), static_cast<uint8_t>(kTypeBits), output);
  } else {
    const IntType xor_val = current_val_uint ^ prev_bits_;
    prev_bits_ = current_val_uint;

    if (xor_val == 0) {
      // Same value: single '0' bit.
      bytes_written += putBits(0, 1, output);
    } else {
      // Non-zero XOR: emit '1' first.
      bytes_written += putBits(1, 1, output);
      const uint8_t leading = static_cast<uint8_t>(std::countl_zero(static_cast<IntType>(xor_val)));
      const uint8_t trailing = static_cast<uint8_t>(std::countr_zero(static_cast<IntType>(xor_val)));

      if (prev_leading_ != kLeadingSentinel && leading >= prev_leading_ && trailing >= prev_trailing_) {
        // Fits inside previous window.
        bytes_written += putBits(0, 1, output);
        const uint8_t meaningful = static_cast<uint8_t>(kTypeBits - prev_leading_ - prev_trailing_);
        const uint64_t bits = static_cast<uint64_t>(xor_val >> prev_trailing_);
        bytes_written += putBits(bits, meaningful, output);
      } else {
        // New window.
        bytes_written += putBits(1, 1, output);
        uint8_t stored_leading = leading;
        if (stored_leading > 31) {
          stored_leading = 31;
        }
        const uint8_t meaningful = static_cast<uint8_t>(kTypeBits - stored_leading - trailing);
        bytes_written += putBits(static_cast<uint64_t>(stored_leading), 5, output);
        bytes_written += putBits(static_cast<uint64_t>(meaningful - 1), 6, output);
        const uint64_t bits = static_cast<uint64_t>(xor_val >> trailing);
        bytes_written += putBits(bits, meaningful, output);
        prev_leading_ = stored_leading;
        prev_trailing_ = trailing;
      }
    }
  }

  // Byte-align: flush any partial byte to the output so each encode() call produces a
  // contiguous byte stream. This is what lets the encoder compose with other byte-oriented
  // encoders in the shared per-point output buffer.
  if (bit_count_ > 0) {
    output.data()[0] = static_cast<uint8_t>(bit_buf_ & 0xFF);
    output.trim_front(1);
    bit_buf_ = 0;
    bit_count_ = 0;
    ++bytes_written;
  }

  return bytes_written;
}

template <typename FloatType>
inline size_t FieldEncoderFloat_Gorilla<FloatType>::flush(BufferView& /*output*/) {
  // Per-call byte-alignment in encode() leaves nothing to flush at chunk boundaries.
  return 0;
}

//------------------------------------------------------------------------------------------
// Specialization for points XYZ and XYZI
class FieldEncoderFloatN_Lossy : public FieldEncoder {
 public:
  struct FieldData {
    size_t offset = 0;
    float resolution = 0.001f;
    FieldData() = default;
    FieldData(size_t o, float r) : offset(o), resolution(r) {}
  };

  FieldEncoderFloatN_Lossy(const std::vector<FieldData>& field_data);

  size_t encode(const ConstBufferView& point_view, BufferView& output) override;

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
inline size_t FieldEncoderFloat_Lossy<FloatType>::encode(const ConstBufferView& point_view, BufferView& output) {
  FloatType value_real = *(reinterpret_cast<const FloatType*>(point_view.data() + offset_));
  if (std::isnan(value_real)) {
    output.data()[0] = 0;  // value 0 is reserved for NaN
    prev_value_ = 0;
    output.trim_front(1);
    return 1;
  }
  const int64_t value = static_cast<int64_t>(std::round(value_real * multiplier_));
  const int64_t delta = value - prev_value_;
  prev_value_ = value;
  auto count = encodeVarint64(delta, output.data());
  output.trim_front(count);
  return count;
}

template <typename IntType>
inline size_t FieldEncoderFloat_XOR<IntType>::encode(const ConstBufferView& point_view, BufferView& output) {
  IntType current_val_uint;
  memcpy(&current_val_uint, point_view.data() + offset_, sizeof(IntType));

  const IntType residual = current_val_uint ^ prev_bits_;
  prev_bits_ = current_val_uint;

  memcpy(output.data(), &residual, sizeof(IntType));
  output.trim_front(sizeof(IntType));
  return sizeof(IntType);
}

//------------------------------------------------------------------------------------------

}  // namespace Cloudini
