#pragma once

#include <stdexcept>

#include "cloudini/cloudini.hpp"
#include "cloudini/encoding_utils.hpp"

namespace Cloudini {

class FieldEncoder {
 public:
  FieldEncoder() = default;

  virtual ~FieldEncoder() = default;

  /**
   * @brief Encode the field data from the input buffer to the output buffer.
   * Both buffers will be advanced.
   */
  virtual size_t encode(ConstBufferView& input, BufferView& output) = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for all the integer types
template <typename IntType>
class FieldEncoderInt : public FieldEncoder {
 public:
  FieldEncoderInt(size_t input_advance) : input_advance_(input_advance) {
    static_assert(std::is_integral<IntType>::value, "FieldEncoderInt requires an integral type");
  }

  size_t encode(ConstBufferView& input, BufferView& output) override {
    auto value = ToInt64<IntType>(input.data);
    auto diff = value - prev_value_;
    prev_value_ = value;
    auto offset = encodeVarint(diff, output.data);

    input.advance(input_advance_);
    output.advance(offset);
    return offset;
  }

 private:
  int64_t prev_value_ = 0;
  size_t input_advance_ = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossy compression
class FieldEncoderFloatLossy : public FieldEncoder {
 public:
  FieldEncoderFloatLossy(size_t input_advance, float resolution)
      : input_advance_(input_advance), resolution_inv_(1.0f / resolution) {
    if (resolution <= 0.0) {
      throw std::runtime_error("FieldEncoder(Float/Lossy) requires a resolution with value > 0.0");
    }
  }

  size_t encode(ConstBufferView& input, BufferView& output) override;

 private:
  int64_t prev_value_ = 0.0;
  size_t input_advance_ = 0;
  float resolution_inv_ = 0.0;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossless compression
class FieldEncoderFloatXOR : public FieldEncoder {
 public:
  FieldEncoderFloatXOR(size_t input_advance) : input_advance_(input_advance) {}

  size_t encode(ConstBufferView& input, BufferView& output) override;

 private:
  size_t input_advance_ = 0;
  uint32_t prev_value_bits_ = 0;
};

}  // namespace Cloudini
