#pragma once

#include <stdexcept>

#include "cloudini/encoding_utils.hpp"
#include "cloudini/intrinsics.hpp"

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

  virtual void clear() = 0;
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

  void clear() override {
    prev_value_ = 0;
    input_advance_ = 0;
  }

 private:
  int64_t prev_value_ = 0;
  size_t input_advance_ = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossy compression
class FieldEncoderFloat_Lossy : public FieldEncoder {
 public:
  FieldEncoderFloat_Lossy(size_t input_advance, float resolution)
      : input_advance_(input_advance), resolution_inv_(1.0f / resolution) {
    if (resolution <= 0.0) {
      throw std::runtime_error("FieldEncoder(Float/Lossy) requires a resolution with value > 0.0");
    }
  }

  size_t encode(ConstBufferView& input, BufferView& output) override;

  void clear() override {
    prev_value_ = 0.0;
    input_advance_ = 0;
  }

 private:
  int64_t prev_value_ = 0.0;
  size_t input_advance_ = 0;
  float resolution_inv_ = 0.0;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossless compression
class FieldEncoderFloat_XOR : public FieldEncoder {
 public:
  FieldEncoderFloat_XOR(size_t input_advance) : input_advance_(input_advance) {}

  size_t encode(ConstBufferView& input, BufferView& output) override;

  void clear() override {
    prev_value_bits_ = 0.0;
    input_advance_ = 0;
  }

 private:
  size_t input_advance_ = 0;
  uint32_t prev_value_bits_ = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for POSITION_XYZ
class FieldEncoderXYZ_Lossy : public FieldEncoder {
 public:
  FieldEncoderXYZ_Lossy(size_t input_advance, float resolution)
      : input_advance_(input_advance),
        resolution_inv_(1.0f / resolution),
        multiplier_({resolution_inv_, resolution_inv_, resolution_inv_, 1.0}) {
    if (resolution <= 0.0) {
      throw std::runtime_error("FieldEncoder(XYZ/Lossy) requires a resolution with value > 0.0");
    }
  }

  size_t encode(ConstBufferView& input, BufferView& output) override;

  void clear() override {
    prev_vect_ = Vector4i(0, 0, 0, 0);
    input_advance_ = 0;
  }

 private:
  size_t input_advance_ = 0;
  Vector4i prev_vect_ = Vector4i(0, 0, 0, 0);
  float resolution_inv_ = 0.0;
  Vector4f multiplier_ = Vector4f(0, 0, 0, 0);
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossless compression
class FieldEncoderXYZI_Lossy : public FieldEncoder {
 public:
  FieldEncoderXYZI_Lossy(size_t input_advance, const Vector4f& multiplier)
      : input_advance_(input_advance), multiplier_(multiplier) {}

  size_t encode(ConstBufferView& input, BufferView& output) override;

  void clear() override {
    prev_vect_ = Vector4i(0, 0, 0, 0);
    input_advance_ = 0;
  }

 private:
  size_t input_advance_ = 0;
  Vector4i prev_vect_ = Vector4i(0, 0, 0, 0);
  Vector4f multiplier_ = Vector4f(0, 0, 0, 0);
};

}  // namespace Cloudini
