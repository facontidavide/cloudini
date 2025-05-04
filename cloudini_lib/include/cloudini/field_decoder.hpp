#pragma once

#include <stdexcept>

#include "cloudini/encoding_utils.hpp"
#include "cloudini/intrinsics.hpp"

namespace Cloudini {

class FieldDecoder {
 public:
  FieldDecoder() = default;

  virtual ~FieldDecoder() = default;

  /**
   * @brief Decode the field data from the input buffer to the output buffer.
   * Both buffers will be advanced.
   */
  virtual void decode(ConstBufferView& input, BufferView& output) = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for all the integer types
template <typename IntType>
class FieldDecoderInt : public FieldDecoder {
 public:
  FieldDecoderInt(size_t output_advance) : output_advance_(output_advance) {
    static_assert(std::is_integral<IntType>::value, "FieldDecoderInt requires an integral type");
  }

  void decode(ConstBufferView& input, BufferView& output) override {
    int64_t diff = 0;
    auto offset = decodeVarint(input.data, diff);

    IntType value = prev_value_ + diff;
    prev_value_ = value;

    memcpy(output.data, &value, sizeof(IntType));

    input.advance(offset);
    output.advance(output_advance_);
  }

 private:
  size_t output_advance_ = 0;
  int64_t prev_value_ = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossy compression
class FieldDecoderFloat_Lossy : public FieldDecoder {
 public:
  FieldDecoderFloat_Lossy(size_t output_advance, float resolution)
      : output_advance_(output_advance), resolution_(resolution) {
    if (resolution <= 0.0) {
      throw std::runtime_error("FieldDecoder(Float/Lossy) requires a resolution with value > 0.0");
    }
  }

  void decode(ConstBufferView& input, BufferView& output) override;

 private:
  size_t output_advance_ = 0;
  float resolution_ = 0.0;
  int64_t prev_value_ = 00;
};

//------------------------------------------------------------------------------------------
// Specialization for POSITION_XYZ
class FieldDecoderXYZ_Lossy : public FieldDecoder {
 public:
  FieldDecoderXYZ_Lossy(size_t output_advance, float resolution)
      : output_advance_(output_advance),
        resolution_(resolution),
        multiplier_({resolution_, resolution_, resolution_, 0.0f}) {
    if (resolution <= 0.0) {
      throw std::runtime_error("FieldDecoder(XYZ/Lossy) requires a resolution with value > 0.0");
    }
  }

  void decode(ConstBufferView& input, BufferView& output) override;

 private:
  size_t output_advance_ = 0;
  float resolution_ = 0.0;
  Vector4f multiplier_ = Vector4f(0, 0, 0, 0);
  Vector4l prev_vect_ = Vector4l(0, 0, 0, 0);
};

//------------------------------------------------------------------------------------------
// Specialization for POINT_XYZI
class FieldDecoderXYZI_Lossy : public FieldDecoder {
 public:
  FieldDecoderXYZI_Lossy(size_t output_advance, Vector4f multiplier)
      : output_advance_(output_advance), multiplier_(multiplier) {}

  void decode(ConstBufferView& input, BufferView& output) override;

 private:
  size_t output_advance_ = 0;
  Vector4f multiplier_ = Vector4f(0, 0, 0, 0);
  Vector4l prev_vect_ = Vector4l(0, 0, 0, 0);
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossless compression
class FieldDecoderFloat_XOR : public FieldDecoder {
 public:
  FieldDecoderFloat_XOR(size_t output_advance) : output_advance_(output_advance) {}

  void decode(ConstBufferView& input, BufferView& output) override;

 private:
  size_t output_advance_ = 0;
  uint32_t prev_value_bits_ = 0;
};

}  // namespace Cloudini
