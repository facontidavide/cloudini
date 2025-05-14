#pragma once

#include <array>
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
   *
   * @param point_view The input buffer containing the pointer to the current point.
   * @param output The output buffer to write the encoded data. It will be advanced.
   * @return The number of bytes written to the output buffer.
   */
  virtual size_t encode(const ConstBufferView& point_view, BufferView& output) = 0;

  virtual void reset() = 0;
};

//------------------------------------------------------------------------------------------
class FieldEncoderCopy : public FieldEncoder {
 public:
  FieldEncoderCopy(size_t field_offset, FieldType field_type)
      : offset_(field_offset), field_size_(SizeOf(field_type)) {}

  size_t encode(const ConstBufferView& point_view, BufferView& output) override {
    memcpy(output.data, point_view.data + offset_, field_size_);
    output.advance(field_size_);
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
    auto value = ToInt64<IntType>(point_view.data + offset_);
    auto diff = value - prev_value_;
    prev_value_ = value;
    auto var_size = encodeVarint(diff, output.data);
    output.advance(var_size);
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
class FieldEncoderFloat_Lossy : public FieldEncoder {
 public:
  FieldEncoderFloat_Lossy(size_t field_offset, float resolution)
      : offset_(field_offset), multiplier_(1.0F / (2.0F * resolution)) {
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
  float multiplier_;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossless compression
class FieldEncoderFloat_XOR : public FieldEncoder {
 public:
  FieldEncoderFloat_XOR(size_t field_offset) : offset_(field_offset) {}

  size_t encode(const ConstBufferView& point_view, BufferView& output) override;

  void reset() override {
    prev_value_bits_ = 0;
  }

 private:
  size_t offset_;
  float prev_1_ = 0.0;
  float prev_2_ = 0.0;
  uint32_t prev_value_bits_ = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for points XYZ and XIZI
class FieldEncoderFloatN_Lossy : public FieldEncoder {
 public:
  struct FieldData {
    size_t offset;
    float resolution;
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

}  // namespace Cloudini
