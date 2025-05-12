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
   * Both buffers will be advanced.
   */
  virtual size_t encode(const ConstBufferView& point_view, BufferView& output) = 0;

  virtual void clear() = 0;
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

  void clear() override {}

 private:
  size_t offset_;
  size_t field_size_;
};

//------------------------------------------------------------------------------------------
// Specialization for all the integer types
template <typename IntType>
class FieldEncoderInt : public FieldEncoder {
 public:
  FieldEncoderInt(size_t field_offset) : field_offset_(field_offset) {
    static_assert(std::is_integral<IntType>::value, "FieldEncoderInt requires an integral type");
  }

  size_t encode(const ConstBufferView& point_view, BufferView& output) override {
    auto value = ToInt64<IntType>(point_view.data);
    auto diff = value - prev_value_;
    prev_value_ = value;
    auto var_size = encodeVarint(diff, output.data);
    output.advance(var_size);
    return var_size;
  }

  void clear() override {
    prev_value_ = 0;
  }

 private:
  int64_t prev_value_ = 0;
  size_t field_offset_ = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossy compression
class FieldEncoderFloat_Lossy : public FieldEncoder {
 public:
  FieldEncoderFloat_Lossy(size_t field_offset, float resolution)
      : field_offset_(field_offset), resolution_inv_(1.0f / resolution) {
    if (resolution <= 0.0) {
      throw std::runtime_error("FieldEncoder(Float/Lossy) requires a resolution with value > 0.0");
    }
  }

  size_t encode(const ConstBufferView& point_view, BufferView& output) override;

  void clear() override {
    prev_value_ = 0.0;
  }

 private:
  int64_t prev_value_ = 0.0;
  size_t field_offset_;
  float resolution_inv_;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossless compression
class FieldEncoderFloat_XOR : public FieldEncoder {
 public:
  FieldEncoderFloat_XOR(size_t field_offset) : field_offset_(field_offset) {}

  size_t encode(const ConstBufferView& point_view, BufferView& output) override;

  void clear() override {
    prev_value_bits_ = 0.0;
  }

 private:
  size_t field_offset_;
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

  FieldEncoderFloatN_Lossy(const std::vector<FieldData>& field_data) : fields_count_(field_data.size()) {
    if (fields_count_ < 2) {
      throw std::runtime_error("FieldEncoderFloatN_Lossy requires at least one field");
    }
    if (fields_count_ > 4) {
      throw std::runtime_error("FieldEncoderFloatN_Lossy can have at most 4 fields");
    }

    for (size_t i = 0; i < fields_count_; ++i) {
      multiplier_[i] = 1.0f / field_data[i].resolution;
      if (multiplier_[i] <= 0.0) {
        throw std::runtime_error("FieldEncoderFloatN_Lossy requires a resolution with value > 0.0");
      }
      offset_[i] = field_data[i].offset;
    }
  }

  size_t encode(const ConstBufferView& point_view, BufferView& output) override;

  void clear() override {
    prev_vect_ = Vector4i(0, 0, 0, 0);
  }

 private:
  std::array<size_t, 4> offset_ = {0, 0, 0, 0};
  size_t fields_count_ = 0;

  Vector4i prev_vect_ = Vector4i(0, 0, 0, 0);
  Vector4f multiplier_ = Vector4f(0, 0, 0, 0);
};

}  // namespace Cloudini
