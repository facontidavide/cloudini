#pragma once

#include <stdexcept>

#include "cloudini/cloudini.hpp"
#include "cloudini/encoding_utils.hpp"

namespace Cloudini {

class FieldEncoder {
 public:
  FieldEncoder(PointField field_info) : info_(field_info) {}

  virtual ~FieldEncoder() = default;

  /**
   * @brief Encode the field data from the input buffer to the output buffer.
   * Both buffers will be advanced.
   */
  virtual size_t encode(const ConstBufferView& input, BufferView& output) = 0;

 protected:
  const PointField& info() const {
    return info_;
  }

 private:
  PointField info_;
};

//------------------------------------------------------------------------------------------
// Specialization for all the integer types
template <typename IntType>
class FieldEncoderInt : public FieldEncoder {
 public:
  FieldEncoderInt(PointField field_info) : FieldEncoder(field_info) {
    static_assert(std::is_integral<IntType>::value, "FieldEncoderInt requires an integral type");
  }

  size_t encode(const ConstBufferView& input, BufferView& output) override {
    auto value = ToInt64<IntType>(input.data);
    auto diff = value - prev_value_;
    prev_value_ = value;
    auto offset = encodeVarint(diff, output.data);
    output.advance(offset);
    return offset;
  }

 private:
  int64_t prev_value_ = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossy compression
class FieldEncoderFloatLossy : public FieldEncoder {
 public:
  FieldEncoderFloatLossy(PointField field_info);
  size_t encode(const ConstBufferView& input, BufferView& output) override;

 private:
  int64_t prev_value_ = 0.0;
  float resolution_inv_ = 0.0;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossless compression
class FieldEncoderFloatXOR : public FieldEncoder {
 public:
  FieldEncoderFloatXOR(PointField field_info);

  size_t encode(const ConstBufferView& input, BufferView& output) override;

 private:
  uint32_t prev_value_bits_ = 0;
};

}  // namespace Cloudini
