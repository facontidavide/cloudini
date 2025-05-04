#pragma once

#include <stdexcept>

#include "cloudini/cloudini.hpp"
#include "cloudini/encoding_utils.hpp"

namespace Cloudini {

class FieldDecoder {
 public:
  FieldDecoder(PointField field_info) : info_(field_info) {}

  virtual ~FieldDecoder() = default;

  /**
   * @brief Decode the field data from the input buffer to the output buffer.
   * Both buffers will be advanced.
   */
  virtual void decode(ConstBufferView& input, BufferView& output) = 0;

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
class FieldDecoderInt : public FieldDecoder {
 public:
  FieldDecoderInt(PointField field_info) : FieldDecoder(field_info) {
    static_assert(std::is_integral<IntType>::value, "FieldDecoderInt requires an integral type");
  }

  void decode(ConstBufferView& input, BufferView& output) override {
    int64_t diff = 0;
    auto offset = decodeVarint(input.data, diff);
    input.advance(offset);

    IntType value = prev_value_ + diff;
    prev_value_ = value;

    memcpy(output.data, &value, sizeof(IntType));
    output.advance(sizeof(value));
  }

 private:
  int64_t prev_value_ = 0;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossy compression
class FieldDecoderFloatLossy : public FieldDecoder {
 public:
  FieldDecoderFloatLossy(PointField field_info);
  void decode(ConstBufferView& input, BufferView& output) override;

 private:
  int64_t prev_value_ = 00;
  float resolution_ = 0.0;
};

//------------------------------------------------------------------------------------------
// Specialization for floating point types and lossless compression
class FieldDecoderFloatXOR : public FieldDecoder {
 public:
  FieldDecoderFloatXOR(PointField field_info);

  void decode(ConstBufferView& input, BufferView& output) override;

 private:
  uint32_t prev_value_bits_ = 0;
};

}  // namespace Cloudini
