#pragma once
#include <cstdint>

#include "cloudini/cloudini.hpp"
#include "cloudini/encoding_utils.hpp"

namespace Cloudini {

class FieldEncoder {
 public:
  FieldEncoder(PointField field_info) : info_(field_info), field_size_(SizeOf(info_.type)) {}

  virtual ~FieldEncoder() = default;

  // Implementation of the encode function
  virtual int encode(BufferView input, BufferView& output) {
    // basic version doesn't implement any encoding
    memcpy(output.data, input.data, field_size_);
    output.data += field_size_;
    output.size -= field_size_;
    return field_size_;
  }

 protected:
  const PointField& info() const {
    return info_;
  }

 private:
  PointField info_;
  int32_t field_size_ = 0;
};

// Specialization for all the integer types
template <typename IntType>
class FieldEncoderInt : public FieldEncoder {
 public:
  FieldEncoderInt(PointField field_info) : FieldEncoder(field_info) {}

  size_t encode(BufferView input, BufferView& output) override {
    auto value = ToInt64<IntType>(input.data);
    auto diff = value - prev_value_;
    prev_value_ = value;
    return encodeVarint(diff)
  }

 private:
  int64_t prev_value_ = 0;
};

// not owning view of a buffer, similar to std::span
struct BufferView {
  uint8_t* data = nullptr;
  size_t size = 0;
};

enum class FirstStageOpt : uint8_t { NONE = 0, LOSSY = 1, LOSSLES = 2 };

enum class SecondStageOpt : uint8_t { NONE = 0, LZ4 = 1, ZSTD = 2 };

struct EncodingOptions {
  // the fist step of the encoding
  FirstStageOpt firts_stage = FirstStageOpt::LOSSY;

  // the second step of the encoding (general purpose compression)
  SecondStageOpt second_stage = SecondStageOpt::ZSTD;

  // Used only if (firts_stage == FirstStageOpt::LOSSY)
  std::optional<double> resolution_XYZ = std::nullopt;
};

}  // namespace Cloudini
