#pragma once

#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

namespace Cloudini {

// Enums 1 to 8 conveniently match sensor_msgs/PointField.msg
enum class FieldType : uint8_t {
  UNKNOWN = 0,

  INT8 = 1,
  UINT8 = 2,

  INT16 = 3,
  UINT16 = 4,

  INT32 = 5,
  UINT32 = 6,

  FLOAT32 = 7,
  FLOAT64 = 8,

  INT64 = 9,
  UINT64 = 10,
};

struct PointField {
  // name of the field
  std::string name;

  // offset in memory with respect to the start of the point
  uint32_t offset = 0;

  // The data type of the field
  FieldType type = FieldType::UNKNOWN;

  // optionally used by non integer types, when encoding is lossy.
  // IMPORTANT: the maximum quantization error is equal to (0.5 * resolution)
  std::optional<float> resolution;

  bool operator==(const PointField& other) const {
    return name == other.name && offset == other.offset && type == other.type && resolution == other.resolution;
  }
  bool operator!=(const PointField& other) const {
    return !(*this == other);
  }
};

// If the value of PointField::offset is equal to this one, it means that the field was encoded, but we don't want to
// save it when doing the decoding
constexpr static uint32_t kDecodeButSkipStore = std::numeric_limits<uint32_t>::max();

inline int constexpr SizeOf(const FieldType& type) {
  switch (type) {
    case FieldType::INT8:
    case FieldType::UINT8:
      return sizeof(uint8_t);
    case FieldType::INT16:
    case FieldType::UINT16:
      return sizeof(uint16_t);
    case FieldType::INT32:
    case FieldType::UINT32:
      return sizeof(uint32_t);
    case FieldType::FLOAT32:
      return sizeof(float);
    case FieldType::FLOAT64:
      return sizeof(double);
    case FieldType::INT64:
      return sizeof(int64_t);
    case FieldType::UINT64:
      return sizeof(uint64_t);
    default:
      return 0;
  }
}

inline const char* ToString(const FieldType& type) {
  switch (type) {
    case FieldType::INT8:
      return "INT8";
    case FieldType::UINT8:
      return "UINT8";
    case FieldType::INT16:
      return "INT16";
    case FieldType::UINT16:
      return "UINT16";
    case FieldType::INT32:
      return "INT32";
    case FieldType::UINT32:
      return "UINT32";
    case FieldType::FLOAT32:
      return "FLOAT32";
    case FieldType::FLOAT64:
      return "FLOAT64";
    case FieldType::INT64:
      return "INT64";
    case FieldType::UINT64:
      return "UINT64";

    case FieldType::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

}  // namespace Cloudini
