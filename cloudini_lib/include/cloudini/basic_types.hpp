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

  // a pack of 3 FLOAT32 (x,y,z)
  POSITION_XYZ = 9,

  // Contains 3 FLOAT32 (x,y,z) and 1 UINT32 (intensity)
  // Common in LIDAR messages
  POINT_XYZI = 10,

};

struct PointField {
  std::string name;
  uint32_t offset = 0;
  FieldType type = FieldType::UNKNOWN;
  // optionally used by non integer types, when encoding is lossy
  std::optional<double> resolution;  // for XYZ fields
};

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
      return 4;
    case FieldType::FLOAT64:
      return 8;
    case FieldType::POSITION_XYZ:
      return 3 * 4;
    case FieldType::POINT_XYZI:
      return 4 * 4;
    default:
      return 0;
  }
}

}  // namespace Cloudini
