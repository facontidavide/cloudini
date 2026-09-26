/*
 * Copyright 2025 Davide Faconti
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>
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

// Returns true if the field name denotes a color packed as integer bits (e.g. "rgb", "rgba").
// Many ROS drivers store packed RGB(A) as uint32 bits reinterpreted into a FLOAT32 field;
// such fields must never be quantized (lossy), otherwise the colors are destroyed.
// The match is case-insensitive.
inline bool isPackedColorField(std::string_view name) {
  constexpr std::string_view kNames[] = {"rgb", "rgba", "bgr", "bgra", "argb", "abgr"};
  for (const auto& candidate : kNames) {
    if (name.size() != candidate.size()) {
      continue;
    }
    bool equal = true;
    for (size_t i = 0; i < name.size() && equal; ++i) {
      char c = name[i];
      if (c >= 'A' && c <= 'Z') {
        c = static_cast<char>(c - 'A' + 'a');
      }
      equal = (c == candidate[i]);
    }
    if (equal) {
      return true;
    }
  }
  return false;
}

// Default resolution to use for a field when the user did not explicitly specify one:
// FLOAT32 fields get `default_resolution`, except packed colors, which stay lossless.
inline std::optional<float> defaultFieldResolution(const PointField& field, std::optional<float> default_resolution) {
  if (field.type == FieldType::FLOAT32 && !isPackedColorField(field.name)) {
    return default_resolution;
  }
  return std::nullopt;
}

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

}  // namespace Cloudini
