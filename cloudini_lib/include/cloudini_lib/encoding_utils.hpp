#pragma once

#include <bit>
#include <cassert>
#include <limits>

#include "cloudini_lib/basic_types.hpp"
#include "cloudini_lib/contrib/span.hpp"

namespace Cloudini {

using ConstBufferView = Span<const uint8_t>;
using BufferView = Span<uint8_t>;

// Simple encoding that doesn't account for endianess (TODO?)
template <typename T>
inline void encode(const T& val, BufferView& buff) {
  memcpy(buff.data(), &val, sizeof(val));
  buff.trim_front(sizeof(val));
};

// specialization for std::string
template <>
inline void encode(const std::string& str, BufferView& buff) {
  uint16_t len = static_cast<uint16_t>(str.size());
  encode(len, buff);
  memcpy(buff.data(), str.c_str(), len);
  buff.trim_front(len);
}

inline size_t encodeVarint64(int64_t value, uint8_t* ptr) {
  uint64_t val = static_cast<uint64_t>((value << 1) ^ (value >> 63));  // Zig-zag encoding
  val++;                                                               // reserving value 0 for NaN
  uint8_t* ptr_start = ptr;
  while (val > 0x7F) {
    *ptr = (static_cast<uint8_t>((val & 0x7F) | 0x80));
    val >>= 7;
    ptr++;
  }
  *ptr = static_cast<uint8_t>(val);
  ptr++;
  return ptr - ptr_start;
}

inline size_t encodeVarint32(int32_t value, uint8_t* ptr) {
  uint32_t val = static_cast<uint32_t>((value << 1) ^ (value >> 31));  // Zig-zag encoding
  val++;                                                               // reserving value 0 for NaN
  uint8_t* ptr_start = ptr;
  while (val > 0x7F) {
    *ptr = (static_cast<uint8_t>((val & 0x7F) | 0x80));
    val >>= 7;
    ptr++;
  }
  *ptr = static_cast<uint8_t>(val);
  ptr++;
  return ptr - ptr_start;
}

template <typename T>
int64_t ToInt64(const uint8_t* ptr) {
  T tmp = *(reinterpret_cast<const T*>(ptr));
  return static_cast<int64_t>(tmp);
}

//-----------------------------------------------------------------------------------------

template <typename T>
inline void decode(ConstBufferView& buff, T& val) {
  memcpy(&val, buff.data(), sizeof(val));
  buff.trim_front(sizeof(val));
};

template <>
inline void decode(ConstBufferView& buff, std::string& str) {
  uint16_t len = 0;
  decode(buff, len);
  str.resize(len);
  memcpy(str.data(), buff.data(), len);
  buff.trim_front(len);
};

inline size_t decodeVarint(const uint8_t* buf, int64_t& val) {
  int64_t uval = 0;
  uint8_t shift = 0;
  const uint8_t* ptr = buf;
  while (true) {
    uint8_t byte = *ptr;
    ptr++;
    uval |= (byte & 0x7f) << shift;
    shift += 7;
    if ((byte & 0x80) == 0) {
      break;
    }
  }
  // we have reserved the value 0 for NaN
  uval--;
  // Perform zigzag decoding to retrieve the original signed value.
  val = static_cast<int64_t>((uval >> 1) ^ -(uval & 1));
  const auto count = static_cast<size_t>(ptr - buf);
  return count;
}

}  // namespace Cloudini
