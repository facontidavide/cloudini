#pragma once

#include <bit>

#include "cloudini/basic_types.hpp"

namespace Cloudini {

struct ConstBufferView {
  const uint8_t* data = nullptr;
  size_t size = 0;

  void advance(size_t bytes) {
    data += bytes;
    size -= bytes;
  }
};

// not owning view of a buffer, similar to std::span
struct BufferView {
  uint8_t* data = nullptr;
  size_t size = 0;

  void advance(size_t bytes) {
    data += bytes;
    size -= bytes;
  }
};

// Simple encodiing that doesn't account for endianess (TODO?)
template <typename T>
inline void encode(const T& val, BufferView& buff) {
  memcpy(buff.data, &val, sizeof(val));
  buff.advance(sizeof(val));
};

// specialization for std::string
template <>
inline void encode(const std::string& str, BufferView& buff) {
  uint16_t len = static_cast<uint16_t>(str.size());
  encode(len, buff);
  memcpy(buff.data, str.c_str(), len);
  buff.advance(sizeof(len));
}

inline uint64_t encodeVarint(int64_t val, uint8_t* buf) {
  auto tmp = static_cast<uint64_t>(val);
  auto uval = static_cast<uint64_t>((tmp << 1) ^ (val >> 63));
  uint8_t* ptr = buf;
  while (uval >= 128) {
    *ptr = 0x80 | (uval & 0x7f);
    ptr++;
    uval >>= 7;
  }
  *ptr = uint8_t(uval);
  ptr++;
  return uint64_t(ptr - buf);
}

template <typename T>
int64_t ToInt64(const uint8_t* ptr) {
  T tmp = *(reinterpret_cast<const T*>(ptr));
  return static_cast<int64_t>(tmp);
}

//-----------------------------------------------------------------------------------------

template <typename T>
inline void decode(ConstBufferView& buff, T& val) {
  memcpy(&val, buff.data, sizeof(val));
  buff.advance(sizeof(val));
};

template <>
inline void decode(ConstBufferView& buff, std::string& str) {
  uint16_t len = 0;
  decode(buff, len);
  str.resize(len);
  memcpy(str.data(), buff.data, len);
  buff.advance(len);
};

inline size_t decodeVarint(const uint8_t* buf, int64_t& val) {
  uint64_t uval = 0;
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
  val = static_cast<int64_t>((uval >> 1) ^ -(uval & 1));
  return uint64_t(ptr - buf);
}

}  // namespace Cloudini
