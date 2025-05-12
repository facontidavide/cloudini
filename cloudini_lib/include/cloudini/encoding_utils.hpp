#pragma once

#include <bit>
#include <cassert>
#include <limits>

#include "cloudini/basic_types.hpp"

namespace Cloudini {

struct ConstBufferView {
  const uint8_t* data = nullptr;
  size_t size = 0;

  ConstBufferView() = default;

  ConstBufferView(const ConstBufferView& other) = default;
  ConstBufferView(ConstBufferView&& other) = default;
  ConstBufferView& operator=(const ConstBufferView& other) = default;
  ConstBufferView& operator=(ConstBufferView&& other) = default;

  ConstBufferView(const void* data, size_t size) : data(reinterpret_cast<const uint8_t*>(data)), size(size) {}

  void advance(size_t bytes) {
    data += bytes;
    size -= bytes;
  }
};

// not owning view of a buffer, similar to std::span
struct BufferView {
  uint8_t* data = nullptr;
  size_t size = 0;

  BufferView() = default;

  BufferView(const BufferView& other) = default;
  BufferView(BufferView&& other) = default;
  BufferView& operator=(const BufferView& other) = default;
  BufferView& operator=(BufferView&& other) = default;

  BufferView(void* data, size_t size) : data(reinterpret_cast<uint8_t*>(data)), size(size) {}

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

template <typename IntType>
inline size_t encodeVarint(const IntType& val, uint8_t* buf) {
  static_assert(
      std::is_same_v<IntType, int64_t> || std::is_same_v<IntType, int32_t>, "encodeVarint requires int64_t or int32_t");

  // Perform zigzag encoding to handle negative values efficiently.
  // Zigzag encoding maps signed integers to unsigned integers such that
  // small absolute values (both positive and negative) have small encoded values.
  auto tmp = static_cast<IntType>(val);
  constexpr auto bits_shift = (sizeof(IntType) * 8 - 1);
  auto uval = static_cast<IntType>((tmp << 1) ^ (val >> bits_shift));

  // Encode the value using variable-length encoding.
  uint8_t* ptr = buf;
  while (uval >= 128) {
    *ptr = 0x80 | (uval & 0x7f);
    ptr++;
    uval >>= 7;
  }
  *ptr = uint8_t(uval);
  ptr++;
  assert((ptr - buf) <= sizeof(IntType));
  return static_cast<size_t>(ptr - buf);
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

template <typename IntType>
inline size_t decodeVarint(const uint8_t* buf, IntType& val) {
  static_assert(
      std::is_same_v<IntType, int64_t> || std::is_same_v<IntType, int32_t>, "decodeVarint requires int64_t or int32_t");

  IntType uval = 0;
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
  // Perform zigzag decoding to retrieve the original signed value.
  val = static_cast<IntType>((uval >> 1) ^ -(uval & 1));
  assert((ptr - buf) <= sizeof(IntType));
  return static_cast<size_t>(ptr - buf);
}

}  // namespace Cloudini
