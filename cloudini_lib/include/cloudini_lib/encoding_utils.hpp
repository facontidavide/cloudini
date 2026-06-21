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

#include <bit>
#include <cassert>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "cloudini_lib/basic_types.hpp"
#include "cloudini_lib/contrib/span.hpp"

namespace Cloudini {

using ConstBufferView = Span<const uint8_t>;
using BufferView = Span<uint8_t>;

// Simple encoding that doesn't account for endianess (TODO?)
template <typename T>
inline void encode(const T& val, BufferView& buff) {
  if (buff.size() < sizeof(val)) {
    throw std::runtime_error("encode: not enough output buffer space");
  }
  memcpy(buff.data(), &val, sizeof(val));
  buff.trim_front(sizeof(val));
};

// specialization for std::string
template <>
inline void encode(const std::string& str, BufferView& buff) {
  uint16_t len = static_cast<uint16_t>(str.size());
  encode(len, buff);
  if (buff.size() < len) {
    throw std::runtime_error("encode(string): not enough output buffer space");
  }
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

template <typename T>
int64_t ToInt64(const uint8_t* ptr) {
  T tmp = *(reinterpret_cast<const T*>(ptr));
  return static_cast<int64_t>(tmp);
}

//-----------------------------------------------------------------------------------------

template <typename T>
inline void decode(ConstBufferView& buff, T& val) {
  if (buff.size() < sizeof(val)) {
    throw std::runtime_error("decode: not enough input data");
  }
  memcpy(&val, buff.data(), sizeof(val));
  buff.trim_front(sizeof(val));
};

template <>
inline void decode(ConstBufferView& buff, std::string& str) {
  uint16_t len = 0;
  decode(buff, len);
  if (buff.size() < len) {
    throw std::runtime_error("decode(string): not enough input data");
  }
  str.resize(len);
  memcpy(str.data(), buff.data(), len);
  buff.trim_front(len);
};

inline size_t decodeVarint(const uint8_t* buf, size_t max_size, int64_t& val) {
  if (max_size == 0) {
    throw std::runtime_error("decodeVarint: empty input");
  }
  uint64_t uval;
  size_t count;
  const uint8_t b0 = buf[0];
  if ((b0 & 0x80) == 0) {
    // 1-byte varint: by far the most common case for small deltas. Fully safe
    // (max_size >= 1 guaranteed above).
    uval = b0;
    count = 1;
  } else if (max_size >= 2 && (buf[1] & 0x80) == 0) {
    // 2-byte varint. Max value 0x3FFF, so no overflow is possible. The
    // `max_size >= 2` guard is checked before buf[1] so we never read past bound.
    uval = static_cast<uint64_t>(b0 & 0x7f) | (static_cast<uint64_t>(buf[1]) << 7);
    count = 2;
  } else {
    // General path for 3+ byte varints: per-byte bounds + overflow checks.
    uval = 0;
    uint8_t shift = 0;
    const uint8_t* ptr = buf;
    while (true) {
      if (static_cast<size_t>(ptr - buf) >= max_size) {
        throw std::runtime_error("decodeVarint: truncated input");
      }
      uint8_t byte = *ptr;
      ptr++;
      const uint8_t payload = byte & 0x7f;
      if (shift >= 64 || (shift == 63 && payload > 1)) {
        throw std::runtime_error("decodeVarint: value overflow");
      }
      uval |= (static_cast<uint64_t>(payload) << shift);
      if ((byte & 0x80) == 0) {
        break;
      }
      if (shift >= 63) {
        throw std::runtime_error("decodeVarint: value overflow");
      }
      shift = static_cast<uint8_t>(shift + 7);
    }
    count = static_cast<size_t>(ptr - buf);
  }
  if (uval == 0) {
    throw std::runtime_error("decodeVarint: unexpected NaN marker");
  }
  uval--;
  // Perform zigzag decoding to retrieve the original signed value.
  val = static_cast<int64_t>((uval >> 1) ^ static_cast<uint64_t>(-(static_cast<int64_t>(uval & 1))));
  return count;
}

}  // namespace Cloudini
