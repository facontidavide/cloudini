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

inline size_t decodeVarint(const uint8_t* buf, [[maybe_unused]] size_t max_size, int64_t& val) {
  assert(max_size > 0 && "decodeVarint: empty input");
  uint64_t uval = 0;
  uint8_t shift = 0;
  const uint8_t* ptr = buf;
  while (true) {
    assert(static_cast<size_t>(ptr - buf) < max_size && "decodeVarint: truncated input");
    uint8_t byte = *ptr;
    ptr++;
    assert(!(shift >= 63 && (byte & 0x7f) > 1) && "decodeVarint: value overflow");
    uval |= (static_cast<uint64_t>(byte & 0x7f) << shift);
    shift += 7;
    if ((byte & 0x80) == 0) {
      break;
    }
  }
  assert(uval != 0 && "decodeVarint: unexpected NaN marker (value 0)");
  uval--;
  // Perform zigzag decoding to retrieve the original signed value.
  val = static_cast<int64_t>((uval >> 1) ^ static_cast<uint64_t>(-(static_cast<int64_t>(uval & 1))));
  const auto count = static_cast<size_t>(ptr - buf);
  return count;
}

}  // namespace Cloudini
