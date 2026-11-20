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

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

#include "cloudini_lib/intrinsics.hpp"
#include "codec_common.hpp"

namespace Cloudini::detail {

#if defined(__SSE4_1__)
#define CLOUDINI_HAS_VARINT_BLOCKS 1

#if defined(__GNUC__)
#define CLOUDINI_VARINT_INLINE inline __attribute__((always_inline))
#else
#define CLOUDINI_VARINT_INLINE inline
#endif

//------------------------------------------------------------------------------------------
// Block decoding of a stream of varints (encodeVarint64: zig-zag + 1, 7 bits per byte): up to 4 values per
// byte shuffle, into a column of int32. It handles the streams whose varints have at most 4 bytes (values
// below 2^27 in magnitude) and no zero code (the NaN marker of the float streams, invalid elsewhere), and
// reports the others, which the callers decode value by value.
//
// The serial part of decoding is the position of the next varint, known only once the previous ones are
// parsed. Several streams decoded in the same loop overlap those chains, so a single stream is first cut
// into kVarintChains parts at varint boundaries (a varint starts after any byte below 0x80), found with a
// scan of the continuation bits.

// Byte shuffle that spreads the first varints among 8 stream bytes into 32-bit lanes, by the continuation
// bits of those bytes: as many as end within the 8 bytes, at most 4, of at most 4 bytes each.
struct VarintUnpack {
  alignas(16) uint8_t shuffle[16];
  uint8_t count;        // varints unpacked; 0 when the first one is longer than 4 bytes
  uint8_t bytes;        // their total size
  uint8_t empty_lanes;  // the lanes without a varint, as a movemask
};
inline constexpr std::array<VarintUnpack, 256> kVarintUnpacks = [] {
  std::array<VarintUnpack, 256> table{};
  for (unsigned mask = 0; mask < 256; ++mask) {
    VarintUnpack& unpack = table[mask];
    for (uint8_t& byte : unpack.shuffle) {
      byte = 0x80;  // a zero byte in the lane
    }
    unsigned pos = 0, lane = 0;
    while (lane < 4) {
      unsigned len = 1;
      while (pos + len <= 8 && ((mask >> (pos + len - 1)) & 1u)) {
        ++len;
      }
      if (pos + len > 8 || len > 4) {
        break;  // ends beyond the 8 bytes, or too long
      }
      for (unsigned b = 0; b < len; ++b) {
        unpack.shuffle[lane * 4 + b] = static_cast<uint8_t>(pos + b);
      }
      pos += len;
      ++lane;
    }
    unpack.count = static_cast<uint8_t>(lane);
    unpack.bytes = static_cast<uint8_t>(pos);
    // with no varint at all the lanes never match: the caller gives up
    unpack.empty_lanes = lane == 0 ? 0xFF : static_cast<uint8_t>((0xFu << lane) & 0xFu);
  }
  return table;
}();

// Unpacks the next varints of a stream (1 to 4) into values at `out`, which needs room for 4. Reads 16
// bytes at `ptr`. False for a zero code and for a varint longer than 4 bytes.
CLOUDINI_VARINT_INLINE bool unpackVarints(const uint8_t*& ptr, int32_t*& out) {
  const __m128i bytes = _mm_loadu_si128(reinterpret_cast<const __m128i*>(ptr));
  const VarintUnpack& unpack = kVarintUnpacks[static_cast<size_t>(_mm_movemask_epi8(bytes) & 0xFF)];
  const __m128i lanes = _mm_and_si128(
      _mm_shuffle_epi8(bytes, _mm_load_si128(reinterpret_cast<const __m128i*>(unpack.shuffle))), _mm_set1_epi8(0x7F));
  // 7-bit groups: b0 + (b1 << 7) in each 16-bit half, then lo + (hi << 14)
  const __m128i halves = _mm_maddubs_epi16(_mm_set1_epi16(static_cast<short>(0x8001)), lanes);
  const __m128i code = _mm_madd_epi16(halves, _mm_set1_epi32(0x40000001));
  const int zero_lanes = _mm_movemask_ps(_mm_castsi128_ps(_mm_cmpeq_epi32(code, _mm_setzero_si128())));
  if (zero_lanes != unpack.empty_lanes) {
    return false;
  }
  // encodeVarint64 in reverse: minus 1, then zig-zag
  const __m128i u = _mm_sub_epi32(code, _mm_set1_epi32(1));
  const __m128i value =
      _mm_xor_si128(_mm_srli_epi32(u, 1), _mm_sub_epi32(_mm_setzero_si128(), _mm_and_si128(u, _mm_set1_epi32(1))));
  _mm_storeu_si128(reinterpret_cast<__m128i*>(out), value);
  out += unpack.count;
  ptr += unpack.bytes;
  return true;
}

// One varint, for the end of a stream. False when unpackVarints would be, or at `end`.
inline bool readVarintChecked(const uint8_t*& ptr, const uint8_t* end, int32_t& value) {
  uint32_t code = 0;
  for (unsigned shift = 0; shift < 28; shift += 7) {
    if (ptr == end) {
      return false;
    }
    const uint8_t byte = *ptr++;
    code |= static_cast<uint32_t>(byte & 0x7Fu) << shift;
    if (byte < 0x80u) {
      if (code == 0) {
        return false;
      }
      const uint32_t u = code - 1;
      value = static_cast<int32_t>((u >> 1) ^ (uint32_t{0} - (u & 1u)));
      return true;
    }
  }
  return false;
}

// count[k] varints of each of N streams into the columns `out` (room for count[k] + 3 values each), in one
// loop. Stream k starts at ptr[k] and may not go beyond end[k]; 16 bytes can be read anywhere before
// `readable_end`. On success ptr[k] is the end of the varints of stream k.
template <size_t N>
bool readVarintStreams(
    std::array<const uint8_t*, N>& ptr, const std::array<const uint8_t*, N>& end, const uint8_t* readable_end,
    const std::array<size_t, N>& count, const std::array<int32_t*, N>& out) {
  std::array<int32_t*, N> cursor = out;
  // Steps that stream k can take without a test: each one unpacks at most 4 values and 8 bytes.
  auto safe_steps = [&](size_t k) {
    const size_t values_left = count[k] - static_cast<size_t>(cursor[k] - out[k]);
    const size_t bytes_left = static_cast<size_t>(readable_end - ptr[k]);
    return std::min(values_left / 4, bytes_left >= 16 ? (bytes_left - 16) / 8 + 1 : 0);
  };
  for (;;) {
    std::array<size_t, N> active{};
    size_t active_count = 0;
    size_t steps = std::numeric_limits<size_t>::max();
    for (size_t k = 0; k < N; ++k) {
      if (const size_t s = safe_steps(k); s > 0) {
        active[active_count++] = k;
        steps = std::min(steps, s);
      }
    }
    if (active_count == 0) {
      break;
    }
    // the active streams in one loop: their pointer chains overlap
    auto run = [&]<size_t... I>(std::index_sequence<I...>) {
      std::array<const uint8_t*, sizeof...(I)> p = {ptr[active[I]]...};
      std::array<int32_t*, sizeof...(I)> c = {cursor[active[I]]...};
      bool ok = true;
      for (size_t s = 0; s < steps && ok; ++s) {
        ok = (unpackVarints(p[I], c[I]) && ...);
      }
      ((ptr[active[I]] = p[I], cursor[active[I]] = c[I]), ...);
      return ok;
    };
    bool ok = true;
    if (N >= 4 && active_count == 4) {
      ok = run(std::make_index_sequence<(N >= 4 ? 4 : 1)>{});
    } else if (N >= 3 && active_count == 3) {
      ok = run(std::make_index_sequence<(N >= 3 ? 3 : 1)>{});
    } else if (N >= 2 && active_count == 2) {
      ok = run(std::make_index_sequence<(N >= 2 ? 2 : 1)>{});
    } else {
      ok = run(std::make_index_sequence<1>{});
    }
    if (!ok) {
      return false;
    }
  }
  for (size_t k = 0; k < N; ++k) {  // the last values of each stream
    if (ptr[k] > end[k]) {
      return false;
    }
    for (int32_t* const column_end = out[k] + count[k]; cursor[k] < column_end; ++cursor[k]) {
      if (!readVarintChecked(ptr[k], end[k], *cursor[k])) {
        return false;
      }
    }
  }
  return true;
}

constexpr size_t kVarintChains = 4;

// Where the varints number index[0] <= index[1] <= ... of a stream start: after as many bytes below 0x80.
// False when the stream has fewer varints before `end`.
template <size_t N>
bool findVarintStarts(
    const uint8_t* begin, const uint8_t* end, const std::array<size_t, N>& index,
    std::array<const uint8_t*, N>& start) {
  static constexpr std::array<uint8_t, 256> kBits = [] {
    std::array<uint8_t, 256> table{};
    for (unsigned value = 0; value < 256; ++value) {
      for (unsigned bit = 0; bit < 8; ++bit) {
        table[value] = static_cast<uint8_t>(table[value] + ((value >> bit) & 1u));
      }
    }
    return table;
  }();
  size_t found = 0;
  while (found < N && index[found] == 0) {
    start[found++] = begin;
  }
  size_t varints = 0;  // that end before ptr
  const uint8_t* ptr = begin;
  for (; found < N && end - ptr >= 16; ptr += 16) {
    const __m128i bytes = _mm_loadu_si128(reinterpret_cast<const __m128i*>(ptr));
    unsigned ends = ~static_cast<unsigned>(_mm_movemask_epi8(bytes)) & 0xFFFFu;  // the bytes below 0x80
    const size_t block = size_t{kBits[ends & 0xFFu]} + kBits[ends >> 8];
    while (found < N && index[found] <= varints + block) {  // starts within these 16 bytes, or right after
      unsigned rest = ends;
      for (size_t skip = index[found] - varints; skip > 1; --skip) {
        rest &= rest - 1;
      }
      start[found++] = ptr + std::countr_zero(rest) + 1;
    }
    varints += block;
  }
  for (; found < N && ptr < end; ++ptr) {
    varints += *ptr < 0x80u;
    while (found < N && index[found] == varints) {
      start[found++] = ptr + 1;
    }
  }
  return found == N;
}

// The `count` varints of the stream at `begin`, which ends somewhere before `readable_end`, into `out` (room
// for count + 3 values). On success `stream_end` is the end of the stream.
inline bool readVarintStream(
    const uint8_t* begin, const uint8_t* readable_end, size_t count, int32_t* out, const uint8_t*& stream_end) {
  constexpr size_t N = kVarintChains;
  if (count < 16 * N) {  // not worth the scan
    std::array<const uint8_t*, 1> one = {begin};
    if (!readVarintStreams<1>(one, {readable_end}, readable_end, {count}, {out})) {
      return false;
    }
    stream_end = one[0];
    return true;
  }
  // N parts of count / N varints, the last one with the rest. Its end is where its varints end: only the
  // N - 1 boundaries between the parts are searched.
  const size_t part = count / N;
  std::array<size_t, N - 1> index;
  std::array<const uint8_t*, N - 1> boundary;
  for (size_t k = 0; k + 1 < N; ++k) {
    index[k] = (k + 1) * part;
  }
  if (!findVarintStarts<N - 1>(begin, readable_end, index, boundary)) {
    return false;
  }
  std::array<const uint8_t*, N> ptr;
  std::array<const uint8_t*, N> end;
  std::array<size_t, N> counts;
  std::array<int32_t*, N> columns;
  for (size_t k = 0; k < N; ++k) {
    ptr[k] = k == 0 ? begin : boundary[k - 1];
    end[k] = k + 1 < N ? boundary[k] : readable_end;
    counts[k] = k + 1 < N ? part : count - (N - 1) * part;
    columns[k] = out + k * part;
  }
  // A part only writes the 4 lanes of a step inside its own values, so the parts do not overwrite each other.
  if (!readVarintStreams<N>(ptr, end, readable_end, counts, columns)) {
    return false;
  }
  for (size_t k = 0; k + 1 < N; ++k) {
    if (ptr[k] != end[k]) {
      return false;
    }
  }
  stream_end = ptr[N - 1];
  return true;
}

// The int32 column of this thread that the block decoders unpack a section into, one for all of them (a
// section is decoded before the next one is read).
inline int32_t* threadVarintColumn(size_t count) {
  thread_local ScratchArray<int32_t> column;
  column.ensure(count);
  return column.data();
}

// Running sums of the 4 values of `x`, added to lane 3 of `carry`, which then holds the last sum in every
// lane. The arithmetic wraps around.
CLOUDINI_VARINT_INLINE __m128i prefixSum4(__m128i x, __m128i& carry) {
  x = _mm_add_epi32(x, _mm_slli_si128(x, 4));
  x = _mm_add_epi32(_mm_add_epi32(x, _mm_slli_si128(x, 8)), carry);
  carry = _mm_shuffle_epi32(x, _MM_SHUFFLE(3, 3, 3, 3));
  return x;
}

#endif  // __SSE4_1__

}  // namespace Cloudini::detail
