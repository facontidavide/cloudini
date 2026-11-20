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
#include <memory>
#include <span>
#include <stdexcept>
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/field_decoder.hpp"
#include "cloudini_lib/field_encoder.hpp"

#if defined(__GNUC__)
#define CLOUDINI_FORCE_INLINE inline __attribute__((always_inline))
#else
#define CLOUDINI_FORCE_INLINE inline
#endif

namespace Cloudini::detail {

constexpr size_t kPointsPerChunk = 32 * 1024;

// Scratch array that grows without zero-filling: its contents are always written before they are read,
// and are not kept when it grows.
template <typename T>
class ScratchArray {
 public:
  T* data() {
    return data_.get();
  }
  // Room for `count` elements; returns data().
  T* ensure(size_t count) {
    if (count > size_) {
      data_ = std::make_unique_for_overwrite<T[]>(count);
      size_ = count;
    }
    return data_.get();
  }

 private:
  std::unique_ptr<T[]> data_;
  size_t size_ = 0;
};

size_t MaxSerializedFieldSize(const PointField& field, EncodingOptions encoding_opt);
size_t MaxSerializedPointSize(const EncodingInfo& info);

size_t LeadingLossyFloatFieldCount(const EncodingInfo& info);

size_t AppendLeadingLossyFloatEncoder(const EncodingInfo& info, std::vector<std::unique_ptr<FieldEncoder>>& encoders);
size_t AppendLeadingLossyFloatDecoder(const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders);

std::unique_ptr<FieldEncoder> CreateCompatibleEncoder(const EncodingInfo& info, const PointField& field);
std::unique_ptr<FieldDecoder> CreateCompatibleDecoder(const EncodingInfo& info, const PointField& field);

void ResetEncoders(std::vector<std::unique_ptr<FieldEncoder>>& encoders);
void ResetDecoders(std::vector<std::unique_ptr<FieldDecoder>>& decoders);

// Decodes `count` points of a chunk whose per-point fields are interleaved (V4, V5). With several decoders,
// the points are decoded with FieldDecoder::decodeUnchecked() while the input holds a longest possible
// point, and the rest with the checked decode(). `output` must hold count * point_step bytes.
void DecodePoints(
    std::vector<std::unique_ptr<FieldDecoder>>& decoders, ConstBufferView& input, uint8_t* output, size_t point_step,
    size_t count);
size_t FlushEncoders(std::vector<std::unique_ptr<FieldEncoder>>& encoders, BufferView& output);

// Worst-case size of CompressChunk() output for `input_size` bytes of input.
size_t CompressBound(CompressionOption compression, size_t input_size);

// `block_starts` (increasing offsets into `input`) are positions where stage 2 should begin a new
// compressed block, e.g. where a V5 adaptive section starts. They are a hint: the compressed stream stays a
// single standard LZ4/ZSTD payload, so decoding does not depend on them.
uint32_t CompressChunk(
    CompressionOption compression, ConstBufferView input, BufferView& output,
    std::span<const size_t> block_starts = {});

// Stage 2 in reverse. The view is `chunk_data` itself without compression. Otherwise it is a buffer of this
// thread, shared by its decoders and never zero-filled, that stays valid until the next call on the thread;
// or `large_buffer`, for more than kThreadScratchBytes (the thread keeps its buffer, so it stays small).
constexpr size_t kThreadScratchBytes = size_t{8} << 20;

// Test only: false makes the decoders skip their SIMD block paths, so that a test can compare them with the
// value-by-value ones. Change it only while no decode runs.
extern bool g_block_decoders;
ConstBufferView DecompressChunk(
    CompressionOption compression, ConstBufferView chunk_data, std::vector<uint8_t>& large_buffer,
    size_t max_decompressed_size);

// Byte writers and readers of the V5 and V6 stage-1 sections.
inline void appendByte(BufferView& out, uint8_t value) {
  if (out.empty()) {
    throw std::runtime_error("stage 1: output buffer full");
  }
  out.data()[0] = value;
  out.trim_front(1);
}

inline void appendByte(std::vector<uint8_t>& out, uint8_t value) {
  out.push_back(value);
}

inline void appendUVarint(uint64_t value, BufferView& out) {
  while (value > 0x7Fu) {
    appendByte(out, static_cast<uint8_t>((value & 0x7Fu) | 0x80u));
    value >>= 7u;
  }
  appendByte(out, static_cast<uint8_t>(value));
}

inline void appendUVarint(uint64_t value, std::vector<uint8_t>& out) {
  while (value > 0x7Fu) {
    appendByte(out, static_cast<uint8_t>((value & 0x7Fu) | 0x80u));
    value >>= 7u;
  }
  appendByte(out, static_cast<uint8_t>(value));
}

inline uint64_t readUVarint(ConstBufferView& input) {
  uint64_t value = 0;
  uint8_t shift = 0;
  while (true) {
    if (input.empty()) {
      throw std::runtime_error("stage 1: truncated unsigned varint");
    }
    const uint8_t byte = input.data()[0];
    input.trim_front(1);
    value |= (static_cast<uint64_t>(byte & 0x7Fu) << shift);
    if ((byte & 0x80u) == 0) {
      return value;
    }
    shift = static_cast<uint8_t>(shift + 7u);
    if (shift >= 64) {
      throw std::runtime_error("stage 1: unsigned varint overflow");
    }
  }
}

}  // namespace Cloudini::detail
