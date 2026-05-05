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

#include "v5_codec.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "codec_common.hpp"
#include "cloudini_lib/encoding_utils.hpp"
#include "cloudini_lib/field_encoder.hpp"

namespace Cloudini::detail {
namespace {

// V5 adaptive integer wire mode ids. These values are part of the V5 chunk
// format and must stay stable once V5 is released.
enum class AdaptiveIntMode : uint8_t {
  DeltaVarint = 0,
  Palette = 1,
  Rle = 2,
  DeltaRle = 3,
};

struct V5AdaptiveIntField {
  size_t field_index = 0;
  std::string name;
  FieldType type = FieldType::UNKNOWN;
  uint32_t offset = 0;
  size_t bytes_per_value = 0;
  std::vector<int64_t> values;
  std::vector<uint64_t> raw_values;
  std::vector<uint64_t> palette;
  std::vector<uint32_t> palette_indexes;
  std::vector<uint32_t> palette_slots;
  std::vector<uint32_t> palette_slot_generations;
  uint32_t palette_generation = 1;
  bool committed = false;
  AdaptiveIntMode committed_mode = AdaptiveIntMode::DeltaVarint;

  std::vector<uint8_t> section_bytes;
  bool streaming_section = false;
  size_t run_count_offset = 0;
  uint32_t stream_run_count = 0;
  int64_t stream_prev_value = 0;
  int64_t stream_run_diff = 0;
  uint64_t stream_run_raw = 0;
  uint64_t stream_run_len = 0;
  bool stream_has_run = false;
};

struct V5AdaptiveIntStats {
  size_t delta_bytes = 0;
  size_t palette_bytes = 0;
  size_t rle_bytes = 0;
  size_t delta_rle_bytes = 0;
  uint32_t rle_runs = 0;
  uint32_t delta_rle_runs = 0;
};

constexpr size_t kAdaptiveModeProbePoints = 4096;

struct V5EncoderPlan {
  std::vector<std::unique_ptr<FieldEncoder>> regular;
  std::vector<V5AdaptiveIntField> adaptive;
};

bool isV5AdaptiveIntType(FieldType type) {
  switch (type) {
    case FieldType::INT16:
    case FieldType::UINT16:
    case FieldType::INT32:
    case FieldType::UINT32:
    case FieldType::INT64:
    case FieldType::UINT64:
      return true;
    default:
      return false;
  }
}

int64_t readIntAsI64(const uint8_t* ptr, FieldType type) {
  switch (type) {
    case FieldType::INT16:
      return ToInt64<int16_t>(ptr);
    case FieldType::UINT16:
      return ToInt64<uint16_t>(ptr);
    case FieldType::INT32:
      return ToInt64<int32_t>(ptr);
    case FieldType::UINT32:
      return ToInt64<uint32_t>(ptr);
    case FieldType::INT64:
      return ToInt64<int64_t>(ptr);
    case FieldType::UINT64:
      return static_cast<int64_t>(ToInt64<uint64_t>(ptr));
    default:
      throw std::runtime_error("V5 adaptive int called on non-integer field");
  }
}

uint64_t readRawBits(const uint8_t* ptr, size_t bytes) {
  uint64_t out = 0;
  std::memcpy(&out, ptr, bytes);
  return out;
}

void writeRawBitsToPoint(uint64_t value, size_t bytes, uint8_t* dst) {
  std::memcpy(dst, &value, bytes);
}

void appendByte(BufferView& out, uint8_t value) {
  if (out.empty()) {
    throw std::runtime_error("V5 adaptive int: output buffer full");
  }
  out.data()[0] = value;
  out.trim_front(1);
}

void appendByte(std::vector<uint8_t>& out, uint8_t value) {
  out.push_back(value);
}

void appendRawBits(BufferView& out, uint64_t value, size_t bytes) {
  if (out.size() < bytes) {
    throw std::runtime_error("V5 adaptive int: output buffer full");
  }
  std::memcpy(out.data(), &value, bytes);
  out.trim_front(bytes);
}

void appendRawBits(std::vector<uint8_t>& out, uint64_t value, size_t bytes) {
  const size_t offset = out.size();
  out.resize(offset + bytes);
  std::memcpy(out.data() + offset, &value, bytes);
}

void appendU32(std::vector<uint8_t>& out, uint32_t value) {
  appendRawBits(out, value, sizeof(value));
}

void patchU32(std::vector<uint8_t>& out, size_t offset, uint32_t value) {
  std::memcpy(out.data() + offset, &value, sizeof(value));
}

void appendUVarint(uint64_t value, BufferView& out) {
  while (value > 0x7Fu) {
    appendByte(out, static_cast<uint8_t>((value & 0x7Fu) | 0x80u));
    value >>= 7u;
  }
  appendByte(out, static_cast<uint8_t>(value));
}

void appendUVarint(uint64_t value, std::vector<uint8_t>& out) {
  while (value > 0x7Fu) {
    appendByte(out, static_cast<uint8_t>((value & 0x7Fu) | 0x80u));
    value >>= 7u;
  }
  appendByte(out, static_cast<uint8_t>(value));
}

uint64_t readUVarint(ConstBufferView& input) {
  uint64_t value = 0;
  uint8_t shift = 0;
  while (true) {
    if (input.empty()) {
      throw std::runtime_error("V5 adaptive int: truncated unsigned varint");
    }
    const uint8_t byte = input.data()[0];
    input.trim_front(1);
    value |= (static_cast<uint64_t>(byte & 0x7Fu) << shift);
    if ((byte & 0x80u) == 0) {
      return value;
    }
    shift = static_cast<uint8_t>(shift + 7u);
    if (shift >= 64) {
      throw std::runtime_error("V5 adaptive int: unsigned varint overflow");
    }
  }
}

uint8_t bitsForPaletteIndex(size_t unique_count) {
  if (unique_count <= 1) {
    return 0;
  }
  uint8_t bits = 0;
  size_t max_index = unique_count - 1;
  while (max_index > 0) {
    ++bits;
    max_index >>= 1u;
  }
  return bits;
}

void appendBitpackedIndexes(const std::vector<uint32_t>& indexes, uint8_t bits, BufferView& out) {
  if (bits == 0) {
    return;
  }
  uint64_t scratch = 0;
  uint8_t held = 0;
  for (uint32_t index : indexes) {
    scratch |= (static_cast<uint64_t>(index) << held);
    held = static_cast<uint8_t>(held + bits);
    while (held >= 8) {
      appendByte(out, static_cast<uint8_t>(scratch & 0xFFu));
      scratch >>= 8u;
      held = static_cast<uint8_t>(held - 8u);
    }
  }
  if (held > 0) {
    appendByte(out, static_cast<uint8_t>(scratch & 0xFFu));
  }
}

uint32_t readBitpackedIndex(const uint8_t*& ptr, uint64_t& scratch, uint8_t& held, uint8_t bits) {
  if (bits == 0) {
    return 0;
  }
  while (held < bits) {
    scratch |= (static_cast<uint64_t>(*ptr++) << held);
    held = static_cast<uint8_t>(held + 8u);
  }
  const uint64_t mask = (uint64_t{1} << bits) - 1u;
  const uint32_t out = static_cast<uint32_t>(scratch & mask);
  scratch >>= bits;
  held = static_cast<uint8_t>(held - bits);
  return out;
}

size_t encodedUVarintSize(uint64_t value) {
  size_t bytes = 1;
  while (value > 0x7Fu) {
    value >>= 7u;
    ++bytes;
  }
  return bytes;
}

size_t encodedVarint64Size(int64_t value) {
  uint8_t tmp[10];
  return encodeVarint64(value, tmp);
}

size_t encodedDeltaVarintSectionSize(const std::vector<int64_t>& values) {
  size_t bytes = 1;  // mode byte
  int64_t prev = 0;
  for (int64_t value : values) {
    const int64_t diff = value - prev;
    prev = value;
    bytes += encodedVarint64Size(diff);
  }
  return bytes;
}

template <typename Callback>
void forEachDeltaRun(const std::vector<int64_t>& values, Callback callback) {
  int64_t prev = 0;
  size_t i = 0;
  while (i < values.size()) {
    const int64_t diff = values[i] - prev;
    prev = values[i];
    size_t j = i + 1;
    while (j < values.size()) {
      const int64_t next_diff = values[j] - prev;
      if (next_diff != diff) {
        break;
      }
      prev = values[j];
      ++j;
    }
    callback(diff, j - i);
    i = j;
  }
}

size_t encodedDeltaRleSectionSize(const std::vector<int64_t>& values, uint32_t& run_count) {
  size_t bytes = 1 + sizeof(uint32_t);  // mode byte + run count
  run_count = 0;
  forEachDeltaRun(values, [&](int64_t diff, size_t run_len) {
    bytes += encodedVarint64Size(diff) + encodedUVarintSize(run_len);
    ++run_count;
  });
  return bytes;
}

size_t encodedRleSectionSize(
    const std::vector<uint64_t>& raw_values, size_t bytes_per_value, uint32_t& run_count) {
  size_t bytes = 1 + sizeof(uint32_t);  // mode byte + run count
  run_count = 0;
  size_t i = 0;
  while (i < raw_values.size()) {
    const uint64_t value = raw_values[i];
    size_t j = i + 1;
    while (j < raw_values.size() && raw_values[j] == value) {
      ++j;
    }
    bytes += bytes_per_value + encodedUVarintSize(j - i);
    ++run_count;
    i = j;
  }
  return bytes;
}

size_t nextPowerOfTwo(size_t value) {
  size_t out = 1;
  while (out < value) {
    out <<= 1u;
  }
  return out;
}

size_t hashPaletteValue(uint64_t value) {
  value ^= value >> 30u;
  value *= 0xbf58476d1ce4e5b9ULL;
  value ^= value >> 27u;
  value *= 0x94d049bb133111ebULL;
  value ^= value >> 31u;
  return static_cast<size_t>(value);
}

void preparePaletteTable(V5AdaptiveIntField& field, size_t value_count) {
  const size_t slot_count = nextPowerOfTwo(std::max<size_t>(16, value_count * 2u));
  if (field.palette_slots.size() < slot_count) {
    field.palette_slots.assign(slot_count, 0);
    field.palette_slot_generations.assign(slot_count, 0);
  } else {
    ++field.palette_generation;
    if (field.palette_generation == 0) {
      std::fill(field.palette_slot_generations.begin(), field.palette_slot_generations.end(), 0);
      field.palette_generation = 1;
    }
  }
}

uint32_t addPaletteValue(V5AdaptiveIntField& field, uint64_t value) {
  const size_t mask = field.palette_slots.size() - 1u;
  size_t slot = hashPaletteValue(value) & mask;
  while (true) {
    if (field.palette_slot_generations[slot] != field.palette_generation) {
      const uint32_t index = static_cast<uint32_t>(field.palette.size());
      field.palette.push_back(value);
      field.palette_slots[slot] = index + 1u;
      field.palette_slot_generations[slot] = field.palette_generation;
      return index;
    }

    const uint32_t index = field.palette_slots[slot] - 1u;
    if (field.palette[index] == value) {
      return index;
    }
    slot = (slot + 1u) & mask;
  }
}

void buildPaletteIndexes(V5AdaptiveIntField& field) {
  field.palette.clear();
  field.palette_indexes.clear();
  field.palette.reserve(field.raw_values.size());
  field.palette_indexes.reserve(field.raw_values.size());
  preparePaletteTable(field, field.raw_values.size());

  for (uint64_t value : field.raw_values) {
    field.palette_indexes.push_back(addPaletteValue(field, value));
  }
}

size_t encodedPaletteSectionSize(const V5AdaptiveIntField& field) {
  const uint8_t bits = bitsForPaletteIndex(field.palette.size());
  return 1 + sizeof(uint16_t) + field.palette.size() * field.bytes_per_value +
         (static_cast<size_t>(bits) * field.raw_values.size() + 7u) / 8u;
}

AdaptiveIntMode selectBestAdaptiveIntMode(const V5AdaptiveIntStats& stats) {
  AdaptiveIntMode best_mode = AdaptiveIntMode::DeltaVarint;
  size_t best_size = stats.delta_bytes;
  if (stats.palette_bytes < best_size) {
    best_size = stats.palette_bytes;
    best_mode = AdaptiveIntMode::Palette;
  }
  if (stats.rle_bytes < best_size) {
    best_mode = AdaptiveIntMode::Rle;
    best_size = stats.rle_bytes;
  }
  if (stats.delta_rle_bytes < best_size) {
    best_mode = AdaptiveIntMode::DeltaRle;
  }
  return best_mode;
}

V5AdaptiveIntStats analyzeAdaptiveIntField(V5AdaptiveIntField& field) {
  V5AdaptiveIntStats stats;
  stats.delta_bytes = encodedDeltaVarintSectionSize(field.values);
  buildPaletteIndexes(field);
  stats.palette_bytes = encodedPaletteSectionSize(field);
  stats.rle_bytes = encodedRleSectionSize(field.raw_values, field.bytes_per_value, stats.rle_runs);
  stats.delta_rle_bytes = encodedDeltaRleSectionSize(field.values, stats.delta_rle_runs);
  return stats;
}

void commitAdaptiveIntMode(V5AdaptiveIntField& field) {
  if (field.committed) {
    return;
  }
  const V5AdaptiveIntStats stats = analyzeAdaptiveIntField(field);
  field.committed_mode = selectBestAdaptiveIntMode(stats);
  field.committed = true;
}

void appendDeltaVarintSection(const std::vector<int64_t>& values, BufferView& out) {
  appendByte(out, static_cast<uint8_t>(AdaptiveIntMode::DeltaVarint));
  int64_t prev = 0;
  for (int64_t value : values) {
    const int64_t diff = value - prev;
    prev = value;
    const size_t bytes = encodeVarint64(diff, out.data());
    out.trim_front(bytes);
  }
}

void appendVarint64(int64_t value, BufferView& out) {
  const size_t bytes = encodeVarint64(value, out.data());
  out.trim_front(bytes);
}

void appendVarint64(int64_t value, std::vector<uint8_t>& out) {
  uint8_t bytes[10];
  const size_t count = encodeVarint64(value, bytes);
  const size_t offset = out.size();
  out.resize(offset + count);
  std::memcpy(out.data() + offset, bytes, count);
}

void appendDeltaRleSection(const std::vector<int64_t>& values, BufferView& out) {
  appendByte(out, static_cast<uint8_t>(AdaptiveIntMode::DeltaRle));
  uint8_t* run_count_ptr = out.data();
  encode(uint32_t{0}, out);

  uint32_t run_count = 0;
  forEachDeltaRun(values, [&](int64_t diff, size_t run_len) {
    appendVarint64(diff, out);
    appendUVarint(run_len, out);
    ++run_count;
  });

  std::memcpy(run_count_ptr, &run_count, sizeof(run_count));
}

void appendPaletteSection(const V5AdaptiveIntField& field, BufferView& out) {
  appendByte(out, static_cast<uint8_t>(AdaptiveIntMode::Palette));
  encode(static_cast<uint16_t>(field.palette.size()), out);
  for (uint64_t value : field.palette) {
    appendRawBits(out, value, field.bytes_per_value);
  }
  appendBitpackedIndexes(field.palette_indexes, bitsForPaletteIndex(field.palette.size()), out);
}

void appendRleSection(const std::vector<uint64_t>& raw_values, size_t bytes_per_value, BufferView& out) {
  appendByte(out, static_cast<uint8_t>(AdaptiveIntMode::Rle));
  uint8_t* run_count_ptr = out.data();
  encode(uint32_t{0}, out);

  uint32_t run_count = 0;
  size_t i = 0;
  while (i < raw_values.size()) {
    const uint64_t value = raw_values[i];
    size_t j = i + 1;
    while (j < raw_values.size() && raw_values[j] == value) {
      ++j;
    }
    appendRawBits(out, value, bytes_per_value);
    appendUVarint(j - i, out);
    ++run_count;
    i = j;
  }

  std::memcpy(run_count_ptr, &run_count, sizeof(run_count));
}

void beginCommittedAdaptiveIntSection(V5AdaptiveIntField& field, size_t points_in_chunk) {
  field.section_bytes.clear();
  field.streaming_section = false;
  field.run_count_offset = 0;
  field.stream_run_count = 0;
  field.stream_prev_value = 0;
  field.stream_run_diff = 0;
  field.stream_run_raw = 0;
  field.stream_run_len = 0;
  field.stream_has_run = false;

  switch (field.committed_mode) {
    case AdaptiveIntMode::DeltaVarint:
      field.streaming_section = true;
      field.section_bytes.reserve(1u + points_in_chunk * 10u);
      appendByte(field.section_bytes, static_cast<uint8_t>(AdaptiveIntMode::DeltaVarint));
      break;
    case AdaptiveIntMode::DeltaRle:
      field.streaming_section = true;
      field.section_bytes.reserve(1u + sizeof(uint32_t) + points_in_chunk * 11u);
      appendByte(field.section_bytes, static_cast<uint8_t>(AdaptiveIntMode::DeltaRle));
      field.run_count_offset = field.section_bytes.size();
      appendU32(field.section_bytes, 0);
      break;
    case AdaptiveIntMode::Rle:
      field.streaming_section = true;
      field.section_bytes.reserve(1u + sizeof(uint32_t) + points_in_chunk * (field.bytes_per_value + 10u));
      appendByte(field.section_bytes, static_cast<uint8_t>(AdaptiveIntMode::Rle));
      field.run_count_offset = field.section_bytes.size();
      appendU32(field.section_bytes, 0);
      break;
    case AdaptiveIntMode::Palette:
      break;
  }
}

void flushDeltaRleRun(V5AdaptiveIntField& field) {
  if (!field.stream_has_run) {
    return;
  }
  appendVarint64(field.stream_run_diff, field.section_bytes);
  appendUVarint(field.stream_run_len, field.section_bytes);
  ++field.stream_run_count;
  field.stream_has_run = false;
}

void appendDeltaRleValue(V5AdaptiveIntField& field, int64_t value) {
  const int64_t diff = value - field.stream_prev_value;
  field.stream_prev_value = value;
  if (!field.stream_has_run) {
    field.stream_run_diff = diff;
    field.stream_run_len = 1;
    field.stream_has_run = true;
    return;
  }
  if (diff == field.stream_run_diff) {
    ++field.stream_run_len;
    return;
  }
  flushDeltaRleRun(field);
  field.stream_run_diff = diff;
  field.stream_run_len = 1;
  field.stream_has_run = true;
}

void flushRleRun(V5AdaptiveIntField& field) {
  if (!field.stream_has_run) {
    return;
  }
  appendRawBits(field.section_bytes, field.stream_run_raw, field.bytes_per_value);
  appendUVarint(field.stream_run_len, field.section_bytes);
  ++field.stream_run_count;
  field.stream_has_run = false;
}

void appendRleValue(V5AdaptiveIntField& field, uint64_t value) {
  if (!field.stream_has_run) {
    field.stream_run_raw = value;
    field.stream_run_len = 1;
    field.stream_has_run = true;
    return;
  }
  if (value == field.stream_run_raw) {
    ++field.stream_run_len;
    return;
  }
  flushRleRun(field);
  field.stream_run_raw = value;
  field.stream_run_len = 1;
  field.stream_has_run = true;
}

void appendCommittedValueToSection(V5AdaptiveIntField& field, const uint8_t* field_ptr) {
  switch (field.committed_mode) {
    case AdaptiveIntMode::DeltaVarint: {
      const int64_t value = readIntAsI64(field_ptr, field.type);
      const int64_t diff = value - field.stream_prev_value;
      field.stream_prev_value = value;
      appendVarint64(diff, field.section_bytes);
    } break;
    case AdaptiveIntMode::DeltaRle:
      appendDeltaRleValue(field, readIntAsI64(field_ptr, field.type));
      break;
    case AdaptiveIntMode::Rle:
      appendRleValue(field, readRawBits(field_ptr, field.bytes_per_value));
      break;
    case AdaptiveIntMode::Palette:
      field.raw_values.push_back(readRawBits(field_ptr, field.bytes_per_value));
      break;
  }
}

void appendCommittedValueToSection(V5AdaptiveIntField& field, size_t index) {
  switch (field.committed_mode) {
    case AdaptiveIntMode::DeltaVarint: {
      const int64_t value = field.values[index];
      const int64_t diff = value - field.stream_prev_value;
      field.stream_prev_value = value;
      appendVarint64(diff, field.section_bytes);
    } break;
    case AdaptiveIntMode::DeltaRle:
      appendDeltaRleValue(field, field.values[index]);
      break;
    case AdaptiveIntMode::Rle:
      appendRleValue(field, field.raw_values[index]);
      break;
    case AdaptiveIntMode::Palette:
      break;
  }
}

void appendProbeValuesToCommittedSection(V5AdaptiveIntField& field) {
  if (!field.streaming_section) {
    return;
  }
  const size_t values_count =
      (field.committed_mode == AdaptiveIntMode::Rle) ? field.raw_values.size() : field.values.size();
  for (size_t i = 0; i < values_count; ++i) {
    appendCommittedValueToSection(field, i);
  }
}

void finishCommittedAdaptiveIntSection(V5AdaptiveIntField& field) {
  switch (field.committed_mode) {
    case AdaptiveIntMode::DeltaVarint:
      break;
    case AdaptiveIntMode::DeltaRle:
      flushDeltaRleRun(field);
      patchU32(field.section_bytes, field.run_count_offset, field.stream_run_count);
      break;
    case AdaptiveIntMode::Rle:
      flushRleRun(field);
      patchU32(field.section_bytes, field.run_count_offset, field.stream_run_count);
      break;
    case AdaptiveIntMode::Palette:
      break;
  }
}

void appendBufferedSection(const V5AdaptiveIntField& field, BufferView& out) {
  if (out.size() < field.section_bytes.size()) {
    throw std::runtime_error("V5 adaptive int: output buffer full");
  }
  std::memcpy(out.data(), field.section_bytes.data(), field.section_bytes.size());
  out.trim_front(field.section_bytes.size());
}

void prepareAdaptiveIntFieldForChunk(V5AdaptiveIntField& field, size_t points_in_chunk) {
  field.values.clear();
  field.raw_values.clear();
  field.palette.clear();
  field.palette_indexes.clear();
  field.section_bytes.clear();
  field.streaming_section = false;

  if (field.committed) {
    beginCommittedAdaptiveIntSection(field, points_in_chunk);
  }

  if (!field.committed) {
    field.values.reserve(points_in_chunk);
    field.raw_values.reserve(points_in_chunk);
  } else if (field.committed_mode == AdaptiveIntMode::Palette) {
    field.raw_values.reserve(points_in_chunk);
  }
}

void collectAdaptiveIntValue(V5AdaptiveIntField& field, const uint8_t* field_ptr) {
  if (!field.committed) {
    field.values.push_back(readIntAsI64(field_ptr, field.type));
    field.raw_values.push_back(readRawBits(field_ptr, field.bytes_per_value));
    return;
  }

  appendCommittedValueToSection(field, field_ptr);
}

void appendCommittedAdaptiveIntSection(V5AdaptiveIntField& field, BufferView& out) {
  if (!field.committed) {
    commitAdaptiveIntMode(field);
  } else if (field.streaming_section) {
    finishCommittedAdaptiveIntSection(field);
    appendBufferedSection(field, out);
    return;
  }

  if (field.committed_mode == AdaptiveIntMode::Palette) {
    buildPaletteIndexes(field);
  }

  switch (field.committed_mode) {
    case AdaptiveIntMode::DeltaVarint:
      appendDeltaVarintSection(field.values, out);
      break;
    case AdaptiveIntMode::DeltaRle:
      appendDeltaRleSection(field.values, out);
      break;
    case AdaptiveIntMode::Palette:
      appendPaletteSection(field, out);
      break;
    case AdaptiveIntMode::Rle:
      appendRleSection(field.raw_values, field.bytes_per_value, out);
      break;
  }
}

V5EncoderPlan buildV5Plan(const EncodingInfo& info, size_t points_in_chunk) {
  V5EncoderPlan plan;

  const size_t start_index = AppendLeadingLossyFloatEncoder(info, plan.regular);
  for (size_t i = start_index; i < info.fields.size(); ++i) {
    const auto& field = info.fields[i];
    if (info.encoding_opt == EncodingOptions::LOSSY && isV5AdaptiveIntType(field.type)) {
      V5AdaptiveIntField adaptive;
      adaptive.field_index = i;
      adaptive.name = field.name;
      adaptive.type = field.type;
      adaptive.offset = field.offset;
      adaptive.bytes_per_value = static_cast<size_t>(SizeOf(field.type));
      adaptive.values.reserve(points_in_chunk);
      adaptive.raw_values.reserve(points_in_chunk);
      plan.adaptive.push_back(std::move(adaptive));
    } else {
      plan.regular.push_back(CreateCompatibleEncoder(info, field));
    }
  }
  return plan;
}

std::vector<V5AdaptiveIntField> getV5AdaptiveFields(const EncodingInfo& info) {
  std::vector<V5AdaptiveIntField> fields;
  if (info.encoding_opt != EncodingOptions::LOSSY) {
    return fields;
  }

  const size_t start_index = LeadingLossyFloatFieldCount(info);
  for (size_t i = start_index; i < info.fields.size(); ++i) {
    const auto& field = info.fields[i];
    if (isV5AdaptiveIntType(field.type)) {
      V5AdaptiveIntField adaptive;
      adaptive.field_index = i;
      adaptive.name = field.name;
      adaptive.type = field.type;
      adaptive.offset = field.offset;
      adaptive.bytes_per_value = static_cast<size_t>(SizeOf(field.type));
      fields.push_back(std::move(adaptive));
    }
  }
  return fields;
}

void decodeV5AdaptiveIntSection(
    const V5AdaptiveIntField& field, ConstBufferView& input, uint8_t* output_base,
    size_t point_step, size_t expected_points) {
  if (input.empty()) {
    throw std::runtime_error("V5 adaptive int: missing mode byte");
  }
  const uint8_t mode_byte = input.data()[0];
  input.trim_front(1);
  if (mode_byte > static_cast<uint8_t>(AdaptiveIntMode::DeltaRle)) {
    throw std::runtime_error(
        "V5 adaptive int: unknown mode byte " + std::to_string(static_cast<int>(mode_byte)));
  }
  const auto mode = static_cast<AdaptiveIntMode>(mode_byte);

  switch (mode) {
    case AdaptiveIntMode::DeltaVarint: {
      int64_t prev = 0;
      for (size_t i = 0; i < expected_points; ++i) {
        int64_t diff = 0;
        const auto consumed = decodeVarint(input.data(), input.size(), diff);
        input.trim_front(consumed);
        const int64_t value = prev + diff;
        prev = value;
        writeRawBitsToPoint(
            static_cast<uint64_t>(value), field.bytes_per_value,
            output_base + i * point_step + field.offset);
      }
    } break;

    case AdaptiveIntMode::Palette: {
      uint16_t palette_count = 0;
      decode(input, palette_count);
      if (palette_count == 0) {
        throw std::runtime_error("V5 adaptive int: empty palette");
      }
      std::vector<uint64_t> palette(palette_count, 0);
      for (uint64_t& value : palette) {
        if (input.size() < field.bytes_per_value) {
          throw std::runtime_error("V5 adaptive int: truncated palette");
        }
        value = readRawBits(input.data(), field.bytes_per_value);
        input.trim_front(field.bytes_per_value);
      }
      const uint8_t bits = bitsForPaletteIndex(palette_count);
      const size_t index_bytes = (static_cast<size_t>(bits) * expected_points + 7u) / 8u;
      if (input.size() < index_bytes) {
        throw std::runtime_error("V5 adaptive int: truncated palette indexes");
      }
      const uint8_t* index_ptr = input.data();
      uint64_t scratch = 0;
      uint8_t held = 0;
      for (size_t i = 0; i < expected_points; ++i) {
        const uint32_t idx = readBitpackedIndex(index_ptr, scratch, held, bits);
        if (idx >= palette.size()) {
          throw std::runtime_error("V5 adaptive int: palette index out of range");
        }
        writeRawBitsToPoint(palette[idx], field.bytes_per_value, output_base + i * point_step + field.offset);
      }
      input.trim_front(index_bytes);
    } break;

    case AdaptiveIntMode::Rle: {
      uint32_t run_count = 0;
      decode(input, run_count);
      size_t out_index = 0;
      for (uint32_t r = 0; r < run_count; ++r) {
        if (input.size() < field.bytes_per_value) {
          throw std::runtime_error("V5 adaptive int: truncated RLE value");
        }
        const uint64_t value = readRawBits(input.data(), field.bytes_per_value);
        input.trim_front(field.bytes_per_value);
        const uint64_t run_len = readUVarint(input);
        if (out_index + run_len > expected_points) {
          throw std::runtime_error("V5 adaptive int: RLE run exceeds point count");
        }
        for (uint64_t k = 0; k < run_len; ++k) {
          writeRawBitsToPoint(value, field.bytes_per_value, output_base + out_index * point_step + field.offset);
          ++out_index;
        }
      }
      if (out_index != expected_points) {
        throw std::runtime_error("V5 adaptive int: RLE run count does not fill chunk");
      }
    } break;

    case AdaptiveIntMode::DeltaRle: {
      uint32_t run_count = 0;
      decode(input, run_count);
      int64_t prev = 0;
      size_t out_index = 0;
      for (uint32_t r = 0; r < run_count; ++r) {
        int64_t diff = 0;
        const auto consumed = decodeVarint(input.data(), input.size(), diff);
        input.trim_front(consumed);
        const uint64_t run_len = readUVarint(input);
        if (out_index + run_len > expected_points) {
          throw std::runtime_error("V5 adaptive int: Delta-RLE run exceeds point count");
        }
        for (uint64_t k = 0; k < run_len; ++k) {
          const int64_t value = prev + diff;
          prev = value;
          writeRawBitsToPoint(
              static_cast<uint64_t>(value), field.bytes_per_value,
              output_base + out_index * point_step + field.offset);
          ++out_index;
        }
      }
      if (out_index != expected_points) {
        throw std::runtime_error("V5 adaptive int: Delta-RLE run count does not fill chunk");
      }
    } break;

    default:
      throw std::runtime_error("V5 adaptive int: unknown mode");
  }
}

}  // namespace

bool UsesV5Codec(const EncodingInfo& info) {
  if (info.version < 5 || info.encoding_opt != EncodingOptions::LOSSY) {
    return false;
  }

  const size_t start_index = LeadingLossyFloatFieldCount(info);
  return std::any_of(
      info.fields.begin() + start_index, info.fields.end(),
      [](const auto& field) { return isV5AdaptiveIntType(field.type); });
}

size_t V5StageBufferSize(const EncodingInfo& info, size_t points_per_chunk) {
  const size_t max_per_point = MaxSerializedPointSize(info);
  return points_per_chunk * (std::max<size_t>(info.point_step, max_per_point) + 64u) +
         info.fields.size() * 64u + 1024u;
}

void EncodeV5Stage1(
    const EncodingInfo& info, ConstBufferView cloud_data, size_t points_count,
    size_t points_per_chunk, const std::function<BufferView()>& get_stage_buffer,
    const std::function<void(size_t serialized_size)>& write_stage1_chunk) {
  V5EncoderPlan plan = buildV5Plan(info, points_per_chunk);

  size_t points_left = points_count;
  size_t point_offset = 0;
  while (points_left > 0) {
    const size_t chunk_points = std::min(points_left, points_per_chunk);
    for (auto& regular : plan.regular) {
      regular->reset();
    }
    for (auto& adaptive : plan.adaptive) {
      prepareAdaptiveIntFieldForChunk(adaptive, chunk_points);
    }

    BufferView stage_buffer = get_stage_buffer();
    BufferView stage_view(stage_buffer.data(), stage_buffer.size());

    auto encode_point_range = [&](size_t first, size_t last) {
      for (size_t i = first; i < last; ++i) {
        const uint8_t* point = cloud_data.data() + (point_offset + i) * info.point_step;
        ConstBufferView point_view(point, info.point_step);
        for (auto& regular : plan.regular) {
          regular->encode(point_view, stage_view);
        }
        for (auto& adaptive : plan.adaptive) {
          const uint8_t* field_ptr = point + adaptive.offset;
          collectAdaptiveIntValue(adaptive, field_ptr);
        }
      }
    };

    const bool has_uncommitted_adaptive =
        std::any_of(plan.adaptive.begin(), plan.adaptive.end(), [](const auto& field) {
          return !field.committed;
        });

    if (has_uncommitted_adaptive && chunk_points > kAdaptiveModeProbePoints) {
      encode_point_range(0, kAdaptiveModeProbePoints);
      for (auto& adaptive : plan.adaptive) {
        commitAdaptiveIntMode(adaptive);
        beginCommittedAdaptiveIntSection(adaptive, chunk_points);
        appendProbeValuesToCommittedSection(adaptive);
      }
      encode_point_range(kAdaptiveModeProbePoints, chunk_points);
    } else {
      encode_point_range(0, chunk_points);
    }

    for (auto& regular : plan.regular) {
      regular->flush(stage_view);
    }
    for (auto& adaptive : plan.adaptive) {
      appendCommittedAdaptiveIntSection(adaptive, stage_view);
    }

    write_stage1_chunk(stage_buffer.size() - stage_view.size());

    point_offset += chunk_points;
    points_left -= chunk_points;
  }
}

void BuildV5Decoders(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders,
    size_t& min_encoded_point_bytes) {
  decoders.clear();
  min_encoded_point_bytes = 0;

  const size_t start_index = AppendLeadingLossyFloatDecoder(info, decoders);
  for (size_t index = start_index; index < info.fields.size(); ++index) {
    if (isV5AdaptiveIntType(info.fields[index].type)) {
      continue;
    }
    decoders.push_back(CreateCompatibleDecoder(info, info.fields[index]));
  }

  for (const auto& decoder : decoders) {
    min_encoded_point_bytes += decoder->minInputBytes();
  }
}

void DecodeV5Stage1Chunk(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders,
    ConstBufferView& encoded_view, BufferView& output_buffer, size_t expected_points) {
  if (expected_points == 0) {
    throw std::runtime_error("V5 chunks require an expected point count");
  }
  const size_t output_bytes = expected_points * info.point_step;
  if (output_buffer.size() < output_bytes) {
    throw std::runtime_error("Output buffer is too small to hold the decoded V5 data");
  }

  ResetDecoders(decoders);
  uint8_t* chunk_output = output_buffer.data();
  for (size_t p = 0; p < expected_points; ++p) {
    BufferView point_view(chunk_output + p * info.point_step, info.point_step);
    for (auto& decoder : decoders) {
      decoder->decode(encoded_view, point_view);
    }
  }

  const std::vector<V5AdaptiveIntField> adaptive_fields = getV5AdaptiveFields(info);
  for (const auto& field : adaptive_fields) {
    decodeV5AdaptiveIntSection(field, encoded_view, chunk_output, info.point_step, expected_points);
  }
  if (!encoded_view.empty()) {
    throw std::runtime_error("V5 chunk has trailing bytes after decode");
  }
  output_buffer.trim_front(output_bytes);
}

}  // namespace Cloudini::detail
