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
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "cloudini_lib/encoding_utils.hpp"
#include "cloudini_lib/field_encoder.hpp"
#include "codec_common.hpp"

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

size_t encodedRleSectionSize(const std::vector<uint64_t>& raw_values, size_t bytes_per_value, uint32_t& run_count) {
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

// Palette mode needs the indexes from buildPaletteIndexes().
void appendAdaptiveIntSection(const V5AdaptiveIntField& field, AdaptiveIntMode mode, BufferView& out) {
  switch (mode) {
    case AdaptiveIntMode::DeltaVarint:
      appendDeltaVarintSection(field.values, out);
      break;
    case AdaptiveIntMode::Palette:
      appendPaletteSection(field, out);
      break;
    case AdaptiveIntMode::Rle:
      appendRleSection(field.raw_values, field.bytes_per_value, out);
      break;
    case AdaptiveIntMode::DeltaRle:
      appendDeltaRleSection(field.values, out);
      break;
  }
}

size_t serializeAdaptiveIntSection(
    const V5AdaptiveIntField& field, AdaptiveIntMode mode, size_t section_bytes, std::vector<uint8_t>& section) {
  section.resize(section_bytes);
  BufferView out(section.data(), section.size());
  appendAdaptiveIntSection(field, mode, out);
  return section.size() - out.size();
}

size_t sectionBytes(const V5AdaptiveIntStats& stats, AdaptiveIntMode mode) {
  switch (mode) {
    case AdaptiveIntMode::DeltaVarint:
      return stats.delta_bytes;
    case AdaptiveIntMode::Palette:
      return stats.palette_bytes;
    case AdaptiveIntMode::Rle:
      return stats.rle_bytes;
    case AdaptiveIntMode::DeltaRle:
      return stats.delta_rle_bytes;
  }
  return stats.delta_bytes;
}

// Below this stage-1 size the choice cannot matter much: skip the trial compression.
constexpr size_t kTrialCompressionMinBytes = 64;

// Adaptive sections are compressed by stage 2 together with the rest of the chunk, and the mode that is
// smallest before compression is not always the smallest after it. A palette stores every distinct value
// raw: for per-column timestamps (1024 distinct values per row) that table barely compresses, while
// delta-varint stores small deltas that repeat row after row. So, when stage 2 is enabled and the
// stage-1 winner is not DeltaVarint, both are serialized for the probe values and the one that
// compresses better is kept.
AdaptiveIntMode selectAdaptiveIntMode(
    V5AdaptiveIntField& field, const V5AdaptiveIntStats& stats, CompressionOption compression) {
  const AdaptiveIntMode stage1_best = selectBestAdaptiveIntMode(stats);
  if (compression == CompressionOption::NONE || stage1_best == AdaptiveIntMode::DeltaVarint ||
      sectionBytes(stats, stage1_best) <= kTrialCompressionMinBytes) {
    return stage1_best;
  }

  std::vector<uint8_t> section;
  std::vector<uint8_t> compressed;
  auto compressed_size = [&](AdaptiveIntMode mode) {
    const size_t section_bytes = serializeAdaptiveIntSection(field, mode, sectionBytes(stats, mode), section);
    compressed.resize(CompressBound(compression, section_bytes));
    BufferView compressed_view(compressed.data(), compressed.size());
    return static_cast<size_t>(
        CompressChunk(compression, ConstBufferView(section.data(), section_bytes), compressed_view));
  };
  return compressed_size(AdaptiveIntMode::DeltaVarint) < compressed_size(stage1_best) ? AdaptiveIntMode::DeltaVarint
                                                                                      : stage1_best;
}

void commitAdaptiveIntMode(V5AdaptiveIntField& field, CompressionOption compression) {
  if (field.committed) {
    return;
  }
  const V5AdaptiveIntStats stats = analyzeAdaptiveIntField(field);
  field.committed_mode = selectAdaptiveIntMode(field, stats, compression);
  field.committed = true;
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

void appendCommittedAdaptiveIntSection(V5AdaptiveIntField& field, CompressionOption compression, BufferView& out) {
  if (!field.committed) {
    commitAdaptiveIntMode(field, compression);
  } else if (field.streaming_section) {
    finishCommittedAdaptiveIntSection(field);
    appendBufferedSection(field, out);
    return;
  }

  if (field.committed_mode == AdaptiveIntMode::Palette) {
    buildPaletteIndexes(field);
  }

  appendAdaptiveIntSection(field, field.committed_mode, out);
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

template <size_t Bytes>
void writeValueToPoint(uint64_t value, uint8_t* dst) {
  std::memcpy(dst, &value, Bytes);
}

template <size_t Bytes>
void decodeV5AdaptiveIntValues(
    const V5AdaptiveIntField& field, AdaptiveIntMode mode, ConstBufferView& input, uint8_t* output_base,
    size_t point_step, size_t expected_points) {
  switch (mode) {
    case AdaptiveIntMode::DeltaVarint: {
      uint64_t prev = 0;
      size_t i = 0;
      // While a longest-possible varint is readable, skip the per-byte bounds checks.
      const uint8_t* ptr = input.data();
      const uint8_t* const end = input.data() + input.size();
      for (; i < expected_points && static_cast<size_t>(end - ptr) >= kMaxVarintBytes; ++i) {
        int64_t diff = 0;
        ptr += decodeVarintUnchecked(ptr, diff);
        prev += static_cast<uint64_t>(diff);
        writeValueToPoint<Bytes>(prev, output_base + i * point_step + field.offset);
      }
      input.trim_front(static_cast<size_t>(ptr - input.data()));
      for (; i < expected_points; ++i) {
        int64_t diff = 0;
        const auto consumed = decodeVarint(input.data(), input.size(), diff);
        input.trim_front(consumed);
        prev += static_cast<uint64_t>(diff);
        writeValueToPoint<Bytes>(prev, output_base + i * point_step + field.offset);
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
        writeValueToPoint<Bytes>(palette[idx], output_base + i * point_step + field.offset);
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
          writeValueToPoint<Bytes>(value, output_base + out_index * point_step + field.offset);
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
      uint64_t prev = 0;
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
          prev += static_cast<uint64_t>(diff);
          writeValueToPoint<Bytes>(prev, output_base + out_index * point_step + field.offset);
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

void decodeV5AdaptiveIntSection(
    const V5AdaptiveIntField& field, ConstBufferView& input, uint8_t* output_base, size_t point_step,
    size_t expected_points) {
  if (input.empty()) {
    throw std::runtime_error("V5 adaptive int: missing mode byte");
  }
  const uint8_t mode_byte = input.data()[0];
  input.trim_front(1);
  if (mode_byte > static_cast<uint8_t>(AdaptiveIntMode::DeltaRle)) {
    throw std::runtime_error("V5 adaptive int: unknown mode byte " + std::to_string(static_cast<int>(mode_byte)));
  }
  const auto mode = static_cast<AdaptiveIntMode>(mode_byte);

  switch (field.bytes_per_value) {
    case 2:
      decodeV5AdaptiveIntValues<2>(field, mode, input, output_base, point_step, expected_points);
      break;
    case 4:
      decodeV5AdaptiveIntValues<4>(field, mode, input, output_base, point_step, expected_points);
      break;
    case 8:
      decodeV5AdaptiveIntValues<8>(field, mode, input, output_base, point_step, expected_points);
      break;
    default:
      throw std::runtime_error("V5 adaptive int: unsupported value size");
  }
}

}  // namespace

//==========================================================================================
// V6 (experimental): geometry predicted from the best neighbour, invalid-point mask, one stream per field.
//
// Chunk layout (stage 1):
//   geometry section   u8 mode: 0 = raw FLOAT32 columns x, y, z; 1 = predicted, followed by
//                        u8 predictor (V6Predictor), uvarint K, u8 mask kind (V6MaskKind),
//                        [ceil(n / 8) bytes validity bits, LSB first, when a mask is used],
//                        uvarint size of the x stream, uvarint size of the y stream,
//                        then the x, y and z streams: one varint per valid point (encodeVarint64,
//                        0 = NaN), the residual against the prediction of the quantized value.
//   The decoder reconstructs float(double(q) * resolution).
//   regular columns    every other non-integer field, its field encoder run over all the points.
//   integer sections   the V5 adaptive sections.
namespace {

enum class V6GeometryMode : uint8_t { Raw = 0, Predicted = 1 };
enum class V6Predictor : uint8_t { Previous = 0, LagK = 1, Median = 2, SecondOrder = 3 };
enum class V6MaskKind : uint8_t { None = 0, NaN = 1, Zero = 2 };

constexpr size_t kV6GeometryFields = 3;
constexpr size_t kV6MaxLag = 2100;
constexpr size_t kV6ProbePoints = 4096;
// Quantized values beyond this magnitude make the chunk store its geometry raw.
constexpr double kV6MaxQuantized = 1125899906842624.0;  // 2^50

struct V6Geometry {
  std::array<uint32_t, 3> offset{};
  std::array<float, 3> resolution{};
  std::array<double, 3> inv_resolution{};
};

V6Geometry makeV6Geometry(const EncodingInfo& info) {
  V6Geometry g;
  for (size_t a = 0; a < kV6GeometryFields; ++a) {
    g.offset[a] = info.fields[a].offset;
    g.resolution[a] = *info.fields[a].resolution;
    g.inv_resolution[a] = 1.0 / static_cast<double>(g.resolution[a]);
  }
  return g;
}

float readF32(const uint8_t* ptr) {
  float v;
  std::memcpy(&v, ptr, sizeof(v));
  return v;
}

// Prediction of value i of one axis from the values already reconstructed (same axis only).
template <V6Predictor P>
inline int64_t v6Predict(const int64_t* q, size_t i, size_t K) {
  const int64_t prev = i >= 1 ? q[i - 1] : 0;
  if constexpr (P == V6Predictor::Previous) {
    return prev;
  } else if constexpr (P == V6Predictor::LagK) {
    return i >= K ? q[i - K] : prev;
  } else if constexpr (P == V6Predictor::Median) {
    if (i <= K) {
      return prev;
    }
    // LOCO-I median edge detector
    const int64_t pa = prev, pb = q[i - K], pc = q[i - K - 1];
    const int64_t mx = std::max(pa, pb), mn = std::min(pa, pb);
    return pc >= mx ? mn : (pc <= mn ? mx : pa + pb - pc);
  } else {
    return i >= 2 ? 2 * prev - q[i - 2] : prev;
  }
}

// Quantized geometry of one chunk, axis-major (value i of axis a at a * points + i).
struct V6ChunkGeometry {
  size_t points = 0;
  std::vector<int64_t> quantized;  // NaN axes as 0
  std::vector<uint8_t> nan;
  std::vector<uint8_t> valid;  // 1 per point: coded in the streams
  V6MaskKind mask = V6MaskKind::None;
  bool raw = false;
};

void quantizeV6Chunk(const V6Geometry& g, const uint8_t* points, size_t point_step, size_t n, V6ChunkGeometry& out) {
  out.points = n;
  out.quantized.resize(n * 3);
  out.nan.resize(n * 3);
  out.valid.assign(n, 1);
  out.raw = false;
  out.mask = V6MaskKind::None;
  size_t nan_points = 0, zero_points = 0;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t* p = points + i * point_step;
    int nan_axes = 0, zero_axes = 0;
    for (size_t a = 0; a < 3; ++a) {
      const float v = readF32(p + g.offset[a]);
      const bool is_nan = std::isnan(v);
      out.nan[a * n + i] = is_nan;
      nan_axes += is_nan;
      zero_axes += (v == 0.0f);
      const double scaled = is_nan ? 0.0 : std::nearbyint(static_cast<double>(v) * g.inv_resolution[a]);
      if (!(std::fabs(scaled) < kV6MaxQuantized)) {
        out.raw = true;  // infinite or too large for the integer streams
      }
      out.quantized[a * n + i] = out.raw ? 0 : static_cast<int64_t>(scaled);
    }
    nan_points += (nan_axes == 3);
    zero_points += (zero_axes == 3);
  }
  if (out.raw || (nan_points == 0 && zero_points == 0)) {
    return;
  }
  out.mask = nan_points >= zero_points ? V6MaskKind::NaN : V6MaskKind::Zero;
  for (size_t i = 0; i < n; ++i) {
    bool invalid = true;
    for (size_t a = 0; a < 3; ++a) {
      const bool is_nan = out.nan[a * n + i];
      invalid &= out.mask == V6MaskKind::NaN ? is_nan
                                             : (!is_nan && out.quantized[a * n + i] == 0 &&
                                                readF32(points + i * point_step + g.offset[a]) == 0.0f);
    }
    out.valid[i] = !invalid;
  }
}

// Residual stream of one axis for predictor P, written at `out`; returns its size. `f` receives the
// values the decoder reconstructs (an invalid point repeats the previous value, a NaN axis the prediction).
template <V6Predictor P>
size_t encodeV6Axis(
    const int64_t* q, const uint8_t* nan, const uint8_t* valid, size_t n, size_t K, int64_t* f, uint8_t* out) {
  uint8_t* o = out;
  for (size_t i = 0; i < n; ++i) {
    if (valid && !valid[i]) {
      f[i] = i >= 1 ? f[i - 1] : 0;
      continue;
    }
    const int64_t pred = v6Predict<P>(f, i, K);
    if (nan[i]) {
      f[i] = pred;
      *o++ = 0;  // NaN marker
      continue;
    }
    f[i] = q[i];
    o += encodeVarint64(q[i] - pred, o);
  }
  return static_cast<size_t>(o - out);
}

// Size of the residual stream of one axis for predictor P, without writing it.
template <V6Predictor P>
size_t estimateV6Axis(const int64_t* q, const uint8_t* nan, const uint8_t* valid, size_t n, size_t K, int64_t* f) {
  size_t bytes = 0;
  for (size_t i = 0; i < n; ++i) {
    if (valid && !valid[i]) {
      f[i] = i >= 1 ? f[i - 1] : 0;
      continue;
    }
    const int64_t pred = v6Predict<P>(f, i, K);
    if (nan[i]) {
      f[i] = pred;
      ++bytes;
      continue;
    }
    f[i] = q[i];
    const int64_t d = q[i] - pred;
    const uint64_t zz = (static_cast<uint64_t>(d) << 1) ^ static_cast<uint64_t>(d >> 63);
    const int bits = 64 - __builtin_clzll(zz + 1);  // encodeVarint64 codes zz + 1
    bytes += static_cast<size_t>((bits + 6) / 7);
  }
  return bytes;
}

struct V6Streams {
  std::array<std::vector<uint8_t>, 3> data;  // grows, never shrinks (no zero-fill per chunk)
  std::array<size_t, 3> size{0, 0, 0};
  std::vector<int64_t> filled;
};

// The three residual streams of the first `count` points in one pass over the points.
template <V6Predictor P>
void encodeV6Geometry(const V6ChunkGeometry& c, size_t count, size_t K, V6Streams& s) {
  const size_t n = c.points;
  const uint8_t* valid = c.mask == V6MaskKind::None ? nullptr : c.valid.data();
  const int64_t* q[3] = {c.quantized.data(), c.quantized.data() + n, c.quantized.data() + 2 * n};
  const uint8_t* nan[3] = {c.nan.data(), c.nan.data() + n, c.nan.data() + 2 * n};
  int64_t* f[3] = {s.filled.data(), s.filled.data() + count, s.filled.data() + 2 * count};
  uint8_t* o[3] = {s.data[0].data(), s.data[1].data(), s.data[2].data()};
  for (size_t i = 0; i < count; ++i) {
    if (valid && !valid[i]) {
      for (size_t a = 0; a < 3; ++a) {
        f[a][i] = i >= 1 ? f[a][i - 1] : 0;
      }
      continue;
    }
    for (size_t a = 0; a < 3; ++a) {
      const int64_t pred = v6Predict<P>(f[a], i, K);
      if (nan[a][i]) {
        f[a][i] = pred;
        *o[a]++ = 0;  // NaN marker
      } else {
        f[a][i] = q[a][i];
        o[a] += encodeVarint64(q[a][i] - pred, o[a]);
      }
    }
  }
  for (size_t a = 0; a < 3; ++a) {
    s.size[a] = static_cast<size_t>(o[a] - s.data[a].data());
  }
}

// Residual streams of the first `count` points of the chunk for a predictor.
void buildV6Streams(const V6ChunkGeometry& c, size_t count, V6Predictor predictor, size_t K, V6Streams& s) {
  if (s.filled.size() < 3 * count) {
    s.filled.resize(3 * count);
  }
  for (size_t a = 0; a < 3; ++a) {
    if (s.data[a].size() < count * kMaxVarintBytes) {
      s.data[a].resize(count * kMaxVarintBytes);
    }
  }
  switch (predictor) {
    case V6Predictor::Previous:
      encodeV6Geometry<V6Predictor::Previous>(c, count, K, s);
      break;
    case V6Predictor::LagK:
      encodeV6Geometry<V6Predictor::LagK>(c, count, K, s);
      break;
    case V6Predictor::Median:
      encodeV6Geometry<V6Predictor::Median>(c, count, K, s);
      break;
    case V6Predictor::SecondOrder:
      encodeV6Geometry<V6Predictor::SecondOrder>(c, count, K, s);
      break;
  }
}

// How the predictor of a chunk is chosen: by the stage-1 size of the probe (default) or by its
// size after trial compression. Set with "v6_select=compressed" in EncodingInfo::encoding_config.
bool v6SelectByCompressedSize(const EncodingInfo& info) {
  return info.encoding_config.find("v6_select=compressed") != std::string::npos;
}

size_t v6StreamsCost(const V6Streams& s, CompressionOption compression, bool compressed_size) {
  size_t cost = 0;
  std::vector<uint8_t> compressed;
  for (size_t a = 0; a < 3; ++a) {
    if (!compressed_size || compression == CompressionOption::NONE || s.size[a] == 0) {
      cost += s.size[a];
      continue;
    }
    compressed.resize(CompressBound(compression, s.size[a]));
    BufferView view(compressed.data(), compressed.size());
    cost += CompressChunk(compression, ConstBufferView(s.data[a].data(), s.size[a]), view);
  }
  return cost;
}

// Lag between a point and "the same laser one firing earlier", from the first chunk: the K with the
// smallest mean distance between points i and i - K, on a sample of points.
size_t detectV6Lag(const V6ChunkGeometry& c) {
  const size_t n = std::min<size_t>(c.points, 8192);
  if (n < 64) {
    return 0;
  }
  // forward-filled copy of the first n points, point-major for the lag scan
  std::vector<int64_t> q(n * 3, 0);
  for (size_t i = 0; i < n; ++i) {
    for (size_t a = 0; a < 3; ++a) {
      const bool keep = c.valid[i] && !c.nan[a * c.points + i];
      q[i * 3 + a] = keep ? c.quantized[a * c.points + i] : (i >= 1 ? q[(i - 1) * 3 + a] : 0);
    }
  }
  const size_t max_lag = std::min(kV6MaxLag, n / 2);
  constexpr size_t kSamples = 64;
  const size_t span = n - max_lag;
  size_t best_lag = 0;
  int64_t best_cost = 0;
  for (size_t K = 2; K <= max_lag; ++K) {
    int64_t cost = 0;
    for (size_t s = 0; s < kSamples; ++s) {
      const size_t i = max_lag + (s * span) / kSamples;
      for (size_t a = 0; a < 3; ++a) {
        cost += std::llabs(q[i * 3 + a] - q[(i - K) * 3 + a]);
      }
    }
    if (best_lag == 0 || cost < best_cost) {
      best_lag = K;
      best_cost = cost;
    }
  }
  return best_lag;
}

size_t estimateV6Streams(const V6ChunkGeometry& c, size_t count, V6Predictor predictor, size_t K, V6Streams& s) {
  if (s.filled.size() < count) {
    s.filled.resize(count);
  }
  const uint8_t* valid = c.mask == V6MaskKind::None ? nullptr : c.valid.data();
  size_t bytes = 0;
  for (size_t a = 0; a < 3; ++a) {
    const int64_t* q = c.quantized.data() + a * c.points;
    const uint8_t* nan = c.nan.data() + a * c.points;
    switch (predictor) {
      case V6Predictor::Previous:
        bytes += estimateV6Axis<V6Predictor::Previous>(q, nan, valid, count, K, s.filled.data());
        break;
      case V6Predictor::LagK:
        bytes += estimateV6Axis<V6Predictor::LagK>(q, nan, valid, count, K, s.filled.data());
        break;
      case V6Predictor::Median:
        bytes += estimateV6Axis<V6Predictor::Median>(q, nan, valid, count, K, s.filled.data());
        break;
      case V6Predictor::SecondOrder:
        bytes += estimateV6Axis<V6Predictor::SecondOrder>(q, nan, valid, count, K, s.filled.data());
        break;
    }
  }
  return bytes;
}

V6Predictor chooseV6Predictor(
    const V6ChunkGeometry& c, size_t K, CompressionOption compression, bool compressed_size, V6Streams& s) {
  const size_t probe = std::min(c.points, kV6ProbePoints);
  V6Predictor candidates[4] = {V6Predictor::Previous, V6Predictor::SecondOrder, V6Predictor::LagK, V6Predictor::Median};
  const size_t count = (K > 1 && K + 1 < probe) ? 4 : 2;
  V6Predictor best = V6Predictor::Previous;
  size_t best_cost = std::numeric_limits<size_t>::max();
  for (size_t k = 0; k < count; ++k) {
    size_t cost = 0;
    if (compressed_size && compression != CompressionOption::NONE) {
      buildV6Streams(c, probe, candidates[k], K, s);
      cost = v6StreamsCost(s, compression, true);
    } else {
      cost = estimateV6Streams(c, probe, candidates[k], K, s);
    }
    if (cost < best_cost) {
      best_cost = cost;
      best = candidates[k];
    }
  }
  return best;
}

void appendBytes(BufferView& out, const uint8_t* data, size_t size) {
  if (out.size() < size) {
    throw std::runtime_error("V6: output buffer full");
  }
  std::memcpy(out.data(), data, size);
  out.trim_front(size);
}

bool isV6RegularField(const EncodingInfo& info, size_t index) {
  return index >= kV6GeometryFields && !isV5AdaptiveIntType(info.fields[index].type);
}

// FLOAT32 fields with a resolution after x, y, z (e.g. intensity): coded like one geometry axis, with the
// previous-value predictor, in a column of their own.
bool isV6FloatColumn(const EncodingInfo& info, size_t index) {
  const auto& field = info.fields[index];
  return index >= kV6GeometryFields && field.type == FieldType::FLOAT32 && field.resolution && *field.resolution > 0.0f;
}

// One V6 float column: u8 mode (V6GeometryMode), then the raw FLOAT32 values or the residual stream.
void encodeV6FloatColumn(
    const uint8_t* base, size_t step, uint32_t offset, float resolution, size_t n, V6Streams& s,
    std::vector<int64_t>& quantized, std::vector<uint8_t>& nan, BufferView& out) {
  quantized.resize(n);
  nan.resize(n);
  const double inv = 1.0 / static_cast<double>(resolution);
  bool raw = false;
  for (size_t i = 0; i < n; ++i) {
    const float v = readF32(base + i * step + offset);
    nan[i] = std::isnan(v);
    const double scaled = nan[i] ? 0.0 : std::nearbyint(static_cast<double>(v) * inv);
    raw |= !(std::fabs(scaled) < kV6MaxQuantized);
    quantized[i] = raw ? 0 : static_cast<int64_t>(scaled);
  }
  if (raw) {
    appendByte(out, static_cast<uint8_t>(V6GeometryMode::Raw));
    for (size_t i = 0; i < n; ++i) {
      appendBytes(out, base + i * step + offset, sizeof(float));
    }
    return;
  }
  appendByte(out, static_cast<uint8_t>(V6GeometryMode::Predicted));
  if (s.filled.size() < n) {
    s.filled.resize(n);
  }
  if (s.data[0].size() < n * kMaxVarintBytes) {
    s.data[0].resize(n * kMaxVarintBytes);
  }
  const size_t bytes = encodeV6Axis<V6Predictor::Previous>(
      quantized.data(), nan.data(), nullptr, n, 0, s.filled.data(), s.data[0].data());
  appendBytes(out, s.data[0].data(), bytes);
}

}  // namespace

bool UsesV5Codec(const EncodingInfo& info) {
  if (info.version < 5 || info.encoding_opt != EncodingOptions::LOSSY) {
    return false;
  }

  const size_t start_index = LeadingLossyFloatFieldCount(info);
  return std::any_of(info.fields.begin() + start_index, info.fields.end(), [](const auto& field) {
    return isV5AdaptiveIntType(field.type);
  });
}

size_t V5StageBufferSize(const EncodingInfo& info, size_t points_per_chunk) {
  const size_t max_per_point = MaxSerializedPointSize(info);
  return points_per_chunk * (std::max<size_t>(info.point_step, max_per_point) + 64u) + info.fields.size() * 64u + 1024u;
}

void EncodeV5Stage1(
    const EncodingInfo& info, ConstBufferView cloud_data, size_t points_count, size_t points_per_chunk,
    const std::function<BufferView()>& get_stage_buffer,
    const std::function<void(size_t serialized_size, std::span<const size_t> section_starts)>& write_stage1_chunk) {
  V5EncoderPlan plan = buildV5Plan(info, points_per_chunk);
  std::vector<size_t> section_starts;
  section_starts.reserve(plan.adaptive.size());

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
        std::any_of(plan.adaptive.begin(), plan.adaptive.end(), [](const auto& field) { return !field.committed; });

    if (has_uncommitted_adaptive && chunk_points > kAdaptiveModeProbePoints) {
      encode_point_range(0, kAdaptiveModeProbePoints);
      for (auto& adaptive : plan.adaptive) {
        commitAdaptiveIntMode(adaptive, info.compression_opt);
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
    section_starts.clear();
    for (auto& adaptive : plan.adaptive) {
      section_starts.push_back(stage_buffer.size() - stage_view.size());
      appendCommittedAdaptiveIntSection(adaptive, info.compression_opt, stage_view);
    }

    write_stage1_chunk(stage_buffer.size() - stage_view.size(), section_starts);

    point_offset += chunk_points;
    points_left -= chunk_points;
  }
}

void BuildV5Decoders(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders, size_t& min_encoded_point_bytes) {
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
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders, ConstBufferView& encoded_view,
    BufferView& output_buffer, size_t expected_points) {
  if (expected_points == 0) {
    throw std::runtime_error("V5 chunks require an expected point count");
  }
  const size_t output_bytes = expected_points * info.point_step;
  if (output_buffer.size() < output_bytes) {
    throw std::runtime_error("Output buffer is too small to hold the decoded V5 data");
  }

  ResetDecoders(decoders);
  uint8_t* chunk_output = output_buffer.data();
  if (decoders.size() == 1) {
    // Typically the xyz(i) vector, when every other field is an adaptive section.
    decoders.front()->decodePoints(encoded_view, chunk_output, info.point_step, expected_points);
  } else {
    for (size_t p = 0; p < expected_points; ++p) {
      BufferView point_view(chunk_output + p * info.point_step, info.point_step);
      for (auto& decoder : decoders) {
        decoder->decode(encoded_view, point_view);
      }
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

//==========================================================================================
// V6 (experimental)

bool UsesV6Codec(const EncodingInfo& info) {
  if (info.version < 6 || info.encoding_opt != EncodingOptions::LOSSY || info.fields.size() < kV6GeometryFields) {
    return false;
  }
  for (size_t a = 0; a < kV6GeometryFields; ++a) {
    const auto& field = info.fields[a];
    if (field.type != FieldType::FLOAT32 || !field.resolution || *field.resolution <= 0.0f) {
      return false;
    }
  }
  return true;
}

bool V6SeparateZstdFrames(const EncodingInfo& info) {
  return UsesV6Codec(info) && info.encoding_config.find("v6_zstd=frames") != std::string::npos;
}

size_t V6StageBufferSize(const EncodingInfo& info, size_t points_per_chunk) {
  return V5StageBufferSize(info, points_per_chunk) + points_per_chunk / 8 + 1024;
}

namespace {
struct V6EncoderScratch {
  V6ChunkGeometry chunk;
  V6Streams streams;
  std::vector<uint8_t> mask_bits;
  std::vector<size_t> section_starts;
  std::vector<int64_t> column_quantized;
  std::vector<uint8_t> column_nan;
};
}  // namespace

void EncodeV6Stage1(
    const EncodingInfo& info, V6EncoderState& state, ConstBufferView cloud_data, size_t points_count,
    size_t points_per_chunk, const std::function<BufferView()>& get_stage_buffer,
    const std::function<void(size_t serialized_size, std::span<const size_t> section_starts)>& write_stage1_chunk) {
  const V6Geometry geometry = makeV6Geometry(info);
  // non-integer columns in field order: a V6 float column (null encoder) or a V5 field encoder
  std::vector<std::pair<size_t, std::unique_ptr<FieldEncoder>>> regular;
  std::vector<V5AdaptiveIntField> adaptive;
  for (size_t i = kV6GeometryFields; i < info.fields.size(); ++i) {
    const auto& field = info.fields[i];
    if (isV6FloatColumn(info, i)) {
      regular.emplace_back(i, nullptr);
    } else if (isV5AdaptiveIntType(field.type)) {
      V5AdaptiveIntField a;
      a.field_index = i;
      a.name = field.name;
      a.type = field.type;
      a.offset = field.offset;
      a.bytes_per_value = static_cast<size_t>(SizeOf(field.type));
      a.values.reserve(points_per_chunk);
      a.raw_values.reserve(points_per_chunk);
      adaptive.push_back(std::move(a));
    } else {
      regular.emplace_back(i, CreateCompatibleEncoder(info, field));
    }
  }
  if (!state.scratch) {
    state.scratch = std::make_shared<V6EncoderScratch>();
  }
  auto& scratch = *static_cast<V6EncoderScratch*>(state.scratch.get());
  auto& column_quantized = scratch.column_quantized;
  auto& column_nan = scratch.column_nan;
  auto& chunk = scratch.chunk;
  auto& streams = scratch.streams;
  auto& mask_bits = scratch.mask_bits;
  auto& section_starts = scratch.section_starts;

  // Lag and predictors: reused from the previous clouds of the same size, probed again periodically.
  const bool use_cache = info.encoding_config.find("v6_cache=off") == std::string::npos;
  const size_t chunks_count = (points_count + points_per_chunk - 1) / points_per_chunk;
  if (!use_cache || state.encodes % kV6ReprobeInterval == 0 || state.cloud_points != points_count) {
    state.cloud_points = points_count;
    state.lag_known = false;
    state.predictors.assign(chunks_count, 0xFF);
    state.encodes = 0;
  }
  state.encodes++;
  if (info.height > 1) {  // organized clouds: the point one row up
    state.lag = info.width;
    state.lag_known = true;
  }

  const bool select_by_compressed_size = v6SelectByCompressedSize(info);
  // experiment: "v6_blocks=none" starts no ZSTD block at the V6 sections (one-shot compression)
  const bool mark_sections = info.encoding_config.find("v6_blocks=none") == std::string::npos;
  size_t chunk_index = 0;

  size_t points_left = points_count;
  size_t point_offset = 0;
  while (points_left > 0) {
    const size_t n = std::min(points_left, points_per_chunk);
    const uint8_t* base = cloud_data.data() + point_offset * info.point_step;

    BufferView stage_buffer = get_stage_buffer();
    BufferView out(stage_buffer.data(), stage_buffer.size());
    section_starts.clear();
    auto mark_section = [&] {
      if (mark_sections) {
        section_starts.push_back(stage_buffer.size() - out.size());
      }
    };

    quantizeV6Chunk(geometry, base, info.point_step, n, chunk);
    if (chunk.raw) {
      appendByte(out, static_cast<uint8_t>(V6GeometryMode::Raw));
      for (size_t a = 0; a < kV6GeometryFields; ++a) {
        mark_section();
        for (size_t i = 0; i < n; ++i) {
          appendBytes(out, base + i * info.point_step + geometry.offset[a], sizeof(float));
        }
      }
    } else {
      if (!state.lag_known) {
        state.lag = detectV6Lag(chunk);
        state.lag_known = true;
      }
      const size_t lag = state.lag;
      uint8_t& cached = state.predictors[chunk_index];
      if (cached == 0xFF) {
        cached = static_cast<uint8_t>(
            chooseV6Predictor(chunk, lag, info.compression_opt, select_by_compressed_size, streams));
      }
      V6Predictor predictor = static_cast<V6Predictor>(cached);
      if ((predictor == V6Predictor::LagK || predictor == V6Predictor::Median) && (lag < 2 || lag + 1 >= n)) {
        predictor = V6Predictor::Previous;  // the lag does not fit this chunk
      }
      buildV6Streams(chunk, n, predictor, lag, streams);
      appendByte(out, static_cast<uint8_t>(V6GeometryMode::Predicted));
      appendByte(out, static_cast<uint8_t>(predictor));
      appendUVarint(lag, out);
      appendByte(out, static_cast<uint8_t>(chunk.mask));
      if (chunk.mask != V6MaskKind::None) {
        mask_bits.assign((n + 7) / 8, 0);
        for (size_t i = 0; i < n; ++i) {
          mask_bits[i / 8] |= static_cast<uint8_t>(chunk.valid[i] << (i % 8));
        }
        appendBytes(out, mask_bits.data(), mask_bits.size());
      }
      appendUVarint(streams.size[0], out);
      appendUVarint(streams.size[1], out);
      for (size_t a = 0; a < kV6GeometryFields; ++a) {
        mark_section();
        appendBytes(out, streams.data[a].data(), streams.size[a]);
      }
    }

    // every other non-integer field in its own column
    for (auto& [index, encoder] : regular) {
      mark_section();
      if (!encoder) {
        const auto& field = info.fields[index];
        encodeV6FloatColumn(
            base, info.point_step, field.offset, *field.resolution, n, streams, column_quantized, column_nan, out);
        continue;
      }
      encoder->reset();
      for (size_t i = 0; i < n; ++i) {
        encoder->encode(ConstBufferView(base + i * info.point_step, info.point_step), out);
      }
      encoder->flush(out);
    }

    // integer fields: the V5 adaptive sections (same mode selection as V5)
    for (auto& field : adaptive) {
      prepareAdaptiveIntFieldForChunk(field, n);
    }
    auto collect_range = [&](size_t first, size_t last) {
      for (size_t i = first; i < last; ++i) {
        for (auto& field : adaptive) {
          collectAdaptiveIntValue(field, base + i * info.point_step + field.offset);
        }
      }
    };
    const bool has_uncommitted =
        std::any_of(adaptive.begin(), adaptive.end(), [](const auto& field) { return !field.committed; });
    if (has_uncommitted && n > kAdaptiveModeProbePoints) {
      collect_range(0, kAdaptiveModeProbePoints);
      for (auto& field : adaptive) {
        commitAdaptiveIntMode(field, info.compression_opt);
        beginCommittedAdaptiveIntSection(field, n);
        appendProbeValuesToCommittedSection(field);
      }
      collect_range(kAdaptiveModeProbePoints, n);
    } else {
      collect_range(0, n);
    }
    for (auto& field : adaptive) {
      mark_section();
      appendCommittedAdaptiveIntSection(field, info.compression_opt, out);
    }

    write_stage1_chunk(stage_buffer.size() - out.size(), section_starts);
    point_offset += n;
    points_left -= n;
    ++chunk_index;
  }
}

void BuildV6Decoders(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders, size_t& min_encoded_point_bytes) {
  decoders.clear();
  min_encoded_point_bytes = 0;
  for (size_t i = kV6GeometryFields; i < info.fields.size(); ++i) {
    if (isV6RegularField(info, i) && !isV6FloatColumn(info, i)) {
      decoders.push_back(CreateCompatibleDecoder(info, info.fields[i]));
    }
  }
}

namespace {

// Value of a quantized step. While q is exact in a float, float(q) * resolution gives the same bits (the
// double product of two floats is exact, and both round it once), so the double product costs nothing more.
inline float v6Reconstruct(int64_t q, double res) {
  return static_cast<float>(static_cast<double>(q) * res);
}

// Reads one residual; returns false for the NaN marker. Unchecked: at least kMaxVarintBytes are readable.
template <bool Checked>
inline bool v6ReadResidual(const uint8_t*& ptr, const uint8_t* end, int64_t& residual) {
  if constexpr (Checked) {
    if (ptr == end) {
      throw std::runtime_error("V6: truncated geometry stream");
    }
  }
  if (*ptr == 0) {
    ++ptr;
    return false;
  }
  uint64_t uval;
  if (!Checked && ptr[0] < 0x80u) {
    uval = ptr[0];
    ptr += 1;
  } else if (!Checked && ptr[1] < 0x80u) {
    uval = static_cast<uint64_t>(ptr[0] & 0x7Fu) | (static_cast<uint64_t>(ptr[1]) << 7);
    ptr += 2;
  } else if (!Checked && ptr[2] < 0x80u) {
    uval = static_cast<uint64_t>(ptr[0] & 0x7Fu) | (static_cast<uint64_t>(ptr[1] & 0x7Fu) << 7) |
           (static_cast<uint64_t>(ptr[2]) << 14);
    ptr += 3;
  } else {
    ptr += Checked ? decodeVarint(ptr, static_cast<size_t>(end - ptr), residual) : decodeVarintUnchecked(ptr, residual);
    return true;
  }
  uval--;  // same zig-zag as decodeVarint (0 is the NaN marker, handled above)
  residual = static_cast<int64_t>((uval >> 1) ^ static_cast<uint64_t>(-static_cast<int64_t>(uval & 1)));
  return true;
}

struct V6AxisOut {
  uint8_t* dst = nullptr;  // null: kDecodeButSkipStore
  double res = 0.0;
};

// Decodes one axis value of a valid point.
template <V6Predictor P, bool Checked>
inline float v6DecodeValue(
    const uint8_t*& ptr, const uint8_t* end, int64_t* q, size_t i, size_t K, const V6AxisOut& axis) {
  const int64_t pred = v6Predict<P>(q, i, K);
  int64_t residual = 0;
  if (!v6ReadResidual<Checked>(ptr, end, residual)) {
    q[i] = pred;
    return std::numeric_limits<float>::quiet_NaN();
  }
  q[i] = static_cast<int64_t>(static_cast<uint64_t>(pred) + static_cast<uint64_t>(residual));
  return v6Reconstruct(q[i], axis.res);
}

// Decodes x, y and z together: one pass over the points, three stream pointers. The validity mask is
// read 64 points at a time, and a block of valid points skips the per-point test.
template <V6Predictor P>
void decodeV6Geometry(
    std::array<const uint8_t*, 3>& ptr, const std::array<const uint8_t*, 3>& end, size_t n, size_t K,
    const uint8_t* valid_bits, size_t mask_bytes, std::array<int64_t*, 3> q, size_t step,
    const std::array<V6AxisOut, 3>& axes, float invalid_value) {
  for (size_t block = 0; block < n; block += 64) {
    const size_t block_end = std::min(n, block + 64);
    uint64_t bits = ~uint64_t(0);
    if (valid_bits) {
      bits = 0;
      std::memcpy(&bits, valid_bits + block / 8, std::min<size_t>(8, mask_bytes - block / 8));
    }
    const size_t count = block_end - block;
    const bool all_valid =
        !valid_bits || (count == 64 ? bits == ~uint64_t(0) : (~bits & ((uint64_t(1) << count) - 1)) == 0);
    const size_t worst = count * kMaxVarintBytes;
    const bool fast = static_cast<size_t>(end[0] - ptr[0]) >= worst && static_cast<size_t>(end[1] - ptr[1]) >= worst &&
                      static_cast<size_t>(end[2] - ptr[2]) >= worst;
    auto run = [&](auto checked_tag) {
      constexpr bool Checked = decltype(checked_tag)::value;
      // locals, so that the stream pointers stay in registers
      const uint8_t* p0 = ptr[0];
      const uint8_t* p1 = ptr[1];
      const uint8_t* p2 = ptr[2];
      const uint8_t *e0 = end[0], *e1 = end[1], *e2 = end[2];
      int64_t *q0 = q[0], *q1 = q[1], *q2 = q[2];
      const V6AxisOut a0 = axes[0], a1 = axes[1], a2 = axes[2];
      for (size_t i = block; i < block_end; ++i) {
        if (!all_valid && !((bits >> (i - block)) & 1u)) {
          q0[i] = i >= 1 ? q0[i - 1] : 0;
          q1[i] = i >= 1 ? q1[i - 1] : 0;
          q2[i] = i >= 1 ? q2[i - 1] : 0;
          for (const V6AxisOut* a : {&a0, &a1, &a2}) {
            if (a->dst) {
              std::memcpy(a->dst + i * step, &invalid_value, sizeof(float));
            }
          }
          continue;
        }
        const float x = v6DecodeValue<P, Checked>(p0, e0, q0, i, K, a0);
        const float y = v6DecodeValue<P, Checked>(p1, e1, q1, i, K, a1);
        const float z = v6DecodeValue<P, Checked>(p2, e2, q2, i, K, a2);
        if (a0.dst) {
          std::memcpy(a0.dst + i * step, &x, sizeof(float));
        }
        if (a1.dst) {
          std::memcpy(a1.dst + i * step, &y, sizeof(float));
        }
        if (a2.dst) {
          std::memcpy(a2.dst + i * step, &z, sizeof(float));
        }
      }
      ptr[0] = p0;
      ptr[1] = p1;
      ptr[2] = p2;
    };
    if (fast) {
      run(std::false_type{});
    } else {
      run(std::true_type{});
    }
  }
}

// Decodes a single-stream V6 float column with the previous-value predictor.
void decodeV6FloatColumn(
    ConstBufferView& input, size_t n, int64_t* q, uint8_t* base, size_t step, uint32_t offset, float res) {
  const uint8_t* ptr = input.data();
  const uint8_t* const end = input.data() + input.size();
  V6AxisOut axis;
  axis.dst = offset == kDecodeButSkipStore ? nullptr : base + offset;
  axis.res = static_cast<double>(res);
  size_t i = 0;
  for (; i < n && static_cast<size_t>(end - ptr) >= kMaxVarintBytes; ++i) {
    const float value = v6DecodeValue<V6Predictor::Previous, false>(ptr, end, q, i, 0, axis);
    if (axis.dst) {
      std::memcpy(axis.dst + i * step, &value, sizeof(float));
    }
  }
  for (; i < n; ++i) {
    const float value = v6DecodeValue<V6Predictor::Previous, true>(ptr, end, q, i, 0, axis);
    if (axis.dst) {
      std::memcpy(axis.dst + i * step, &value, sizeof(float));
    }
  }
  input.trim_front(static_cast<size_t>(ptr - input.data()));
}

}  // namespace

void DecodeV6Stage1Chunk(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders, ConstBufferView& encoded_view,
    BufferView& output_buffer, size_t expected_points) {
  const size_t n = expected_points;
  if (n == 0) {
    throw std::runtime_error("V6 chunks require an expected point count");
  }
  const size_t step = info.point_step;
  if (output_buffer.size() < n * step) {
    throw std::runtime_error("Output buffer is too small to hold the decoded V6 data");
  }
  uint8_t* base = output_buffer.data();
  const V6Geometry geometry = makeV6Geometry(info);

  if (encoded_view.empty()) {
    throw std::runtime_error("V6: missing geometry mode");
  }
  const uint8_t mode = encoded_view.data()[0];
  encoded_view.trim_front(1);
  if (mode == static_cast<uint8_t>(V6GeometryMode::Raw)) {
    for (size_t a = 0; a < kV6GeometryFields; ++a) {
      if (encoded_view.size() < n * sizeof(float)) {
        throw std::runtime_error("V6: truncated raw geometry");
      }
      if (geometry.offset[a] != kDecodeButSkipStore) {
        for (size_t i = 0; i < n; ++i) {
          std::memcpy(base + i * step + geometry.offset[a], encoded_view.data() + i * sizeof(float), sizeof(float));
        }
      }
      encoded_view.trim_front(n * sizeof(float));
    }
  } else if (mode == static_cast<uint8_t>(V6GeometryMode::Predicted)) {
    if (encoded_view.size() < 2) {
      throw std::runtime_error("V6: truncated geometry header");
    }
    const uint8_t predictor_byte = encoded_view.data()[0];
    encoded_view.trim_front(1);
    if (predictor_byte > static_cast<uint8_t>(V6Predictor::SecondOrder)) {
      throw std::runtime_error("V6: unknown predictor");
    }
    const auto predictor = static_cast<V6Predictor>(predictor_byte);
    const uint64_t lag64 = readUVarint(encoded_view);
    const size_t lag = static_cast<size_t>(std::min<uint64_t>(lag64, n + 1));
    if (encoded_view.empty()) {
      throw std::runtime_error("V6: truncated geometry header");
    }
    const uint8_t mask_byte = encoded_view.data()[0];
    encoded_view.trim_front(1);
    if (mask_byte > static_cast<uint8_t>(V6MaskKind::Zero)) {
      throw std::runtime_error("V6: unknown mask kind");
    }
    const auto mask = static_cast<V6MaskKind>(mask_byte);
    const uint8_t* valid_bits = nullptr;
    if (mask != V6MaskKind::None) {
      const size_t mask_bytes = (n + 7) / 8;
      if (encoded_view.size() < mask_bytes) {
        throw std::runtime_error("V6: truncated validity mask");
      }
      valid_bits = encoded_view.data();
      encoded_view.trim_front(mask_bytes);
    }
    const float invalid_value = mask == V6MaskKind::NaN ? std::numeric_limits<float>::quiet_NaN() : 0.0f;
    const uint64_t x_size = readUVarint(encoded_view);
    const uint64_t y_size = readUVarint(encoded_view);
    if (x_size > encoded_view.size() || y_size > encoded_view.size() - x_size) {
      throw std::runtime_error("V6: geometry stream sizes exceed the chunk");
    }
    std::array<const uint8_t*, 3> ptr = {
        encoded_view.data(), encoded_view.data() + x_size, encoded_view.data() + x_size + y_size};
    const std::array<const uint8_t*, 3> end = {ptr[1], ptr[2], encoded_view.data() + encoded_view.size()};
    thread_local std::vector<int64_t> q;
    q.resize(3 * n);
    const std::array<int64_t*, 3> qa = {q.data(), q.data() + n, q.data() + 2 * n};
    std::array<V6AxisOut, 3> axes;
    for (size_t a = 0; a < kV6GeometryFields; ++a) {
      axes[a].dst = geometry.offset[a] == kDecodeButSkipStore ? nullptr : base + geometry.offset[a];
      axes[a].res = static_cast<double>(geometry.resolution[a]);
    }
    const size_t mask_bytes = valid_bits ? (n + 7) / 8 : 0;
    switch (predictor) {
      case V6Predictor::Previous:
        decodeV6Geometry<V6Predictor::Previous>(
            ptr, end, n, lag, valid_bits, mask_bytes, qa, step, axes, invalid_value);
        break;
      case V6Predictor::LagK:
        decodeV6Geometry<V6Predictor::LagK>(ptr, end, n, lag, valid_bits, mask_bytes, qa, step, axes, invalid_value);
        break;
      case V6Predictor::Median:
        decodeV6Geometry<V6Predictor::Median>(ptr, end, n, lag, valid_bits, mask_bytes, qa, step, axes, invalid_value);
        break;
      case V6Predictor::SecondOrder:
        decodeV6Geometry<V6Predictor::SecondOrder>(
            ptr, end, n, lag, valid_bits, mask_bytes, qa, step, axes, invalid_value);
        break;
    }
    if (ptr[0] != end[0] || ptr[1] != end[1]) {
      throw std::runtime_error("V6: geometry stream sizes do not match their content");
    }
    encoded_view.trim_front(static_cast<size_t>(ptr[2] - encoded_view.data()));
  } else {
    throw std::runtime_error("V6: unknown geometry mode");
  }

  size_t next_decoder = 0;
  for (size_t index = kV6GeometryFields; index < info.fields.size(); ++index) {
    if (!isV6RegularField(info, index)) {
      continue;
    }
    if (!isV6FloatColumn(info, index)) {
      if (next_decoder >= decoders.size()) {
        throw std::runtime_error("V6: decoders do not match the fields");
      }
      auto& decoder = decoders[next_decoder++];
      decoder->reset();
      decoder->decodePoints(encoded_view, base, step, n);
      continue;
    }
    const auto& field = info.fields[index];
    if (encoded_view.empty()) {
      throw std::runtime_error("V6: missing float column mode");
    }
    const uint8_t column_mode = encoded_view.data()[0];
    encoded_view.trim_front(1);
    if (column_mode == static_cast<uint8_t>(V6GeometryMode::Raw)) {
      if (encoded_view.size() < n * sizeof(float)) {
        throw std::runtime_error("V6: truncated raw float column");
      }
      if (field.offset != kDecodeButSkipStore) {
        for (size_t i = 0; i < n; ++i) {
          std::memcpy(base + i * step + field.offset, encoded_view.data() + i * sizeof(float), sizeof(float));
        }
      }
      encoded_view.trim_front(n * sizeof(float));
    } else if (column_mode == static_cast<uint8_t>(V6GeometryMode::Predicted)) {
      thread_local std::vector<int64_t> column;
      column.resize(n);
      decodeV6FloatColumn(encoded_view, n, column.data(), base, step, field.offset, *field.resolution);
    } else {
      throw std::runtime_error("V6: unknown float column mode");
    }
  }

  for (const auto& field : getV5AdaptiveFields(info)) {
    decodeV5AdaptiveIntSection(field, encoded_view, base, step, n);
  }
  if (!encoded_view.empty()) {
    throw std::runtime_error("V6 chunk has trailing bytes after decode");
  }
  output_buffer.trim_front(n * step);
}

}  // namespace Cloudini::detail
