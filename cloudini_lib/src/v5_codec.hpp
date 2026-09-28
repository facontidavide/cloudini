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

#include <functional>
#include <memory>
#include <span>
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/field_decoder.hpp"

namespace Cloudini::detail {

bool UsesV5Codec(const EncodingInfo& info);
size_t V5StageBufferSize(const EncodingInfo& info, size_t points_per_chunk);

void EncodeV5Stage1(
    const EncodingInfo& info, ConstBufferView cloud_data, size_t points_count, size_t points_per_chunk,
    const std::function<BufferView()>& get_stage_buffer,
    const std::function<void(size_t serialized_size, std::span<const size_t> section_starts)>& write_stage1_chunk);

void BuildV5Decoders(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders, size_t& min_encoded_point_bytes);

void DecodeV5Stage1Chunk(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders, ConstBufferView& encoded_view,
    BufferView& output_buffer, size_t expected_points);

// The V5 adaptive integer sections, for chunks of other layouts (V6).
bool IsAdaptiveIntType(FieldType type);

// Codes the integer fields `field_indexes` of each chunk as V5 adaptive sections. The coding mode of each
// field is chosen on the first chunk and kept for the next ones, as in V5.
class AdaptiveIntSectionsEncoder {
 public:
  AdaptiveIntSectionsEncoder(const EncodingInfo& info, std::span<const size_t> field_indexes, size_t points_per_chunk);
  ~AdaptiveIntSectionsEncoder();

  // Appends one section per field to `out`; calls before_section() before each one.
  void encodeChunk(
      const uint8_t* points, size_t point_step, size_t count, CompressionOption compression, BufferView& out,
      const std::function<void()>& before_section);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Decodes the adaptive sections of the integer fields of `info` that follow its leading lossy float fields.
void DecodeAdaptiveIntSections(
    const EncodingInfo& info, ConstBufferView& input, uint8_t* points, size_t point_step, size_t count);

}  // namespace Cloudini::detail
