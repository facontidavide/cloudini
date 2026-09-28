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
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/field_decoder.hpp"

namespace Cloudini::detail {

// V6 (opt-in with EncodingInfo::version = 6): geometry predicted from the best neighbour, invalid-point
// mask, one stream per field. Requires LOSSY and x, y, z as the first three FLOAT32 fields with a
// resolution in (0, 1e18); other version-6 clouds use the V5 chunk layout (the decoder decides the same way).
bool UsesV6Codec(const EncodingInfo& info);

// State a PointcloudEncoder keeps between encode() calls with V6: the detected lag and the predictor and
// mask kind chosen for each chunk are reused for the next clouds, and probed again every few clouds.
struct V6EncoderScratch;  // buffers reused across calls, defined in v6_codec.cpp
struct V6EncoderState {
  struct ChunkChoice {
    static constexpr uint8_t kNotProbed = 0xFF;
    uint8_t predictor = kNotProbed;
    uint8_t mask = kNotProbed;
  };
  std::optional<size_t> lag;  // the row width for organized clouds, detected otherwise
  uint32_t encodes = 0;
  std::vector<ChunkChoice> chunks;
  std::shared_ptr<V6EncoderScratch> scratch;
};

size_t V6StageBufferSize(const EncodingInfo& info, size_t points_per_chunk);

void EncodeV6Stage1(
    const EncodingInfo& info, V6EncoderState& state, ConstBufferView cloud_data, size_t points_count,
    size_t points_per_chunk, const std::function<BufferView()>& get_stage_buffer,
    const std::function<void(size_t serialized_size, std::span<const size_t> section_starts)>& write_stage1_chunk);

void BuildV6Decoders(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders, size_t& min_encoded_point_bytes);

void DecodeV6Stage1Chunk(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders, ConstBufferView& encoded_view,
    BufferView& output_buffer, size_t expected_points);

}  // namespace Cloudini::detail
