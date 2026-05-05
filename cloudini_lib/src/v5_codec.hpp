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
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/field_decoder.hpp"

namespace Cloudini::detail {

bool UsesV5Codec(const EncodingInfo& info);
size_t V5StageBufferSize(const EncodingInfo& info, size_t points_per_chunk);

void EncodeV5Stage1(
    const EncodingInfo& info, ConstBufferView cloud_data, size_t points_count,
    size_t points_per_chunk, const std::function<BufferView()>& get_stage_buffer,
    const std::function<void(size_t serialized_size)>& write_stage1_chunk);

void BuildV5Decoders(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders,
    size_t& min_encoded_point_bytes);

void DecodeV5Stage1Chunk(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders,
    ConstBufferView& encoded_view, BufferView& output_buffer, size_t expected_points);

}  // namespace Cloudini::detail
