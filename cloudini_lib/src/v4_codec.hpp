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

#include <memory>
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/field_decoder.hpp"
#include "cloudini_lib/field_encoder.hpp"

namespace Cloudini::detail {

void BuildV4Encoders(const EncodingInfo& info, std::vector<std::unique_ptr<FieldEncoder>>& encoders);
void BuildV4Decoders(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders,
    size_t& min_encoded_point_bytes);

size_t EncodeV4Stage1Chunk(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldEncoder>>& encoders,
    ConstBufferView& cloud_data, size_t points_per_chunk, BufferView& output);

void DecodeV4Stage1Chunk(
    std::vector<std::unique_ptr<FieldDecoder>>& decoders, size_t min_encoded_point_bytes,
    ConstBufferView& encoded_view, BufferView& output_buffer, size_t point_step,
    size_t expected_points);

}  // namespace Cloudini::detail
