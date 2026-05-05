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

constexpr size_t kPointsPerChunk = 32 * 1024;

size_t MaxSerializedFieldSize(const PointField& field, EncodingOptions encoding_opt);
size_t MaxSerializedPointSize(const EncodingInfo& info);

size_t LeadingLossyFloatFieldCount(const EncodingInfo& info);

size_t AppendLeadingLossyFloatEncoder(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldEncoder>>& encoders);
size_t AppendLeadingLossyFloatDecoder(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders);

std::unique_ptr<FieldEncoder> CreateCompatibleEncoder(const EncodingInfo& info, const PointField& field);
std::unique_ptr<FieldDecoder> CreateCompatibleDecoder(const EncodingInfo& info, const PointField& field);

void ResetEncoders(std::vector<std::unique_ptr<FieldEncoder>>& encoders);
void ResetDecoders(std::vector<std::unique_ptr<FieldDecoder>>& decoders);
size_t FlushEncoders(std::vector<std::unique_ptr<FieldEncoder>>& encoders, BufferView& output);

uint32_t CompressChunk(CompressionOption compression, ConstBufferView input, BufferView& output);
ConstBufferView DecompressChunk(
    CompressionOption compression, ConstBufferView chunk_data,
    std::vector<uint8_t>& decompressed_buffer, size_t max_decompressed_size);

}  // namespace Cloudini::detail
