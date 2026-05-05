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

#include "v4_codec.hpp"

#include <algorithm>
#include <stdexcept>

#include "codec_common.hpp"

namespace Cloudini::detail {

void BuildV4Encoders(const EncodingInfo& info, std::vector<std::unique_ptr<FieldEncoder>>& encoders) {
  encoders.clear();

  if (info.encoding_opt == EncodingOptions::NONE) {
    for (const auto& field : info.fields) {
      encoders.push_back(std::make_unique<FieldEncoderCopy>(field.offset, field.type));
    }
    return;
  }

  const size_t start_index = AppendLeadingLossyFloatEncoder(info, encoders);
  for (size_t index = start_index; index < info.fields.size(); ++index) {
    encoders.push_back(CreateCompatibleEncoder(info, info.fields[index]));
  }
}

void BuildV4Decoders(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldDecoder>>& decoders,
    size_t& min_encoded_point_bytes) {
  decoders.clear();
  min_encoded_point_bytes = 0;

  if (info.encoding_opt == EncodingOptions::NONE) {
    for (const auto& field : info.fields) {
      decoders.push_back(std::make_unique<FieldDecoderCopy>(field.offset, field.type));
      min_encoded_point_bytes += SizeOf(field.type);
    }
    return;
  }

  const size_t start_index = AppendLeadingLossyFloatDecoder(info, decoders);
  for (size_t index = start_index; index < info.fields.size(); ++index) {
    decoders.push_back(CreateCompatibleDecoder(info, info.fields[index]));
  }

  for (const auto& decoder : decoders) {
    min_encoded_point_bytes += decoder->minInputBytes();
  }
}

size_t EncodeV4Stage1Chunk(
    const EncodingInfo& info, std::vector<std::unique_ptr<FieldEncoder>>& encoders,
    ConstBufferView& cloud_data, size_t points_per_chunk, BufferView& output) {
  ResetEncoders(encoders);

  size_t points_in_current_chunk = 0;
  size_t serialized_size = 0;
  while (!cloud_data.empty() && points_in_current_chunk < points_per_chunk) {
    for (auto& encoder : encoders) {
      serialized_size += encoder->encode(cloud_data, output);
    }
    cloud_data.trim_front(info.point_step);
    ++points_in_current_chunk;
  }

  serialized_size += FlushEncoders(encoders, output);
  return serialized_size;
}

void DecodeV4Stage1Chunk(
    std::vector<std::unique_ptr<FieldDecoder>>& decoders, size_t min_encoded_point_bytes,
    ConstBufferView& encoded_view, BufferView& output_buffer, size_t point_step,
    size_t expected_points) {
  ResetDecoders(decoders);

  auto decode_point = [&] {
    if (output_buffer.size() < point_step) {
      throw std::runtime_error("Output buffer is too small to hold the decoded data");
    }
    BufferView point_view(output_buffer.data(), point_step);
    for (auto& decoder : decoders) {
      decoder->decode(encoded_view, point_view);
    }
    output_buffer.trim_front(point_step);
  };

  if (expected_points > 0) {
    for (size_t p = 0; p < expected_points; ++p) {
      decode_point();
    }
  } else {
    while (!encoded_view.empty()) {
      if (encoded_view.size() < min_encoded_point_bytes) {
        throw std::runtime_error("Truncated encoded data: not enough bytes for a complete point");
      }
      decode_point();
    }
  }
}

}  // namespace Cloudini::detail
