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

#include "chunk_writer.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>

#include "codec_common.hpp"

namespace Cloudini::detail {

size_t WriteStage1Chunk(const EncodingInfo& info, ConstBufferView stage1_data, BufferView& output) {
  if (stage1_data.size() > std::numeric_limits<uint32_t>::max()) {
    throw std::runtime_error("Chunk too large");
  }

  if (info.compression_opt == CompressionOption::NONE) {
    Cloudini::encode(static_cast<uint32_t>(stage1_data.size()), output);
    if (output.size() < stage1_data.size()) {
      throw std::runtime_error("Output buffer too small for uncompressed chunk");
    }
    std::memcpy(output.data(), stage1_data.data(), stage1_data.size());
    output.trim_front(stage1_data.size());
    return stage1_data.size() + sizeof(uint32_t);
  }

  uint8_t* chunk_size_ptr = output.data();
  output.trim_front(sizeof(uint32_t));

  const uint32_t chunk_size = CompressChunk(info.compression_opt, stage1_data, output);
  std::memcpy(chunk_size_ptr, &chunk_size, sizeof(uint32_t));
  return static_cast<size_t>(chunk_size) + sizeof(uint32_t);
}

}  // namespace Cloudini::detail
