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

#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "cloudini_lib/cloudini.hpp"

namespace mcap {
class McapReader;
class McapWriter;
class FileStreamReader;
}  // namespace mcap

class McapConverter {
 public:
  // key: topic name, value: topic type
  using TopicsMap = std::unordered_map<std::string, std::string>;

  // open a MCAP file and return the list of topics and types
  TopicsMap open(std::filesystem::path file_in);

  // Profiles are string that contain the resolution of each filed (xyz are aggregated). Example:
  //
  // "xyz:0.001; intensity:0.1; timestamp:0.000001; ring:remove"
  //
  // This means:
  //   - "x" / "y" / "z" fields with resolution of 0.001
  //   - "intensity" field with resolution of 0.1
  //   - "timestamp" field with resolution of 0.000001
  //   - "ring" field removed
  void addProfile(const std::string& profile);

  void encodePointClouds(
      std::filesystem::path file_out, std::optional<float> default_resolution,
      Cloudini::CompressionOption mcap_writer_compression);

  void decodePointClouds(std::filesystem::path file_out, Cloudini::CompressionOption mcap_writer_compression);

  std::vector<std::pair<std::string, float>> getProfile() const;

  void printStatistics() const;

 private:
  std::ifstream input_stream_;
  std::shared_ptr<mcap::FileStreamReader> data_source_;
  std::shared_ptr<mcap::McapReader> reader_;
  TopicsMap topics_;

  void duplicateSchemasAndChannels(const mcap::McapReader& reader, mcap::McapWriter& writer, bool encoding);

  std::map<uint16_t, uint16_t> old_to_new_schema_id_;
  std::map<uint16_t, uint16_t> old_to_new_channel_id_;
  std::map<std::string, float> profile_resolutions_;

  size_t processed_messages_ = 0;
  size_t total_input_bytes_ = 0;
  size_t total_output_bytes_ = 0;
  std::chrono::microseconds total_processing_time_ = std::chrono::microseconds(0);
};
