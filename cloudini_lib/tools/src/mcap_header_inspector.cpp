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

#include <filesystem>
#include <fstream>
#include <iostream>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/ros_message_definitions.hpp"
#include "cloudini_lib/ros_msg_utils.hpp"
#include "cxxopts.hpp"

#define MCAP_IMPLEMENTATION
#include "mcap/reader.hpp"
#include "mcap/types.hpp"

int main(int argc, char** argv) {
  cxxopts::Options options(
      "mcap_header_inspector", "Inspect Cloudini header from CompressedPointCloud2 messages in MCAP files");

  options.add_options()("h,help", "Print usage")("f,filename", "Input MCAP file", cxxopts::value<std::string>());

  auto parse_result = options.parse(argc, argv);

  if (parse_result.count("help")) {
    std::cout << options.help() << std::endl;
    return 0;
  }

  if (!parse_result.count("filename")) {
    std::cerr << "Error: Input file name is required." << std::endl;
    std::cout << options.help() << std::endl;
    return 1;
  }

  auto input_filename = parse_result["filename"].as<std::string>();
  const std::filesystem::path input_file = input_filename;

  if (input_file.extension() != ".mcap") {
    std::cerr << "Error: Input file must be a .mcap file." << std::endl;
    return 1;
  }

  if (!std::filesystem::exists(input_file)) {
    std::cerr << "Error: Input file does not exist: " << input_file << std::endl;
    return 1;
  }

  std::cout << "Opening MCAP file: " << input_file << std::endl;

  try {
    // Open MCAP file
    std::ifstream input_stream(input_file);
    auto data_source = std::make_shared<mcap::FileStreamReader>(input_stream);
    auto reader = std::make_shared<mcap::McapReader>();

    auto res = reader->open(*data_source);
    if (!res.ok()) {
      throw std::runtime_error("Error opening MCAP file: " + res.message);
    }

    res = reader->readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
    if (!res.ok()) {
      throw std::runtime_error("Error reading MCAP summary: " + res.message);
    }

    // Find CompressedPointCloud2 channels
    std::vector<mcap::ChannelId> compressed_channels;
    for (const auto& [channel_id, channel_ptr] : reader->channels()) {
      const auto& schema_ptr = reader->schema(channel_ptr->schemaId);
      if (schema_ptr->name == compressed_schema_name) {
        compressed_channels.push_back(channel_id);
        std::cout << "Found CompressedPointCloud2 topic: " << channel_ptr->topic << std::endl;
      }
    }

    if (compressed_channels.empty()) {
      std::cout << "No CompressedPointCloud2 topics found in MCAP file." << std::endl;
      return 0;
    }

    // Read first message
    mcap::ReadMessageOptions reader_options;
    mcap::ProblemCallback problem = [](const mcap::Status&) {};

    bool found_message = false;
    for (const auto& msg : reader->readMessages(problem, reader_options)) {
      // Check if this is a CompressedPointCloud2 message
      if (std::find(compressed_channels.begin(), compressed_channels.end(), msg.channel->id) ==
          compressed_channels.end()) {
        continue;
      }

      std::cout << "\n=== First CompressedPointCloud2 Message ===" << std::endl;
      std::cout << "Topic: " << msg.channel->topic << std::endl;
      std::cout << "Timestamp: " << msg.message.logTime << " ns" << std::endl;
      std::cout << "Message size: " << msg.message.dataSize << " bytes" << std::endl;

      // Parse the DDS message to extract compressed_data
      Cloudini::ConstBufferView raw_message(msg.message.data, msg.message.dataSize);

      // Extract the compressed_data field from the CompressedPointCloud2 message
      auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(raw_message);

      std::cout << "\nCompressed data size: " << pc_info.data.size() << " bytes" << std::endl;

      // Create a ConstBufferView for the compressed data
      Cloudini::ConstBufferView compressed_data(pc_info.data.data(), pc_info.data.size());

      // Decode the header
      std::cout << "\n--- Decoding Cloudini Header ---" << std::endl;
      auto encoding_info = Cloudini::DecodeHeader(compressed_data);

      // Convert to YAML and display
      std::string yaml = Cloudini::EncodingInfoToYAML(encoding_info);

      std::cout << "\nYAML Header:" << std::endl;
      std::cout << yaml << std::endl;

      std::cout << "\n--- Parsed Header Info ---" << std::endl;
      std::cout << "Version: " << static_cast<int>(encoding_info.version) << std::endl;
      std::cout << "Width: " << encoding_info.width << std::endl;
      std::cout << "Height: " << encoding_info.height << std::endl;
      std::cout << "Point step: " << encoding_info.point_step << std::endl;
      std::cout << "Total points: " << (encoding_info.width * encoding_info.height) << std::endl;
      std::cout << "Total data size: " << (encoding_info.width * encoding_info.height * encoding_info.point_step)
                << " bytes" << std::endl;
      std::cout << "Encoding option: " << Cloudini::ToString(encoding_info.encoding_opt) << std::endl;
      std::cout << "Compression option: " << Cloudini::ToString(encoding_info.compression_opt) << std::endl;
      std::cout << "Number of fields: " << encoding_info.fields.size() << std::endl;

      std::cout << "\n--- Field Information ---" << std::endl;
      for (size_t i = 0; i < encoding_info.fields.size(); ++i) {
        const auto& field = encoding_info.fields[i];
        std::cout << "Field " << i << ":" << std::endl;
        std::cout << "  Name: " << field.name << std::endl;
        std::cout << "  Offset: " << field.offset << std::endl;
        std::cout << "  Type: " << Cloudini::ToString(field.type) << std::endl;
        std::cout << "  Resolution: " << field.resolution.value_or(1.0) << std::endl;
      }

      found_message = true;
      break;  // Stop after first message
    }

    if (!found_message) {
      std::cout << "No CompressedPointCloud2 messages found in the file." << std::endl;
      return 1;
    }

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
