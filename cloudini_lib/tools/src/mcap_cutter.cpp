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
#include <set>
#include <unordered_map>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/ros_message_definitions.hpp"
#include "cxxopts.hpp"

#define MCAP_IMPLEMENTATION
#include "mcap/reader.hpp"
#include "mcap/types.hpp"
#include "mcap/writer.hpp"

class McapCutter {
 public:
  void cut(
      const std::filesystem::path& input_file, const std::filesystem::path& output_file,
      int max_messages_per_channel = 1000);

 private:
  std::ifstream input_stream_;
};

void McapCutter::cut(
    const std::filesystem::path& input_file, const std::filesystem::path& output_file, int max_messages_per_channel) {
  // Open input file
  input_stream_.open(input_file);
  auto data_source = std::make_shared<mcap::FileStreamReader>(input_stream_);
  auto reader = std::make_shared<mcap::McapReader>();

  auto res = reader->open(*data_source);
  if (!res.ok()) {
    throw std::runtime_error("Error opening MCAP file: " + res.message);
  }

  res = reader->readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  if (!res.ok()) {
    throw std::runtime_error("Error reading MCAP summary: " + res.message);
  }

  // Find PointCloud2 channels and collect their info
  std::vector<std::pair<mcap::ChannelId, std::string>> pointcloud_channels;

  for (const auto& [channel_id, channel_ptr] : reader->channels()) {
    const auto& schema_ptr = reader->schema(channel_ptr->schemaId);
    if (schema_ptr->name == pointcloud_schema_name) {
      pointcloud_channels.push_back({channel_id, channel_ptr->topic});
      std::cout << "Found PointCloud2 topic: " << channel_ptr->topic << std::endl;
    }
  }

  if (pointcloud_channels.empty()) {
    std::cout << "No PointCloud2 topics found in input file." << std::endl;
    return;
  }

  std::cout << "Found " << pointcloud_channels.size() << " PointCloud2 channels" << std::endl;

  // Setup writer
  mcap::McapWriter writer;
  mcap::McapWriterOptions writer_options(reader->header()->profile);
  writer_options.compression = mcap::Compression::Zstd;
  writer_options.chunkSize = 2 * 1024 * 1024;  // 2 MB

  auto status = writer.open(output_file.string(), writer_options);
  if (!status.ok()) {
    throw std::runtime_error("Error opening output MCAP file: " + status.message);
  }

  // Create single PointCloud2 schema
  mcap::Schema pointcloud_schema(pointcloud_schema_name, "ros2msg", pointcloud_schema_data);
  writer.addSchema(pointcloud_schema);

  // Create channels for each topic
  std::map<mcap::ChannelId, mcap::ChannelId> old_to_new_channel_id;
  for (const auto& [old_channel_id, topic_name] : pointcloud_channels) {
    mcap::Channel new_channel(topic_name, "cdr", pointcloud_schema.id);
    writer.addChannel(new_channel);
    old_to_new_channel_id[old_channel_id] = new_channel.id;
  }

  // Process messages
  mcap::ReadMessageOptions reader_options;
  mcap::ProblemCallback problem = [](const mcap::Status&) {};

  std::unordered_map<mcap::ChannelId, int> message_counts;
  int total_messages_written = 0;

  // Create set of relevant channel IDs for fast lookup
  std::set<mcap::ChannelId> relevant_channel_ids;
  for (const auto& [channel_id, _] : pointcloud_channels) {
    relevant_channel_ids.insert(channel_id);
  }

  for (const auto& msg : reader->readMessages(problem, reader_options)) {
    // Only process PointCloud2 messages
    if (relevant_channel_ids.find(msg.channel->id) == relevant_channel_ids.end()) {
      continue;
    }

    // Check message count limit
    if (message_counts[msg.channel->id] >= max_messages_per_channel) {
      continue;
    }

    // Copy message with new channel ID
    mcap::Message new_msg = msg.message;
    new_msg.channelId = old_to_new_channel_id.at(msg.channel->id);

    auto write_status = writer.write(new_msg);
    if (!write_status.ok()) {
      throw std::runtime_error("Error writing message: " + write_status.message);
    }

    message_counts[msg.channel->id]++;
    total_messages_written++;

    if (total_messages_written % 100 == 0) {
      std::cout << "Processed " << total_messages_written << " messages..." << std::endl;
    }
  }

  writer.close();

  std::cout << "\nSummary:" << std::endl;
  std::cout << "Total messages written: " << total_messages_written << std::endl;
  for (const auto& [channel_id, count] : message_counts) {
    const auto channel = reader->channels().at(channel_id);
    std::cout << "  " << channel->topic << ": " << count << " messages" << std::endl;
  }
}

int main(int argc, char** argv) {
  cxxopts::Options options("mcap_cutter", "Extract PointCloud2 messages from MCAP files");

  options.add_options()("h,help", "Print usage")("f,filename", "Input MCAP file", cxxopts::value<std::string>())(
      "o,output", "Output MCAP file", cxxopts::value<std::string>())(
      "n,max-messages", "Maximum messages per channel", cxxopts::value<int>()->default_value("1000"));

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

  std::string output_filename = input_file.stem().string() + "_cut.mcap";
  if (parse_result.count("output")) {
    output_filename = parse_result["output"].as<std::string>();
  }

  if (std::filesystem::path(output_filename).extension() != ".mcap") {
    output_filename += ".mcap";
  }

  int max_messages = parse_result["max-messages"].as<int>();
  if (max_messages <= 0) {
    std::cerr << "Error: max-messages must be positive." << std::endl;
    return 1;
  }

  std::cout << "Input file: " << input_file << std::endl;
  std::cout << "Output file: " << output_filename << std::endl;
  std::cout << "Max messages per channel: " << max_messages << std::endl;

  try {
    McapCutter cutter;
    cutter.cut(input_file, output_filename, max_messages);
    std::cout << "\nFile saved as: " << output_filename << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
