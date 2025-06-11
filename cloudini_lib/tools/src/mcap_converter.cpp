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

#include "mcap_converter.hpp"

#include <set>
#include <stdexcept>

#include "cloudini_lib/ros_message_definitions.hpp"
#include "cloudini_lib/ros_msg_utils.hpp"
#include "mcap/reader.hpp"
#include "mcap/writer.hpp"

McapConverter::TopicsMap McapConverter::open(std::filesystem::path file_in) {
  reader_ = std::make_shared<mcap::McapReader>();
  auto res = reader_->open(file_in.string());
  if (!res.ok()) {
    reader_.reset();
    throw std::runtime_error("Error opening MCAP file: " + res.message);
  }

  TopicsMap topics;
  res = reader_->readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  if (!res.ok()) {
    reader_.reset();
    throw std::runtime_error("Error reading MCAP sumamry: " + res.message);
  }

  for (const auto& [channel_id, channel_ptr] : reader_->channels()) {
    const auto& schema_ptr = reader_->schema(channel_ptr->schemaId);
    if (schema_ptr->name == pointcloud_schema_name || schema_ptr->name == compressed_schema_name) {
      topics[channel_ptr->topic] = schema_ptr->name;
    }
  }
  topics_ = topics;
  return topics;
}

void McapConverter::duplicateSchemasAndChannels(
    const mcap::McapReader& reader, mcap::McapWriter& writer, bool encoding) {
  const auto old_schemas = reader.schemas();
  std::set<mcap::SchemaId> ordered_schema_id;
  for (const auto& [schema_id, _] : old_schemas) {
    ordered_schema_id.insert(schema_id);
  }

  const auto old_channels = reader.channels();
  std::set<mcap::ChannelId> ordered_channels_id;
  for (const auto& [channel_id, _] : old_channels) {
    ordered_channels_id.insert(channel_id);
  }

  auto copy_string_to_vector = [](const char* str, mcap::ByteArray& array) {
    size_t len = strlen(str);
    const auto* data_ptr = reinterpret_cast<const std::byte*>(str);
    array.resize(len);
    std::memcpy(array.data(), data_ptr, len);
  };

  old_to_new_schema_id_.clear();
  old_to_new_channel_id_.clear();

  for (const auto& schema_id : ordered_schema_id) {
    const auto& schema_ptr = old_schemas.at(schema_id);
    auto schema_name = schema_ptr->name;
    auto schema_data = schema_ptr->data;

    if (encoding && schema_name == pointcloud_schema_name) {
      schema_name = compressed_schema_name;
      copy_string_to_vector(compressed_schema_data, schema_data);
    }
    if (!encoding && schema_name == compressed_schema_name) {
      schema_name = pointcloud_schema_name;
      copy_string_to_vector(pointcloud_schema_name, schema_data);
    }

    mcap::Schema new_schema(schema_name, schema_ptr->encoding, schema_data);
    writer.addSchema(new_schema);
    old_to_new_schema_id_.insert({schema_id, new_schema.id});
  }

  for (const auto& channel_id : ordered_channels_id) {
    const auto channel_ptr = old_channels.at(channel_id);
    auto new_schema_id = old_to_new_schema_id_.at(channel_ptr->schemaId);
    mcap::Channel new_channel(channel_ptr->topic, channel_ptr->messageEncoding, new_schema_id);
    writer.addChannel(new_channel);
    old_to_new_channel_id_.insert({channel_ptr->id, new_channel.id});
  }
}

//------------------------------------------------------
void McapConverter::encodePointClouds(
    std::filesystem::path file_out, std::optional<float> default_resolution,
    const mcap::Compression mcap_writer_compression) {
  if (!reader_) {
    throw std::runtime_error("McapReader is not initialized. Call open() first.");
  }

  mcap::McapWriter writer;
  mcap::McapWriterOptions writer_options(reader_->header()->profile);
  writer_options.compression = mcap_writer_compression;

  auto status = writer.open(file_out.string(), writer_options);
  if (!status.ok()) {
    throw std::runtime_error("Error opening MCAP file for writing: " + status.message);
  }

  duplicateSchemasAndChannels(*reader_, writer, true);

  mcap::ReadMessageOptions options;
  mcap::ProblemCallback problem = [](const mcap::Status&) {};

  std::vector<uint8_t> compressed_cloud;
  std::vector<uint8_t> compressed_dds_msg;

  for (const auto& msg : reader_->readMessages(problem, options)) {
    mcap::Message new_msg = msg.message;
    new_msg.channelId = old_to_new_channel_id_.at(msg.channel->id);
    // default case (not a point cloud)
    if (msg.schema->name != pointcloud_schema_name) {
      auto status = writer.write(new_msg);
      if (!status.ok()) {
        throw std::runtime_error("Error writing message to MCAP file: " + status.message);
      }
      continue;
    }

    // resize vectors to maximum size
    compressed_cloud.resize(msg.message.dataSize);    // conservative size
    compressed_dds_msg.resize(msg.message.dataSize);  // conservative size

    Cloudini::ConstBufferView raw_dds_msg(msg.message.data, msg.message.dataSize);
    auto pc_info = Cloudini::readPointCloud2Message(raw_dds_msg);

    // Apply the profile to the encoding info. removing fields if resolution is 0
    // Remove first all fields that have resolution 0.0 in the profile
    applyResolutionProfile(profile_resolutions_, pc_info.fields, default_resolution);

    // Remove padding from the fields to avoid empty padding in the message

    auto encoding_info = Cloudini::toEncodingInfo(pc_info);

    // Start encoding the pointcloud data[]
    Cloudini::PointcloudEncoder pc_encoder(encoding_info);
    auto new_size = pc_encoder.encode(pc_info.data, compressed_cloud);
    compressed_cloud.resize(new_size);

    // substitute the data view
    pc_info.data = Cloudini::ConstBufferView(compressed_cloud);

    // generate the new DDS message
    Cloudini::writePointCloud2Message(pc_info, compressed_dds_msg, true);

    // copy pointers to compressed_dds_msg
    new_msg.data = reinterpret_cast<const std::byte*>(compressed_dds_msg.data());
    new_msg.dataSize = compressed_dds_msg.size();
    auto status = writer.write(new_msg);

    if (!status.ok()) {
      throw std::runtime_error("Error writing message to MCAP file: " + status.message);
    }
  }
  writer.close();
}

//------------------------------------------------------
void McapConverter::decodePointClouds(std::filesystem::path file_out, const mcap::Compression mcap_writer_compression) {
  if (!reader_) {
    throw std::runtime_error("McapReader is not initialized. Call open() first.");
  }

  mcap::McapWriter writer;
  mcap::McapWriterOptions writer_options(reader_->header()->profile);
  writer_options.compression = mcap_writer_compression;

  auto status = writer.open(file_out.string(), writer_options);
  if (!status.ok()) {
    throw std::runtime_error("Error opening MCAP file for writing: " + status.message);
  }

  duplicateSchemasAndChannels(*reader_, writer, false);

  mcap::ReadMessageOptions options;
  mcap::ProblemCallback problem = [](const mcap::Status&) {};

  std::vector<uint8_t> decoded_cloud;
  std::vector<uint8_t> decoded_dds_msg;

  for (const auto& msg : reader_->readMessages(problem, options)) {
    mcap::Message new_msg = msg.message;
    new_msg.channelId = old_to_new_channel_id_.at(msg.channel->id);
    // default case (not a point cloud)
    if (msg.schema->name != compressed_schema_name) {
      auto status = writer.write(new_msg);
      if (!status.ok()) {
        throw std::runtime_error("Error writing message to MCAP file: " + status.message);
      }
      continue;
    }

    Cloudini::ConstBufferView raw_dds_msg(msg.message.data, msg.message.dataSize);
    const auto pc_info = Cloudini::readPointCloud2Message(raw_dds_msg);

    decompressAndWritePointCloud2Message(pc_info, decoded_dds_msg);

    new_msg.data = reinterpret_cast<const std::byte*>(decoded_dds_msg.data());
    new_msg.dataSize = decoded_dds_msg.size();

    auto status = writer.write(new_msg);
    if (!status.ok()) {
      throw std::runtime_error("Error writing message to MCAP file: " + status.message);
    }
  }
  writer.close();
}

//------------------------------------------------------
std::vector<std::string_view> split(std::string_view str, char delimiter) {
  std::vector<std::string_view> tokens;
  size_t start = 0, end = 0;
  while ((end = str.find(delimiter, start)) != std::string_view::npos) {
    tokens.push_back(str.substr(start, end - start));
    start = end + 1;
  }
  tokens.push_back(str.substr(start));
  return tokens;
}

// trim front and back spaces
std::string_view trimSpaces(std::string_view str) {
  size_t start = 0, end = str.size();
  while (start < end && std::isspace(str[start])) {
    start++;
  }
  while (end > start && std::isspace(str[end - 1])) {
    end--;
  }
  return str.substr(start, end - start);
}

void McapConverter::addProfile(const std::string& profile) {
  auto tokens = split(profile, ';');
  for (const auto& token : tokens) {
    auto param_tokens = split(token, ':');
    if (param_tokens.size() != 2) {
      throw std::runtime_error("Invalid profile (wrong number of parameters): " + profile);
    }
    std::string field_str(trimSpaces(param_tokens[0]));
    std::string resolution_str(trimSpaces(param_tokens[1]));
    float resolution = 1.0f;
    if (resolution_str == "remove") {
      resolution = 0.0f;
    } else {
      // check if resolution_str can be converted to float
      try {
        resolution = std::stof(resolution_str);
      } catch (const std::invalid_argument& e) {
        throw std::runtime_error("Invalid profile (failed conversion to float): " + profile);
      }
    }
    if (field_str == "xyz") {
      profile_resolutions_["x"] = resolution;
      profile_resolutions_["y"] = resolution;
      profile_resolutions_["z"] = resolution;
    } else {
      profile_resolutions_[field_str] = resolution;
    }
  }
}

std::vector<std::pair<std::string, float>> McapConverter::getProfile() const {
  std::vector<std::pair<std::string, float>> profile;
  if (profile_resolutions_.empty()) {
    return {};
  }
  if (profile_resolutions_.count("x")) {
    profile.push_back({"x", profile_resolutions_.at("x")});
  }
  if (profile_resolutions_.count("y")) {
    profile.push_back({"y", profile_resolutions_.at("y")});
  }
  if (profile_resolutions_.count("z")) {
    profile.push_back({"z", profile_resolutions_.at("z")});
  }
  for (const auto& [field, resolution] : profile_resolutions_) {
    if (field != "x" && field != "y" && field != "z") {
      profile.push_back({field, resolution});
    }
  }
  return profile;
}
