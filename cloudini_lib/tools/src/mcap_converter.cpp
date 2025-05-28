#include "mcap_converter.hpp"

#include <set>
#include <stdexcept>

#include "cloudini_lib/ros_msg_utils.hpp"
#include "mcap/reader.hpp"
#include "mcap/writer.hpp"
#include "message_definitions.hpp"

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

void McapConverter::encodePointClouds(std::filesystem::path file_out, float resolution) {
  if (!reader_) {
    throw std::runtime_error("McapReader is not initialized. Call open() first.");
  }

  mcap::McapWriter writer;
  mcap::McapWriterOptions writer_options(reader_->header()->profile);
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
    auto pc_info = Cloudini::readPointCloud2(raw_dds_msg);
    auto encoding_info = Cloudini::toEncodingInfo(pc_info);

    // apply resolution to all fields
    for (auto& field : encoding_info.fields) {
      if (field.type == Cloudini::FieldType::FLOAT32 || field.type == Cloudini::FieldType::FLOAT64) {
        field.resolution = resolution;
      }
    }
    // Start encoding the pointcloud data[]
    Cloudini::PointcloudEncoder pc_encoder(encoding_info);
    auto new_size = pc_encoder.encode(pc_info.data, compressed_cloud);
    compressed_cloud.resize(new_size);

    // substitute the data view
    pc_info.data = Cloudini::ConstBufferView(compressed_cloud);

    // generate the new DDS message
    auto dds_size = Cloudini::writePointCloud2(pc_info, compressed_dds_msg, true);
    compressed_dds_msg.resize(dds_size);

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