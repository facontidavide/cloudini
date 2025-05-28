#pragma once

#include <filesystem>
#include <map>
#include <unordered_map>

namespace mcap {
class McapReader;
class McapWriter;
}  // namespace mcap

class McapConverter {
 public:
  // key: topic name, value: topic type
  using TopicsMap = std::unordered_map<std::string, std::string>;

  // open a MCAP file and return the list of topics and types
  TopicsMap open(std::filesystem::path file_in);

  void encodePointClouds(std::filesystem::path file_out, float reolution = 0.001f);

  void decodePointClouds(std::filesystem::path file_out);

 private:
  std::shared_ptr<mcap::McapReader> reader_;
  TopicsMap topics_;

  void duplicateSchemasAndChannels(const mcap::McapReader& reader, mcap::McapWriter& writer, bool encoding);

  std::map<uint16_t, uint16_t> old_to_new_schema_id_;
  std::map<uint16_t, uint16_t> old_to_new_channel_id_;
};
