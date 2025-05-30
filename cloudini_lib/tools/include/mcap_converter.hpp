#pragma once

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

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

  void encodePointClouds(std::filesystem::path file_out, std::optional<float> default_resolution);

  void decodePointClouds(std::filesystem::path file_out);

  std::vector<std::pair<std::string, float>> getProfile() const;

 private:
  std::shared_ptr<mcap::McapReader> reader_;
  TopicsMap topics_;

  void duplicateSchemasAndChannels(const mcap::McapReader& reader, mcap::McapWriter& writer, bool encoding);

  std::map<uint16_t, uint16_t> old_to_new_schema_id_;
  std::map<uint16_t, uint16_t> old_to_new_channel_id_;
  std::map<std::string, float> profile_resolutions_;
};
