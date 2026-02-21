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

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cxxopts.hpp"
#include "mcap_converter.hpp"

namespace {

void replaceAll(std::string& str, const std::string& from, const std::string& to) {
  size_t pos = 0;
  while ((pos = str.find(from, pos)) != std::string::npos) {
    str.replace(pos, from.size(), to);
    pos += to.size();
  }
}

void generateMetadataYaml(
    const std::filesystem::path& input_metadata, const std::filesystem::path& output_dir,
    const std::string& new_mcap_filename, bool encoding) {
  std::ifstream in(input_metadata);
  std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

  // Replace topic types
  const std::string old_type =
      encoding ? "sensor_msgs/msg/PointCloud2" : "point_cloud_interfaces/msg/CompressedPointCloud2";
  const std::string new_type =
      encoding ? "point_cloud_interfaces/msg/CompressedPointCloud2" : "sensor_msgs/msg/PointCloud2";

  replaceAll(content, "type: " + old_type, "type: " + new_type);

  // Replace .mcap filename references (in relative_file_paths and files sections)
  // Extract the original mcap filename from the "relative_file_paths:" section
  size_t rfp_pos = content.find("relative_file_paths:");
  if (rfp_pos != std::string::npos) {
    size_t dash_pos = content.find("- ", rfp_pos);
    if (dash_pos != std::string::npos) {
      size_t name_start = dash_pos + 2;
      size_t name_end = content.find('\n', name_start);
      std::string old_mcap_name = content.substr(name_start, name_end - name_start);
      // Trim trailing whitespace/CR
      while (!old_mcap_name.empty() && (old_mcap_name.back() == ' ' || old_mcap_name.back() == '\r')) {
        old_mcap_name.pop_back();
      }
      if (!old_mcap_name.empty()) {
        replaceAll(content, old_mcap_name, new_mcap_filename);
      }
    }
  }

  std::filesystem::path output_metadata = output_dir / "metadata.yaml";
  std::ofstream out(output_metadata);
  out << content;
  std::cout << "Metadata file saved as: " << output_metadata << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  cxxopts::Options options("cloudini_rosbag_converter", "Encode/Decode PointCloud2 messages in MCAP files");

  options.add_options()                                                                          //
      ("h,help", "Print usage")                                                                  //
      ("y,yes", "Auto-confirm overwrite of files")                                               //
      ("f,filename", "Input .mcap file or ROS2 bag directory", cxxopts::value<std::string>())    //
      ("o,output", "Output file name (default: auto-generated)", cxxopts::value<std::string>())  //
      ("r,resolution", "Resolution applied to floating point fields",                            //
       cxxopts::value<double>()->default_value("0.001"))                                         //
      ("profile", "Apply a profile to encoding. See '--help' for details. It can be a path to a file or a string",
       cxxopts::value<std::string>())                                 //
      ("c,compress", "Convert PointCloud2 to CompressedPointCloud2")  //
      ("d,decode", "Convert CompressedPointCloud2 to PointCloud2")    //
      ("s,stats", "Print compression statistics")                     //
      ("m,method", "Compression method to use when writing data back to mcap ('zstd', 'none')",
       cxxopts::value<std::string>()->default_value("zstd"));

  auto parse_result = options.parse(argc, argv);

  if (parse_result.count("help")) {
    std::cout << options.help() << std::endl;

    std::cout << "Input (-f) can be a .mcap file or a ROS2 bag directory.\n"
              << "When a bag directory is given (or a .mcap with a sibling metadata.yaml),\n"
              << "a transformed metadata.yaml is generated alongside the output .mcap.\n"
              << "Default output is placed in a new sibling directory to avoid overwriting\n"
              << "the original bag (e.g. my_bag/ -> my_bag_encoded/).\n"
              << "\nDuring encoding, you can specify a custom profile, where you specify the resolution of each fields "
                 "or even remove entire fields. Example:\n\n"
              << "  --profile \"xyz:0.001; intensity:0.1; timestamp:remove\"\n"
              << "\nThis means:\n"
              << " - x,y,z fields will be encoded with a resolution of 0.001 meters (1 mm)\n"
              << " - intensity field will be encoded with a resolution of 0.1\n"
              << " - timestamp field will be removed\n"
              << std::endl;

    return 0;
  }

  if (!parse_result.count("filename")) {
    std::cerr << "Error: Input file name is required." << std::endl;
    std::cout << options.help() << std::endl;
    return 1;
  }

  auto filename_str = parse_result["filename"].as<std::string>();
  if (filename_str.empty()) {
    std::cerr << "Input file name is required." << std::endl;
    return 1;
  }

  const std::filesystem::path input_file = filename_str;

  const double resolution = parse_result["resolution"].as<double>();
  const bool encode = parse_result.count("compress");
  const bool decode = parse_result.count("decode");

  if (encode && decode) {
    std::cerr << "Error: Cannot specify both --compress and --decode options." << std::endl;
    return 1;
  }
  if (!encode && !decode) {
    std::cerr << "Error: Must specify either --compress or --decode option." << std::endl;
    return 1;
  }

  // Resolve input: accept a bag directory or a bare .mcap file
  std::filesystem::path mcap_file;
  std::filesystem::path input_metadata;  // empty if no metadata.yaml found

  if (std::filesystem::is_directory(input_file)) {
    // Find metadata.yaml
    auto meta = input_file / "metadata.yaml";
    if (!std::filesystem::exists(meta)) {
      std::cerr << "Error: Directory does not contain metadata.yaml: " << input_file << std::endl;
      return 1;
    }
    input_metadata = meta;

    // Find .mcap files
    std::vector<std::filesystem::path> mcap_files;
    for (const auto& entry : std::filesystem::directory_iterator(input_file)) {
      if (entry.path().extension() == ".mcap") {
        mcap_files.push_back(entry.path());
      }
    }
    if (mcap_files.empty()) {
      std::cerr << "Error: Directory does not contain any .mcap file: " << input_file << std::endl;
      return 1;
    }
    if (mcap_files.size() > 1) {
      std::cerr << "Error: Directory contains multiple .mcap files. Please specify the file directly." << std::endl;
      return 1;
    }
    mcap_file = mcap_files[0];
  } else if (input_file.extension() == ".mcap") {
    mcap_file = input_file;
    // Check for sibling metadata.yaml
    auto meta = input_file.parent_path() / "metadata.yaml";
    if (std::filesystem::exists(meta)) {
      input_metadata = meta;
    }
  } else {
    std::cerr << "Error: Input must be a .mcap file or a bag directory: " << input_file << std::endl;
    return 1;
  }

  // Determine output filename
  std::string output_filename;
  if (parse_result.count("output")) {
    output_filename = parse_result["output"].as<std::string>();
  } else if (!input_metadata.empty()) {
    // When input has metadata.yaml, default output goes to a new sibling directory
    auto input_dir = std::filesystem::canonical(input_metadata.parent_path());
    auto suffix = encode ? "_encoded" : "_decoded";
    auto output_dir = input_dir.parent_path() / (input_dir.filename().string() + suffix);
    output_filename = (output_dir / (mcap_file.stem().string() + suffix + ".mcap")).string();
  } else {
    output_filename = mcap_file.stem().string() + (encode ? "_encoded.mcap" : "_decoded.mcap");
  }

  // if filename doesn't have .mcap extension, add it
  if (std::filesystem::path(output_filename).extension() != ".mcap") {
    output_filename += ".mcap";
  }

  // Safety check: prevent overwriting original metadata.yaml
  if (!input_metadata.empty()) {
    auto output_dir = std::filesystem::path(output_filename).parent_path();
    if (output_dir.empty()) {
      output_dir = std::filesystem::current_path();
    }
    auto input_dir = std::filesystem::canonical(input_metadata.parent_path());
    // Canonicalize output_dir only if it already exists
    if (std::filesystem::exists(output_dir)) {
      output_dir = std::filesystem::canonical(output_dir);
    }
    if (output_dir == input_dir) {
      std::cerr << "Error: Output would be in the same directory as the input bag.\n"
                << "This would overwrite the original metadata.yaml.\n"
                << "Please specify an output in a different directory with -o.\n";
      return 1;
    }
  }

  // Create output directory if it doesn't exist
  auto output_parent = std::filesystem::path(output_filename).parent_path();
  if (!output_parent.empty() && !std::filesystem::exists(output_parent)) {
    std::filesystem::create_directories(output_parent);
  }

  if (decode && parse_result.count("profile")) {
    std::cerr << "The option --profile is used only for compression" << std::endl;
    return 1;
  }

  // if file exists already, ask the user to confirm overwriting
  if (std::filesystem::exists(output_filename) && !parse_result.count("yes")) {
    std::cout << "Output file already exists: " << output_filename << std::endl;
    std::cout << "Do you want to overwrite it? (y/n): ";
    char response;
    std::cin >> response;
    if (response != 'y' && response != 'Y') {
      std::cout << "Operation cancelled." << std::endl;
      return 0;
    }
  }
  std::cout << "----------------------\nInput file: " << mcap_file << std::endl;

  // Parse the mcap writer compression method
  Cloudini::CompressionOption mcap_writer_compression;

  // clang-format off
  const auto compression_options_map = std::unordered_map<std::string, Cloudini::CompressionOption>{
      {"none", Cloudini::CompressionOption::NONE},
      {"zstd", Cloudini::CompressionOption::ZSTD}};
  // clang-format on

  std::string compression_method = parse_result["method"].as<std::string>();
  if (!compression_options_map.contains(compression_method)) {
    std::cerr << "Error: Invalid compression method: " << compression_method << std::endl;
    std::cerr << "The application only supports 'zstd' and 'none'" << std::endl;
    return 1;
  }
  mcap_writer_compression = compression_options_map.at(compression_method);
  std::cout << "Using compression method: " << compression_method << std::endl;

  int compressed_pointclouds_count = 0;
  int regular_pointclouds_count = 0;

  try {
    McapConverter converter;
    auto topics = converter.open(mcap_file);
    std::cout << "\nTopics containing Point Clouds found in the MCAP file:" << std::endl;
    for (const auto& [topic, schema] : topics) {
      std::cout << "Topic: " << topic << ", Schema: " << schema << std::endl;
      if (schema == "sensor_msgs/msg/PointCloud2") {
        regular_pointclouds_count++;
      } else if (schema == "point_cloud_interfaces/msg/CompressedPointCloud2") {
        compressed_pointclouds_count++;
      }
    }
    if (regular_pointclouds_count == 0 && compressed_pointclouds_count == 0) {
      std::cerr << "No PointCloud2 or CompressedPointCloud2 topics found in the MCAP file. Nothing to do" << std::endl;
      return 0;
    }

    if (regular_pointclouds_count == 0 && encode) {
      std::cerr << "No regular pointclouds to encode. Did you intend to use option \"-d\"?" << std::endl;
      return 1;
    }
    if (compressed_pointclouds_count == 0 && decode) {
      std::cerr << "No compressed pointclouds to encode. Did you intend to use option \"-c\"?" << std::endl;
      return 1;
    }

    std::cout << "\n started processing MCAP file: " << mcap_file << std::endl;

    if (encode) {
      if (parse_result.count("profile")) {
        std::string profile_str = parse_result["profile"].as<std::string>();
        // check if it is a file or a string
        if (std::filesystem::exists(profile_str)) {
          std::ifstream file(profile_str);
          std::string profile;
          file >> profile;
          converter.addProfile(profile);
        } else {
          converter.addProfile(profile_str);
        }
        auto profile_resolutions = converter.getProfile();
        std::cout << "\nApplied profile resolutions: \n";
        for (const auto& [field, resolution] : profile_resolutions) {
          std::cout << "  " << field << ": " << resolution << std::endl;
        }
      }
      converter.encodePointClouds(output_filename, resolution, mcap_writer_compression);
    }
    if (decode) {
      converter.decodePointClouds(output_filename, mcap_writer_compression);
    }
    std::cout << "\nFile saved as: " << output_filename << std::endl;

    // Generate transformed metadata.yaml if input had one
    if (!input_metadata.empty()) {
      auto output_path = std::filesystem::path(output_filename);
      auto output_dir = output_path.parent_path();
      if (output_dir.empty()) {
        output_dir = std::filesystem::current_path();
      }
      generateMetadataYaml(input_metadata, output_dir, output_path.filename().string(), encode);
    }

    if (parse_result.count("stats")) {
      std::cout << "\n";
      converter.printStatistics();
    }

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
