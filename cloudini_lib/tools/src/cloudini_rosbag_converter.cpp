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
#include "cxxopts.hpp"
#include "mcap_converter.hpp"

int main(int argc, char** argv) {
  cxxopts::Options options("cloudini_rosbag_converter", "Encode/Decode PointCloud2 messages in MCAP files");

  options.add_options()                                                 //
      ("h,help", "Print usage")                                         //
      ("y,yes", "Auto-confirm overwrite of files")                      //
      ("f,filename", "Input file name", cxxopts::value<std::string>())  //
      ("o,output", "Output file name", cxxopts::value<std::string>())   //
      ("r,resolution", "Resolution applied to floating point fields",   //
       cxxopts::value<double>()->default_value("0.001"))                //
      ("profile", "Apply a profile to encoding. See '--help' for details. It can be a path to a file or a string",
       cxxopts::value<std::string>())                                 //
      ("c,compress", "Convert PointCloud2 to CompressedPointCloud2")  //
      ("d,decode", "Convert CompressedPointCloud2 to PointCloud2")    //
      ("s,stats", "Print compression statistics")                     //
      ("m,method", "Compression method to use when writing data back to mcap ('lz4' or 'zstd', 'none')",
       cxxopts::value<std::string>()->default_value("zstd"));

  auto parse_result = options.parse(argc, argv);

  if (parse_result.count("help")) {
    std::cout << options.help() << std::endl;

    std::cout << "During encoding, you can specify a custom profile, where you specify the resolution of each fields "
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

  // only mcap files are supported
  if (input_file.extension() != ".mcap") {
    std::cerr << "Error: Input file must be a .mcap file. String was: " << input_file << std::endl;
    return 1;
  }

  std::string output_filename = input_file.stem().string() + (encode ? "_encoded.mcap" : "_decoded.mcap");
  if (parse_result.count("output")) {
    output_filename = parse_result["output"].as<std::string>();
  }

  // if filename doesn't have .mcap extension, add it
  if (std::filesystem::path(output_filename).extension() != ".mcap") {
    output_filename += ".mcap";
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
  std::cout << "----------------------\nInput file: " << input_file << std::endl;

  // Parse the mcap writer compression method
  Cloudini::CompressionOption mcap_writer_compression;

  // clang-format off
  const auto compression_options_map = std::unordered_map<std::string, Cloudini::CompressionOption>{
      {"none", Cloudini::CompressionOption::NONE},
      {"lz4", Cloudini::CompressionOption::LZ4},
      {"zstd", Cloudini::CompressionOption::ZSTD}};
  // clang-format on

  std::string compression_method = parse_result["method"].as<std::string>();
  if (!compression_options_map.contains(compression_method)) {
    std::cerr << "Error: Invalid compression method: " << compression_method << std::endl;
    return 1;
  }
  mcap_writer_compression = compression_options_map.at(compression_method);
  std::cout << "Using compression method: " << compression_method << std::endl;

  int compressed_pointclouds_count = 0;
  int regular_pointclouds_count = 0;

  try {
    McapConverter converter;
    auto topics = converter.open(input_file);
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

    std::cout << "\n started processing MCAP file: " << input_file << std::endl;

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
