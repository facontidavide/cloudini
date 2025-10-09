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

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/pcl_conversion.hpp"
#include "cxxopts.hpp"

int main(int argc, char** argv) {
  cxxopts::Options options("pcd_to_cloudini_converter", "Convert PCD files to Cloudini compressed format (.cldn)");

  options.add_options()                                                                   //
      ("h,help", "Print usage")                                                           //
      ("y,yes", "Auto-confirm overwrite of files")                                        //
      ("f,filename", "Input PCD file name", cxxopts::value<std::string>())                //
      ("o,output", "Output file name (default: input_name.cldn)",                         //
       cxxopts::value<std::string>())                                                     //
      ("r,resolution", "Resolution applied to XYZ coordinates (meters)",                  //
       cxxopts::value<double>()->default_value("0.001"))                                  //
      ("i,intensity-resolution", "Resolution applied to intensity field (if present)",    //
       cxxopts::value<double>()->default_value("0.001"))                                  //
      ("e,encoding", "Encoding option: 'lossy', 'lossless', or 'none'",                   //
       cxxopts::value<std::string>()->default_value("lossy"))                             //
      ("c,compression", "Compression method: 'lz4', 'zstd', or 'none'",                   //
       cxxopts::value<std::string>()->default_value("zstd"))                              //
      ("b,batch", "Process all PCD files in a directory", cxxopts::value<std::string>())  //
      ("v,verbose", "Enable verbose output");

  auto parse_result = options.parse(argc, argv);

  if (parse_result.count("help")) {
    std::cout << options.help() << std::endl;
    std::cout << "\nExamples:\n"
              << "  Convert single file with default settings:\n"
              << "    pcd_to_cloudini_converter -f input.pcd\n\n"
              << "  Convert with custom resolution and compression:\n"
              << "    pcd_to_cloudini_converter -f input.pcd -r 0.005 -c lz4\n\n"
              << "  Batch convert all PCD files in a directory:\n"
              << "    pcd_to_cloudini_converter -b /path/to/pcd/files\n"
              << std::endl;
    return 0;
  }

  const bool batch_mode = parse_result.count("batch") > 0;
  const bool single_file = parse_result.count("filename") > 0;

  if (!batch_mode && !single_file) {
    std::cerr << "Error: Either input file (-f) or batch directory (-b) is required." << std::endl;
    std::cout << options.help() << std::endl;
    return 1;
  }

  if (batch_mode && single_file) {
    std::cerr << "Error: Cannot specify both single file and batch mode." << std::endl;
    return 1;
  }

  const double resolution_xyz = parse_result["resolution"].as<double>();
  const double resolution_intensity = parse_result["intensity-resolution"].as<double>();
  const bool auto_overwrite = parse_result.count("yes") > 0;
  const bool verbose = parse_result.count("verbose") > 0;

  // Parse encoding option
  Cloudini::EncodingOptions encoding_opt;
  const std::string encoding_str = parse_result["encoding"].as<std::string>();
  if (encoding_str == "lossy") {
    encoding_opt = Cloudini::EncodingOptions::LOSSY;
  } else if (encoding_str == "lossless") {
    encoding_opt = Cloudini::EncodingOptions::LOSSLESS;
  } else if (encoding_str == "none") {
    encoding_opt = Cloudini::EncodingOptions::NONE;
  } else {
    std::cerr << "Error: Invalid encoding option: " << encoding_str << std::endl;
    return 1;
  }

  // Parse compression option
  Cloudini::CompressionOption compression_opt;
  const std::string compression_str = parse_result["compression"].as<std::string>();
  if (compression_str == "lz4") {
    compression_opt = Cloudini::CompressionOption::LZ4;
  } else if (compression_str == "zstd") {
    compression_opt = Cloudini::CompressionOption::ZSTD;
  } else if (compression_str == "none") {
    compression_opt = Cloudini::CompressionOption::NONE;
  } else {
    std::cerr << "Error: Invalid compression option: " << compression_str << std::endl;
    return 1;
  }

  // Helper function to convert a single PCD file
  auto convert_file = [&](const std::filesystem::path& input_file, const std::filesystem::path& output_file) -> bool {
    if (!std::filesystem::exists(input_file)) {
      std::cerr << "Error: Input file does not exist: " << input_file << std::endl;
      return false;
    }

    // Check if output file exists and handle overwrite
    if (std::filesystem::exists(output_file) && !auto_overwrite) {
      std::cout << "Output file already exists: " << output_file << std::endl;
      std::cout << "Do you want to overwrite it? (y/n): ";
      char response;
      std::cin >> response;
      if (response != 'y' && response != 'Y') {
        std::cout << "Skipping file: " << input_file << std::endl;
        return false;
      }
    }

    try {
      // Encode the point cloud
      std::vector<uint8_t> serialized_cloud;
      const size_t encoded_size = Cloudini::PCLPointCloudEncode(input_file, serialized_cloud, resolution_xyz);
      // Write to output file
      std::ofstream output_stream(output_file, std::ios::binary);
      if (!output_stream) {
        std::cerr << "Error: Failed to open output file: " << output_file << std::endl;
        return false;
      }
      output_stream.write(reinterpret_cast<const char*>(serialized_cloud.data()), serialized_cloud.size());
      output_stream.close();
      return true;

    } catch (const std::exception& e) {
      std::cerr << "Error processing file " << input_file << ": " << e.what() << std::endl;
      return false;
    }
  };

  // Process files
  if (single_file) {
    const std::filesystem::path input_file = parse_result["filename"].as<std::string>();

    if (input_file.extension() != ".pcd") {
      std::cerr << "Error: Input file must be a .pcd file. Got: " << input_file << std::endl;
      return 1;
    }

    std::filesystem::path output_file;
    if (parse_result.count("output")) {
      output_file = parse_result["output"].as<std::string>();
    } else {
      output_file = input_file.parent_path() / (input_file.stem().string() + ".cldn");
    }

    // Ensure output has correct extension
    if (output_file.extension() != ".cldn") {
      output_file.replace_extension(".cldn");
    }

    if (!convert_file(input_file, output_file)) {
      return 1;
    }
    const size_t input_file_size = std::filesystem::file_size(input_file);
    const size_t output_file_size = std::filesystem::file_size(output_file);
    std::cout << "Successfully converted " << input_file << " to " << output_file << std::endl;
    std::cout << "  Size reduced from: " << input_file_size / 1024 << " Kb to " << output_file_size / 1024 << " Kb"
              << " ratio:" << static_cast<double>(output_file_size) / static_cast<double>(input_file_size) << std::endl;

  } else if (batch_mode) {
    const std::filesystem::path batch_dir = parse_result["batch"].as<std::string>();

    if (!std::filesystem::exists(batch_dir) || !std::filesystem::is_directory(batch_dir)) {
      std::cerr << "Error: Batch directory does not exist or is not a directory: " << batch_dir << std::endl;
      return 1;
    }

    std::cout << "Processing PCD files in directory: " << batch_dir << std::endl;

    int processed = 0;
    int failed = 0;

    for (const auto& entry : std::filesystem::directory_iterator(batch_dir)) {
      if (entry.path().extension() == ".pcd") {
        std::filesystem::path output_file = entry.path().parent_path() / (entry.path().stem().string() + ".cldn");

        if (convert_file(entry.path(), output_file)) {
          processed++;
        } else {
          failed++;
        }
      }
    }

    std::cout << "\nBatch processing complete:" << std::endl;
    std::cout << "  Processed: " << processed << " files" << std::endl;
    if (failed > 0) {
      std::cout << "  Failed: " << failed << " files" << std::endl;
    }
  }

  return 0;
}
