#include <iostream>

#include "cxxopts.hpp"
#include "mcap_converter.hpp"

#define MCAP_IMPLEMENTATION
#include "mcap/reader.hpp"
#include "mcap/writer.hpp"

int main(int argc, char** argv) {
  cxxopts::Options options("cloudini_rosbag_converter", "Encode/Decode PointCloud2 messages in MCAP files");

  options.add_options()                                                 //
      ("h,help", "Print usage")                                         //
      ("y,yes", "Auto-confirm overwrite of files")                      //
      ("f,filename", "Input file name", cxxopts::value<std::string>())  //
      ("o,output", "Output file name", cxxopts::value<std::string>())   //
      ("r,resolution", "Resolution applied to floating point fields",   //
       cxxopts::value<double>()->default_value("0.001"))                //
      ("c,compress", "Convert PointCloud2 to CompressedPointCloud2")    //
      ("d,decode", "Convert CompressedPointCloud2 to PointCloud2");

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

  const std::filesystem::path input_file = parse_result["filename"].as<std::string>();

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
    std::cerr << "Error: Input file must be a .mcap file." << std::endl;
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
  std::cout << "Input file: " << input_file << std::endl;

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
      converter.encodePointClouds(output_filename, resolution);
    }
    if (decode) {
      converter.decodePointClouds(output_filename);
    }
    std::cout << "\nFile saved as: " << output_filename << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
