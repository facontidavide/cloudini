#include <iostream>

#include "mcap_converter.hpp"

#define MCAP_IMPLEMENTATION
#include "mcap/reader.hpp"
#include "mcap/writer.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <input_mcap_file> [output_mcap_file]" << std::endl;
    return 1;
  }

  std::filesystem::path input_file(argv[1]);

  // only mcap files are supported
  if (input_file.extension() != ".mcap") {
    std::cerr << "Error: Input file must be a .mcap file." << std::endl;
    return 1;
  }

  std::filesystem::path output_file = (argc > 2) ? argv[2] : "output.mcap";

  try {
    McapConverter converter;
    auto topics = converter.open(input_file);
    std::cout << "Topics found in the MCAP file:" << std::endl;
    for (const auto& [topic, schema] : topics) {
      std::cout << "Topic: " << topic << ", Schema: " << schema << std::endl;
    }

    converter.encodePointClouds(output_file, 0.001f);
    std::cout << "Point clouds encoded and saved to: " << output_file.string() << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}