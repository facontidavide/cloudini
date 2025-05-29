// Copyright (c) 2025 Davide Faconti
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <stdio.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_ros/conversion_utils.hpp"

struct StatsData
{
  int32_t total_time_usec = 0;
  double total_ratio = 0;
};

struct Statistics
{
  int count = 0;
  StatsData lz4_only;
  StatsData zstd_only;
  StatsData lossy_lz4;
  StatsData lossy_zstd;
};

void compress(
  const sensor_msgs::msg::PointCloud2 & msg, const Cloudini::EncodingInfo & encoding_info,
  StatsData & stats)
{
  Cloudini::PointcloudEncoder encoder(encoding_info);
  std::vector<uint8_t> compressed_data;
  Cloudini::ConstBufferView input(msg.data.data(), msg.data.size());

  auto t1 = std::chrono::high_resolution_clock::now();
  encoder.encode(input, compressed_data);
  auto t2 = std::chrono::high_resolution_clock::now();

  stats.total_time_usec += std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
  stats.total_ratio += static_cast<double>(compressed_data.size()) /
    static_cast<double>(msg.data.size());
}

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  rosbag2_cpp::Reader reader;
  reader.open(argv[1]);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;

  std::unordered_map<std::string, Statistics> statistics_by_topic;

  for (const auto & it : reader.get_all_topics_and_types()) {
    if (it.type == "sensor_msgs/msg/PointCloud2") {
      std::cout << "Found PointCloud2 topic: " << it.name << std::endl;
      statistics_by_topic[it.name] = Statistics();
    }
  }

  while (rclcpp::ok() && reader.has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

    if (!statistics_by_topic.contains(msg->topic_name)) {
      continue;
    }

    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    serialization.deserialize_message(&serialized_msg, ros_msg.get());

    auto & statistics = statistics_by_topic[msg->topic_name];
    // 100 microns resolution!
    Cloudini::EncodingInfo encoding_info = Cloudini::ConvertToEncodingInfo(*ros_msg, 0.0001F);

    if (statistics.count == 0) {
      std::cout << "Topic [" << msg->topic_name << "] has fields:\n";
      for (const auto & field : encoding_info.fields) {
        std::cout << " - " << field.name << " (" << Cloudini::ToString(field.type) << ")\n";
      }
      std::cout << std::endl;
    }

    encoding_info.encoding_opt = Cloudini::EncodingOptions::NONE;
    encoding_info.compression_opt = Cloudini::CompressionOption::LZ4;
    compress(*ros_msg, encoding_info, statistics.lz4_only);

    encoding_info.encoding_opt = Cloudini::EncodingOptions::NONE;
    encoding_info.compression_opt = Cloudini::CompressionOption::ZSTD;
    compress(*ros_msg, encoding_info, statistics.zstd_only);

    encoding_info.encoding_opt = Cloudini::EncodingOptions::LOSSY;
    encoding_info.compression_opt = Cloudini::CompressionOption::LZ4;
    compress(*ros_msg, encoding_info, statistics.lossy_lz4);

    encoding_info.encoding_opt = Cloudini::EncodingOptions::LOSSY;
    encoding_info.compression_opt = Cloudini::CompressionOption::ZSTD;
    compress(*ros_msg, encoding_info, statistics.lossy_zstd);

    statistics.count++;
  }

  //------------------------------------------------------------
  for (const auto & [topic, stat] : statistics_by_topic) {
    double dcount = static_cast<double>(stat.count);
    std::cout << "Topic: " << topic << std::endl;
    std::cout << "  Count: " << stat.count << std::endl;
    printf(
        "  [LZ4 only]      ratio: %.2f time (usec): %d\n", stat.lz4_only.total_ratio / dcount,
        stat.lz4_only.total_time_usec / stat.count);
    printf(
        "  [ZSTD only]     ratio: %.2f time (usec): %d\n", stat.zstd_only.total_ratio / dcount,
        stat.zstd_only.total_time_usec / stat.count);

    printf(
        "  [Cloudini-LZ4]  ratio: %.2f time (usec): %d\n", stat.lossy_lz4.total_ratio / dcount,
        stat.lossy_lz4.total_time_usec / stat.count);
    printf(
        "  [Cloudini-ZSTD] ratio: %.2f time (usec): %d\n", stat.lossy_zstd.total_ratio / dcount,
        stat.lossy_zstd.total_time_usec / stat.count);
    std::cout << std::endl;
  }
  return 0;
}
