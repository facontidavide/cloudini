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

#include <cloudini_ros/conversion_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/**
 * @brief Example node that compresses PointCloud2 using the convenience API.
 *
 * Subscribes to a standard sensor_msgs/PointCloud2 topic, compresses it
 * using Cloudini::SerializeCompressedPointCloud2, and publishes the result
 * as a serialized CompressedPointCloud2 via a generic publisher.
 *
 * No topic_converter node needed.
 */
class DirectCompressionPublisher : public rclcpp::Node {
 public:
  DirectCompressionPublisher() : Node("direct_compression_publisher") {
    this->declare_parameter<std::string>("input_topic", "/points");
    this->declare_parameter<std::string>("output_topic", "/points/compressed");
    this->declare_parameter<double>("resolution", 0.001);

    const std::string input_topic = this->get_parameter("input_topic").as_string();
    const std::string output_topic = this->get_parameter("output_topic").as_string();
    resolution_ = static_cast<float>(this->get_parameter("resolution").as_double());

    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to:  %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Resolution:     %.4f", resolution_);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, rclcpp::QoS(10), std::bind(&DirectCompressionPublisher::callback, this, std::placeholders::_1));

    publisher_ = this->create_generic_publisher(
        output_topic, "point_cloud_interfaces/msg/CompressedPointCloud2", rclcpp::QoS(10));
  }

  ~DirectCompressionPublisher() {
    // Null out the borrowed pointer before SerializedMessage destructor runs
    output_message_.get_rcl_serialized_message().buffer = nullptr;
    output_message_.get_rcl_serialized_message().buffer_length = 0;
  }

 private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (publisher_->get_subscription_count() == 0) {
      return;
    }

    const size_t original_size = msg->data.size();

    Cloudini::SerializeCompressedPointCloud2(*msg, resolution_, output_buffer_);

    // Zero-copy publish: point SerializedMessage at our buffer
    output_message_.get_rcl_serialized_message().buffer = output_buffer_.data();
    output_message_.get_rcl_serialized_message().buffer_length = output_buffer_.size();
    publisher_->publish(output_message_);

    tot_original_size_ += original_size;
    tot_compressed_size_ += output_buffer_.size();
    message_count_++;

    if (message_count_ % 20 == 0) {
      const double ratio = static_cast<double>(tot_compressed_size_) / tot_original_size_;
      RCLCPP_INFO(this->get_logger(), "Compressed %zu messages, average ratio: %.2f", message_count_, ratio);
      tot_original_size_ = 0;
      tot_compressed_size_ = 0;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::GenericPublisher::SharedPtr publisher_;

  std::vector<uint8_t> output_buffer_;
  rclcpp::SerializedMessage output_message_;
  float resolution_ = 0.001f;

  size_t message_count_ = 0;
  uint64_t tot_original_size_ = 0;
  uint64_t tot_compressed_size_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectCompressionPublisher>();
  RCLCPP_INFO(node->get_logger(), "Direct compression publisher running. Press Ctrl+C to exit.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
