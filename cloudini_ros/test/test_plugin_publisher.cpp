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

#include <memory>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/**
 * @brief Test node that publishes point clouds using point_cloud_transport
 *
 * This node subscribes to a standard PointCloud2 topic and republishes it
 * using point_cloud_transport, which automatically creates compressed versions
 * using all available plugins (including Cloudini).
 */
class CloudiniPluginPublisherTestNode : public rclcpp::Node {
 public:
  CloudiniPluginPublisherTestNode() : Node("cloudini_plugin_publisher_test") {
    this->declare_parameter<std::string>("input_topic", "/points");
    this->declare_parameter<std::string>("output_topic", "/points_pct");
  }

  // Initialize point_cloud_transport (cannot be done in the constructor)
  void init() {
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Starting Cloudini plugin publisher test node");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());

    // Create point_cloud_transport publisher
    pct_ = std::make_shared<point_cloud_transport::PointCloudTransport>(shared_from_this());
    transport_publisher_ = pct_->advertise(output_topic, 10);

    // Create standard ROS2 subscriber
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { this->pointCloudCallback(msg); });

    RCLCPP_INFO(this->get_logger(), "Cloudini plugin publisher test node initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Point cloud transport will create compressed topics automatically");
  }

 private:
  /**
   * @brief Callback that receives point clouds and republishes via point_cloud_transport
   *
   * @param msg The incoming point cloud message
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    message_count_++;

    // Publish via point_cloud_transport
    // This will automatically create compressed versions on topics like:
    // - /points_pct (raw)
    // - /points_pct/cloudini (compressed with Cloudini)
    // - /points_pct/draco (if draco plugin is available)
    transport_publisher_.publish(msg);

    if (message_count_ % 10 == 0) {
      RCLCPP_INFO(
          this->get_logger(), "Published point cloud #%ld: %u points", message_count_, msg->width * msg->height);
    }
  }

  // Point cloud transport instance
  std::shared_ptr<point_cloud_transport::PointCloudTransport> pct_;

  // Point cloud transport publisher
  point_cloud_transport::Publisher transport_publisher_;

  // Standard ROS2 subscription for input
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  // Message counter
  size_t message_count_ = 0;
};

int main(int argc, char** argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create and initialize the publisher node
  auto node = std::make_shared<CloudiniPluginPublisherTestNode>();
  node->init();

  RCLCPP_INFO(node->get_logger(), "Cloudini plugin publisher test node is running...");
  RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit");

  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception during spin: %s", e.what());
  }

  // Cleanup
  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Cloudini plugin publisher test node shutting down");

  return 0;
}
