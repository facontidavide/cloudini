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

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/**
 * @brief Test node that demonstrates usage of point_cloud_transport plugin
 *        with manual conversion to pcl::PointCloud2
 *
 * This node subscribes to a point cloud topic using point_cloud_transport,
 * which automatically uses the Cloudini plugin for compressed topics.
 * The callback manually converts sensor_msgs::PointCloud2 to pcl::PointCloud2.
 */
class CloudiniPluginTestNode : public rclcpp::Node {
 public:
  CloudiniPluginTestNode() : Node("cloudini_plugin_test") {
    this->declare_parameter<std::string>("topic", "/points_pct");
    this->declare_parameter<std::string>("transport", "cloudini");
  }

  // Initialize point_cloud_transport (cannot be done in the constructor)
  void init() {
    std::string topic = this->get_parameter("topic").as_string();
    std::string transport = this->get_parameter("transport").as_string();

    RCLCPP_INFO(this->get_logger(), "Starting Cloudini plugin test node");
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Using transport: %s", transport.c_str());

    pct_ = std::make_shared<point_cloud_transport::PointCloudTransport>(shared_from_this());

    // Create transport hints to specify which compression to use
    auto transport_hint = std::make_shared<point_cloud_transport::TransportHints>(transport);
    subscription_ = pct_->subscribe(
        topic, 10, [this](const auto& msg) { this->pointCloudCallback(msg); }, {}, transport_hint.get());

    RCLCPP_INFO(this->get_logger(), "Cloudini plugin test node initialized successfully");
  }

 private:
  /**
   * @brief Callback invoked when a point cloud is received
   *
   * @param msg The sensor_msgs::PointCloud2 message (decompressed by plugin)
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    message_count_++;

    try {
      // Manual conversion from sensor_msgs::PointCloud2 to pcl::PointCloud2
      auto pcl_cloud = std::make_shared<pcl::PCLPointCloud2>();

      // Convert using PCL conversions utility
      pcl_conversions::toPCL(*msg, *pcl_cloud);

      // Get cloud dimensions
      uint32_t num_points = pcl_cloud->width * pcl_cloud->height;
      size_t cloud_size_bytes = pcl_cloud->data.size();

      // Print field information
      std::string fields_str;
      for (const auto& field : pcl_cloud->fields) {
        if (!fields_str.empty()) {
          fields_str += ", ";
        }
        fields_str += field.name;
      }

      // Print every 10 messages
      if (message_count_ % 10 == 0) {
        RCLCPP_INFO(
            this->get_logger(), "Received %ld point clouds. %u points (%ux%u), %.2f KB, fields: [%s]", message_count_,
            num_points, pcl_cloud->width, pcl_cloud->height, cloud_size_bytes / 1024.0, fields_str.c_str());
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to decode point cloud: %s", e.what());
    }
  }

  // Point cloud transport instance
  std::shared_ptr<point_cloud_transport::PointCloudTransport> pct_;
  // Point cloud transport subscription
  point_cloud_transport::Subscriber subscription_;
  // Message counter
  size_t message_count_ = 0;
};

int main(int argc, char** argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create and spin the test node
  auto node = std::make_shared<CloudiniPluginTestNode>();
  node->init();

  RCLCPP_INFO(node->get_logger(), "Cloudini plugin test node is running...");
  RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit");

  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception during spin: %s", e.what());
  }

  // Cleanup
  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Cloudini plugin test node shutting down");

  return 0;
}
