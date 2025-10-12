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

#include <pcl/conversions.h>
#include <pcl/point_types.h>

#include <cloudini_ros/cloudini_subscriber_pcl.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Test node that demonstrates usage of CloudiniSubscriberPCL
 *
 * This node subscribes to a compressed point cloud topic and processes
 * the received point clouds using PCL.
 */
class CloudiniSubscriberTestNode : public rclcpp::Node {
 public:
  CloudiniSubscriberTestNode() : Node("cloudini_subscriber_test") {
    this->declare_parameter<std::string>("topic", "/points/compressed");
    std::string topic = this->get_parameter("topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Starting CloudiniSubscriberPCL test node");
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", topic.c_str());

    // Create CloudiniSubscriberPCL with callback using raw pointer (not shared_from_this in constructor)
    subscriber_ = std::make_shared<cloudini_ros::CloudiniSubscriberPCL>(
        this, topic, std::bind(&CloudiniSubscriberTestNode::pointCloudCallback, this, std::placeholders::_1),
        rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "CloudiniSubscriberPCL test node initialized successfully");
  }

 private:
  void pointCloudCallback(const pcl::PCLPointCloud2::Ptr& cloud);

  std::shared_ptr<cloudini_ros::CloudiniSubscriberPCL> subscriber_;
  size_t message_count_ = 0;
};

void CloudiniSubscriberTestNode::pointCloudCallback(const pcl::PCLPointCloud2::Ptr& cloud) {
  message_count_++;

  // Get cloud dimensions
  uint32_t num_points = cloud->width * cloud->height;
  size_t cloud_size_bytes = cloud->data.size();
  bool is_organized = cloud->height > 1;

  // Print field information
  std::string fields_str;
  for (const auto& field : cloud->fields) {
    if (!fields_str.empty()) {
      fields_str += ", ";
    }
    fields_str += field.name;
  }

  // Print every 10 messages
  if (message_count_ % 10 == 0) {
    RCLCPP_INFO(
        this->get_logger(), "Received %ld point clouds. %u points (%ux%u), %.2f KB, fields: [%s]", message_count_,
        num_points, cloud->width, cloud->height, cloud_size_bytes / 1024.0, fields_str.c_str());
  }
}

int main(int argc, char** argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create and spin the test node
  auto node = std::make_shared<CloudiniSubscriberTestNode>();

  RCLCPP_INFO(node->get_logger(), "CloudiniSubscriberPCL test node is running...");
  RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit");

  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception during spin: %s", e.what());
  }

  // Cleanup
  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "CloudiniSubscriberPCL test node shutting down");

  return 0;
}
