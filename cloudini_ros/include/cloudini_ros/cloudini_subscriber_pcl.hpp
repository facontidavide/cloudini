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

#ifndef CLOUDINI_ROS__CLOUDINI_SUBSCRIBER_PCL_HPP_
#define CLOUDINI_ROS__CLOUDINI_SUBSCRIBER_PCL_HPP_

#include <pcl/PCLPointCloud2.h>

#include <deque>
#include <functional>
#include <memory>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <string>

namespace cloudini_ros {

/**
 * @brief ROS2 subscriber class that receives CompressedPointCloud2 messages
 *        and converts them directly to pcl::PCLPointCloud2 format.
 *
 * This class provides a convenient way to subscribe to Cloudini-compressed
 * point cloud topics and receive the decompressed data as PCL point clouds,
 * bypassing the intermediate ROS sensor_msgs::PointCloud2 format.
 */
class CloudiniSubscriberPCL {
 public:
  using CallbackType = std::function<void(const pcl::PCLPointCloud2::Ptr&)>;

  /**
   * @brief Constructor for CloudiniSubscriberPCL
   *
   * @param node Shared pointer to the ROS2 node
   * @param topic_name Name of the topic to subscribe to (CompressedPointCloud2)
   * @param callback User callback function that will be invoked with decompressed PCL cloud
   * @param qos_profile QoS profile for the subscription (default: system default)
   */
  CloudiniSubscriberPCL(
      rclcpp::Node::SharedPtr node, const std::string& topic_name, CallbackType callback,
      const rclcpp::QoS& qos_profile = rclcpp::QoS(10));

  /**
   * @brief Constructor overload using raw node pointer
   *
   * @param node Raw pointer to the ROS2 node
   * @param topic_name Name of the topic to subscribe to (CompressedPointCloud2)
   * @param callback User callback function that will be invoked with decompressed PCL cloud
   * @param qos_profile QoS profile for the subscription (default: system default)
   */
  CloudiniSubscriberPCL(
      rclcpp::Node* node, const std::string& topic_name, CallbackType callback,
      const rclcpp::QoS& qos_profile = rclcpp::QoS(10));

  /**
   * @brief Destructor - cleans up object pool
   */
  ~CloudiniSubscriberPCL();

  /**
   * @brief Get the underlying ROS2 subscription object
   *
   * @return Shared pointer to the subscription
   */
  rclcpp::SubscriptionBase::SharedPtr getSubscription() const {
    return subscription_;
  }

  /**
   * @brief Get the topic name this subscriber is listening to
   *
   * @return Topic name string
   */
  std::string getTopicName() const;

 private:
  /**
   * @brief Internal callback that handles decompression from raw DDS message
   *
   * @param msg Received serialized message
   */
  void messageCallback(std::shared_ptr<rclcpp::SerializedMessage> msg);

  /**
   * @brief Acquire a PCL cloud object from the pool (or create new if pool is empty)
   *
   * @return Shared pointer to PCL cloud with custom deleter that returns to pool
   */
  pcl::PCLPointCloud2::Ptr acquireCloudFromPool();

  // ROS2 generic subscription (for efficient raw DDS message handling)
  rclcpp::GenericSubscription::SharedPtr subscription_;

  // User-provided callback
  CallbackType user_callback_;

  // Node pointer for logging
  rclcpp::Node* node_;

  // Object pool for PCL clouds (avoids repeated allocations)
  std::deque<pcl::PCLPointCloud2*> cloud_pool_;

  std::mutex pool_mutex_;

  // Maximum pool size to prevent unbounded growth
  static constexpr size_t MAX_POOL_SIZE = 4;
};

}  // namespace cloudini_ros

#endif  // CLOUDINI_ROS__CLOUDINI_SUBSCRIBER_PCL_HPP_
