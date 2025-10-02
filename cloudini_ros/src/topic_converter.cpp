// Copyright (c) 2025 Davide Faconti
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <cloudini_lib/cloudini.hpp>
#include <cloudini_lib/ros_msg_utils.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/*
 * This node converts compressed point cloud messages from the
 * `point_cloud_interfaces/msg/CompressedPointCloud2` format to the
 * `sensor_msgs/msg/PointCloud2` format.
 *
 * It is BRUTALLY efficient, because we reade directly the RAW DDS message,
 * and write the output message without any intermediate copies.
 *
 * This means less CPU and less latency.
 */
class CloudiniPointcloudConverter : public rclcpp::Node {
 public:
  CloudiniPointcloudConverter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void callback(std::shared_ptr<rclcpp::SerializedMessage> msg);

  ~CloudiniPointcloudConverter() {
    // bypass the deleter
    output_message_.get_rcl_serialized_message().buffer = nullptr;
    output_message_.get_rcl_serialized_message().buffer_length = 0;
  }

 private:
  // generic subscriber for compressed point cloud messages
  rclcpp::GenericSubscription::SharedPtr point_cloud_subscriber_;

  // generic publisher for sensor_msgs/msg/PointCloud2 (but... raw DDS message)
  rclcpp::GenericPublisher::SharedPtr point_cloud_publisher_;

  // callback for point cloud messages
  void point_cloud_callback(const rclcpp::SerializedMessage& serialized_msg);

  std::vector<uint8_t> output_raw_message_;
  rclcpp::SerializedMessage output_message_;
};
//-----------------------------------------------------

rclcpp::QoS adapt_request_to_offers(
    const std::string& topic_name, const std::vector<rclcpp::TopicEndpointInfo>& endpoints) {
  rclcpp::QoS request_qos(rmw_qos_profile_default.depth);

  if (endpoints.empty()) {
    return request_qos;
  }
  size_t reliability_reliable_endpoints_count = 0;
  size_t durability_transient_local_endpoints_count = 0;
  for (const auto& endpoint : endpoints) {
    const auto& profile = endpoint.qos_profile().get_rmw_qos_profile();
    if (profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      reliability_reliable_endpoints_count++;
    }
    if (profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
      durability_transient_local_endpoints_count++;
    }
  }
  // Policy: reliability
  if (reliability_reliable_endpoints_count == endpoints.size()) {
    request_qos.reliable();
  } else {
    request_qos.best_effort();
  }
  // Policy: durability
  // If all publishers offer transient_local, we can request it and receive latched messages
  if (durability_transient_local_endpoints_count == endpoints.size()) {
    request_qos.transient_local();
  } else {
    request_qos.durability_volatile();
  }
  return request_qos;
}
//-----------------------------------------------------

CloudiniPointcloudConverter::CloudiniPointcloudConverter(const rclcpp::NodeOptions& options)
    : rclcpp::Node("cloudini_pointcloud_converter", options) {
  // Declare parameters for input and output topics
  this->declare_parameter<std::string>("topic_input", "input_points");
  this->declare_parameter<std::string>("topic_output", "output_points");

  // read parameters
  const std::string input_topic = this->get_parameter("topic_input").as_string();
  const std::string output_topic = this->get_parameter("topic_output").as_string();

  // Initialize point cloud type support
  auto publisher_info = this->get_publishers_info_by_topic(input_topic);
  auto detected_qos = adapt_request_to_offers(input_topic, publisher_info);

  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback =
      std::bind(&CloudiniPointcloudConverter::callback, this, std::placeholders::_1);

  const std::string compressed_topic_type = "point_cloud_interfaces/msg/CompressedPointCloud2";
  const std::string pointcloud_topic_type = "sensor_msgs/msg/PointCloud2";

  // Create a generic subscriber for point cloud messages
  point_cloud_subscriber_ = this->create_generic_subscription(
      input_topic,            //
      compressed_topic_type,  // "point_cloud_interfaces/msg/CompressedPointCloud2"
      detected_qos,           //
      callback);

  // Create a generic publisher for point cloud messages
  point_cloud_publisher_ = this->create_generic_publisher(output_topic, pointcloud_topic_type, detected_qos);
}

void CloudiniPointcloudConverter::callback(std::shared_ptr<rclcpp::SerializedMessage> msg) {
  // STEP 1: convert the buffer to a ConstBufferView (this is not a copy)
  const auto& input_msg = msg->get_rcl_serialized_message();
  const Cloudini::ConstBufferView raw_dds_msg(input_msg.buffer, input_msg.buffer_length);

  // STEP 2: extract information from the raw DDS message
  const auto pc_info = Cloudini::readPointCloud2Message(raw_dds_msg);
  Cloudini::decompressAndWritePointCloud2Message(pc_info, output_raw_message_);

  // STEP 3: publish the output message
  output_message_.get_rcl_serialized_message().buffer_length = output_raw_message_.size();
  output_message_.get_rcl_serialized_message().buffer = output_raw_message_.data();
  point_cloud_publisher_->publish(output_message_);

  static int count = 0;
  if (count++ % 100 == 0) {
    RCLCPP_INFO(this->get_logger(), "Received and converted point cloud message: %d", count);
  }
}

int main(int argc, char** argv) {
  // Initialize ROS2 node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<CloudiniPointcloudConverter>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
