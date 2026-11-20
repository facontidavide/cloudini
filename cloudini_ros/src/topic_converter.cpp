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

#include <chrono>
#include <cloudini_lib/cloudini.hpp>
#include <cloudini_lib/ros_msg_utils.hpp>
#include <optional>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
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
  // Subscribes with the QoS that the publishers of the input topic offer, once there is one, and subscribes
  // again when what they offer changes: a reader's QoS cannot change, and a RELIABLE reader receives nothing
  // from a BEST_EFFORT publisher. Called at start and by a timer.
  void adaptToPublishers();

  std::string input_topic_;
  std::string input_topic_type_;
  std::string output_topic_;
  std::string output_topic_type_;
  rclcpp::TimerBase::SharedPtr graph_poll_timer_;
  std::optional<rclcpp::QoS> input_qos_;  // of the current subscription

  // generic subscriber for compressed point cloud messages
  rclcpp::GenericSubscription::SharedPtr point_cloud_subscriber_;

  // generic publisher for sensor_msgs/msg/PointCloud2 (but... raw DDS message)
  rclcpp::GenericPublisher::SharedPtr point_cloud_publisher_;

  // callback for point cloud messages
  void point_cloud_callback(const rclcpp::SerializedMessage& serialized_msg);

  std::vector<uint8_t> output_raw_message_;
  rclcpp::SerializedMessage output_message_;
  bool compressing_ = true;
  double resolution_ = 0.001;  // 1mm
  uint8_t encoding_version_ = Cloudini::kEncodingVersion;
  // one encoder for the whole stream (V6 reuses its per-chunk choices between clouds)
  Cloudini::PointcloudEncoderCache encoder_cache_;

  uint64_t tot_original_size = 0;
  uint64_t tot_compressed_size = 0;
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
    : rclcpp::Node("cloudini_pointcloud_converter", rclcpp::NodeOptions(options).use_intra_process_comms(true)) {
  // Declare parameters for input and output topics
  this->declare_parameter<bool>("compressing", true);
  this->declare_parameter<std::string>("topic_input", "/points");
  this->declare_parameter<std::string>("topic_output", "");
  this->declare_parameter<double>("resolution", 0.001);
  this->declare_parameter<bool>("log_compression_stats", true);
  // 6 (default); 5 for decoders from 1.3.1 and earlier
  this->declare_parameter<int>("encoding_version", Cloudini::kEncodingVersion);

  // read parameters
  compressing_ = this->get_parameter("compressing").as_bool();
  resolution_ = this->get_parameter("resolution").as_double();
  const int64_t encoding_version = this->get_parameter("encoding_version").as_int();
  if (encoding_version < Cloudini::kMinEncodingVersion || encoding_version > Cloudini::kMaxEncodingVersion) {
    RCLCPP_ERROR(
        this->get_logger(), "encoding_version must be %d to %d (got %ld)", Cloudini::kMinEncodingVersion,
        Cloudini::kMaxEncodingVersion, static_cast<long>(encoding_version));
    throw std::runtime_error("Unsupported encoding_version");
  }
  encoding_version_ = static_cast<uint8_t>(encoding_version);

  input_topic_ = this->get_parameter("topic_input").as_string();
  if (input_topic_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Input topic is not set");
    throw std::runtime_error("Input topic is not set");
  }
  output_topic_ = this->get_parameter("topic_output").as_string();
  if (output_topic_.empty()) {
    output_topic_ = input_topic_ + (compressing_ ? "/compressed" : "/decompressed");
    RCLCPP_WARN(this->get_logger(), "Output topic is not set, using default: %s", output_topic_.c_str());
  }

  const std::string compressed_topic_type = "point_cloud_interfaces/msg/CompressedPointCloud2";
  const std::string pointcloud_topic_type = "sensor_msgs/msg/PointCloud2";
  input_topic_type_ = compressing_ ? pointcloud_topic_type : compressed_topic_type;
  output_topic_type_ = compressing_ ? compressed_topic_type : pointcloud_topic_type;

  adaptToPublishers();
  if (!input_qos_) {
    RCLCPP_INFO(this->get_logger(), "Waiting for a publisher on topic '%s'", input_topic_.c_str());
  }
  graph_poll_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this] { adaptToPublishers(); });
}

void CloudiniPointcloudConverter::adaptToPublishers() {
  const auto publishers = this->get_publishers_info_by_topic(input_topic_);
  if (publishers.empty()) {
    return;  // nothing to adapt to: keep the current subscription, if any
  }
  const rclcpp::QoS qos = adapt_request_to_offers(input_topic_, publishers);
  if (input_qos_ && input_qos_->reliability() == qos.reliability() && input_qos_->durability() == qos.durability()) {
    return;
  }
  const bool reliable = qos.reliability() == rclcpp::ReliabilityPolicy::Reliable;
  const bool transient_local = qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
  RCLCPP_INFO(
      this->get_logger(), "Subscribing to topic '%s' of type '%s' (%s, %s)", input_topic_.c_str(),
      input_topic_type_.c_str(), reliable ? "RELIABLE" : "BEST_EFFORT", transient_local ? "TRANSIENT_LOCAL" : "VOLATILE");
  point_cloud_subscriber_ = this->create_generic_subscription(
      input_topic_, input_topic_type_, qos,
      std::bind(&CloudiniPointcloudConverter::callback, this, std::placeholders::_1));
  input_qos_ = qos;

  if (!point_cloud_publisher_) {  // its QoS stays, so that the subscribers of the output keep matching
    RCLCPP_INFO(
        this->get_logger(), "Publishing to topic '%s' of type '%s'", output_topic_.c_str(), output_topic_type_.c_str());
    point_cloud_publisher_ = this->create_generic_publisher(output_topic_, output_topic_type_, qos);
  }
}

void CloudiniPointcloudConverter::callback(std::shared_ptr<rclcpp::SerializedMessage> msg) {
  // Skip processing if there are no subscribers
  if (!point_cloud_publisher_ || point_cloud_publisher_->get_subscription_count() == 0) {
    return;
  }

  // STEP 1: convert the buffer to a ConstBufferView (this is not a copy)
  const auto& input_msg = msg->get_rcl_serialized_message();
  const Cloudini::ConstBufferView raw_dds_msg(input_msg.buffer, input_msg.buffer_length);

  // STEP 2: extract information from the raw DDS message
  auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(raw_dds_msg);

  if (compressing_) {
    cloudini_ros::applyResolutionProfile(cloudini_ros::ResolutionProfile{}, pc_info.fields, resolution_);
    auto encoding_info = cloudini_ros::toEncodingInfo(pc_info);
    encoding_info.version = encoding_version_;
    cloudini_ros::convertPointCloud2ToCompressedCloud(pc_info, encoding_info, output_raw_message_, &encoder_cache_);
  } else {
    cloudini_ros::convertCompressedCloudToPointCloud2(pc_info, output_raw_message_);
  }

  // STEP 3: publish the output message
  output_message_.get_rcl_serialized_message().buffer_length = output_raw_message_.size();
  output_message_.get_rcl_serialized_message().buffer = output_raw_message_.data();
  point_cloud_publisher_->publish(output_message_);

  if (!this->get_parameter("log_compression_stats").as_bool()) {
    return;
  }

  tot_original_size += input_msg.buffer_length;
  tot_compressed_size += output_raw_message_.size();

  static int count = 0;
  if (count % 20 == 0) {
    double average_ratio = static_cast<double>(tot_compressed_size) / tot_original_size;
    tot_compressed_size = 0;
    tot_original_size = 0;
    RCLCPP_INFO(this->get_logger(), "Converted %d messages, average compression ratio: %.2f", count, average_ratio);
  }
  count++;
}

RCLCPP_COMPONENTS_REGISTER_NODE(CloudiniPointcloudConverter)
