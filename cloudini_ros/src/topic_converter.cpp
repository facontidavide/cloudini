#include <cloudini_lib/cloudini.hpp>
#include <cloudini_lib/ros_msg_utils.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/*
 */
class CloudiniPointcloudConverter : public rclcpp::Node {
 public:
  CloudiniPointcloudConverter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void callback(std::shared_ptr<rclcpp::SerializedMessage> msg);

 private:
  // generic subscriber for point cloud messages
  rclcpp::GenericSubscription::SharedPtr point_cloud_subscriber_;

  // generic publisher for point cloud messages
  rclcpp::GenericPublisher::SharedPtr point_cloud_publisher_;

  // callback for point cloud messages
  void point_cloud_callback(const rclcpp::SerializedMessage& serialized_msg);

  // point cloud message type support
  const rosidl_message_type_support_t* point_cloud_type_support_;

  // Buffer used to store the temporarily
  std::vector<uint8_t> decoded_pointcloud_data_;

  std::vector<uint8_t> output_raw_message_;

  Cloudini::PointcloudDecoder pointcloud_decoder_;

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
  this->declare_parameter<std::string>("input_point_cloud", "input_compressed_cloud");
  this->declare_parameter<std::string>("output_point_cloud", "output_cloud");

  // read parameters
  const std::string input_topic = this->get_parameter("input_compressed_cloud").as_string();
  const std::string output_topic = this->get_parameter("output_cloud").as_string();

  // Initialize point cloud type support
  point_cloud_type_support_ = rosidl_typesupport_cpp::get_message_type_support_handle<sensor_msgs::msg::PointCloud2>();

  auto publisher_info = this->get_publishers_info_by_topic(input_topic);
  auto detected_qos = adapt_request_to_offers(input_topic, publisher_info);

  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback =
      std::bind(&CloudiniPointcloudConverter::callback, this, std::placeholders::_1);

  const std::string compressed_topic_type = "point_cloud_interfaces/msg/CompressedPointCloud2";

  // Create a generic subscriber for point cloud messages
  point_cloud_subscriber_ = this->create_generic_subscription(
      input_topic,            //
      compressed_topic_type,  // "point_cloud_interfaces/msg/CompressedPointCloud2"
      detected_qos,           //
      callback);

  // Create a generic publisher for point cloud messages
  point_cloud_publisher_ = this->create_generic_publisher(output_topic, compressed_topic_type, detected_qos);
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
