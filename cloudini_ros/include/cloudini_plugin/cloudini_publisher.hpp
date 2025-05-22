#pragma once

#include <memory>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>

#include <point_cloud_transport/simple_publisher_plugin.hpp>
#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>


namespace cloudini_point_cloud_transport
{

class CloudiniPublisher
  : public point_cloud_transport::SimplePublisherPlugin<
    point_cloud_interfaces::msg::CompressedPointCloud2>
{
public:
  CloudiniPublisher();

  std::string getTransportName() const override {
    return "cloudini";
  }

  void declareParameters(const std::string & base_topic) override;

  std::string getDataType() const override {
    return "point_cloud_interfaces/msg/CompressedPointCloud2";
  }

  TypedEncodeResult encodeTyped(const sensor_msgs::msg::PointCloud2& raw) const override;

private:
    double resolution_ = 0.001;
};

}  // namespace cloudini_point_cloud_transport