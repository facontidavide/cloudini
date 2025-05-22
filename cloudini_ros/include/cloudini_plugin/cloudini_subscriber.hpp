#pragma once

#include <memory>

#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <point_cloud_transport/simple_subscriber_plugin.hpp>
#include <point_cloud_transport/transport_hints.hpp>


namespace Cloudini
{
  class PointcloudDecoder;
}

namespace cloudini_point_cloud_transport
{
class CloudiniSubscriber
  : public point_cloud_transport::SimpleSubscriberPlugin<
    point_cloud_interfaces::msg::CompressedPointCloud2>
{
public:
  CloudiniSubscriber();

  std::string getTransportName() const override {
    return "cloudini";
  }

  void declareParameters() override {
    // no parameters for the time being. Might be changed in the future
  }

  std::string getDataType() const override {
    return "point_cloud_interfaces/msg/CompressedPointCloud2";
  }

  DecodeResult decodeTyped(const point_cloud_interfaces::msg::CompressedPointCloud2 & compressed)
  const override;

private:
  std::shared_ptr<Cloudini::PointcloudDecoder> decoder_;
};

}  // namespace cloudini_point_cloud_transport