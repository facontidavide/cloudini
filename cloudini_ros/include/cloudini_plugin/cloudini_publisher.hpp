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

#ifndef CLOUDINI_PLUGIN__CLOUDINI_PUBLISHER_HPP_
#define CLOUDINI_PLUGIN__CLOUDINI_PUBLISHER_HPP_

#include <memory>
#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/simple_publisher_plugin.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

namespace cloudini_point_cloud_transport {

class CloudiniPublisher
    : public point_cloud_transport::SimplePublisherPlugin<point_cloud_interfaces::msg::CompressedPointCloud2> {
 public:
  CloudiniPublisher();

  std::string getTransportName() const override {
    return "cloudini";
  }

  void declareParameters(const std::string& base_topic) override;

  std::string getDataType() const override {
    return "point_cloud_interfaces/msg/CompressedPointCloud2";
  }

  TypedEncodeResult encodeTyped(const sensor_msgs::msg::PointCloud2& raw) const override;

 private:
  double resolution_ = 0.001;
};

}  // namespace cloudini_point_cloud_transport

#endif  // CLOUDINI_PLUGIN__CLOUDINI_PUBLISHER_HPP_
