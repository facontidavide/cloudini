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

#ifndef CLOUDINI_PLUGIN__CLOUDINI_SUBSCRIBER_HPP_
#define CLOUDINI_PLUGIN__CLOUDINI_SUBSCRIBER_HPP_

#include <memory>
#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <point_cloud_transport/simple_subscriber_plugin.hpp>
#include <point_cloud_transport/transport_hints.hpp>
#include <string>

namespace Cloudini {
class PointcloudDecoder;
}

namespace cloudini_point_cloud_transport {
class CloudiniSubscriber
    : public point_cloud_transport::SimpleSubscriberPlugin<point_cloud_interfaces::msg::CompressedPointCloud2> {
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

  DecodeResult decodeTyped(const point_cloud_interfaces::msg::CompressedPointCloud2& compressed) const override;

 private:
  std::shared_ptr<Cloudini::PointcloudDecoder> decoder_;
};

}  // namespace cloudini_point_cloud_transport

#endif  // CLOUDINI_PLUGIN__CLOUDINI_SUBSCRIBER_HPP_
