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
