// Copyright (c) 2025 Davide Faconti
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef CLOUDINI_PLUGIN__CLOUDINI_SUBSCRIBER_HPP_
#define CLOUDINI_PLUGIN__CLOUDINI_SUBSCRIBER_HPP_

#include <memory>
#include <string>

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
    point_cloud_interfaces::msg::CompressedPointCloud2> {
public:
  CloudiniSubscriber();

  std::string getTransportName() const override {return "cloudini";}

  void declareParameters() override
  {
    // no parameters for the time being. Might be changed in the future
  }

  std::string getDataType() const override
  {
    return "point_cloud_interfaces/msg/CompressedPointCloud2";
  }

  DecodeResult decodeTyped(
    const point_cloud_interfaces::msg::CompressedPointCloud2 & compressed)
  const override;

private:
  std::shared_ptr<Cloudini::PointcloudDecoder> decoder_;
};

}  // namespace cloudini_point_cloud_transport

#endif  // CLOUDINI_PLUGIN__CLOUDINI_SUBSCRIBER_HPP_
