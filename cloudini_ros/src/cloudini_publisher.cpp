// Copyright (c) 2023 John D'Angelo
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

#include <string>

#include "cloudini_plugin/cloudini_publisher.hpp"

#include "cloudini_ros/conversion_utils.hpp"

namespace cloudini_point_cloud_transport
{

CloudiniPublisher::CloudiniPublisher() {}

void CloudiniPublisher::declareParameters(const std::string & base_topic)
{
  rcl_interfaces::msg::ParameterDescriptor encode_resolution_descriptor;
  encode_resolution_descriptor.name = "clodini_resolution";
  encode_resolution_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  encode_resolution_descriptor.description =
    "resolution of floating points fields (XYZ) in meters";

  encode_resolution_descriptor.set__integer_range(
    {rcl_interfaces::msg::IntegerRange().set__to_value(0.001)});

  declareParam<double>(
      encode_resolution_descriptor.name, resolution_, encode_resolution_descriptor);

  getParam<double>(encode_resolution_descriptor.name, resolution_);

  auto param_change_callback = [this](const std::vector<rclcpp::Parameter> & parameters) {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        if (parameter.get_name().find("cloudini_resolution") != std::string::npos) {
          resolution_ = parameter.as_double();
          return result;
        }
      }
      return result;
    };
  setParamCallback(param_change_callback);
}

CloudiniPublisher::TypedEncodeResult CloudiniPublisher::encodeTyped(
  const sensor_msgs::msg::PointCloud2 & raw) const
{
  auto info = Cloudini::ConvertToEncodingInfo(raw, resolution_);
  Cloudini::PointcloudEncoder encoder(info);

  // copy all the fields from the raw point cloud to the compressed one
  point_cloud_interfaces::msg::CompressedPointCloud2 result;

  result.header = raw.header;
  result.width = raw.width;
  result.height = raw.height;
  result.fields = raw.fields;
  result.is_bigendian = false;
  result.point_step = raw.point_step;
  result.row_step = raw.row_step;
  result.is_dense = raw.is_dense;

  // reserve memory for the compressed data
  result.compressed_data.resize(raw.data.size());

  // prepare buffer for compression
  Cloudini::ConstBufferView input(raw.data.data(), raw.data.size());
  auto new_size = encoder.encode(input, result.compressed_data);

  // resize the compressed data to the actual size
  result.compressed_data.resize(new_size);
  return result;
}

}  // namespace cloudini_point_cloud_transport
