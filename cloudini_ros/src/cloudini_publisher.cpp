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

#include "cloudini_plugin/cloudini_publisher.hpp"

#include <string>

#include "cloudini_ros/conversion_utils.hpp"

namespace cloudini_point_cloud_transport {

CloudiniPublisher::CloudiniPublisher() {}

void CloudiniPublisher::declareParameters(const std::string& base_topic) {
  rcl_interfaces::msg::ParameterDescriptor encode_resolution_descriptor;
  encode_resolution_descriptor.name = "clodini_resolution";
  encode_resolution_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  encode_resolution_descriptor.description = "resolution of floating points fields (XYZ) in meters";

  encode_resolution_descriptor.set__integer_range({rcl_interfaces::msg::IntegerRange().set__to_value(0.001)});

  declareParam<double>(encode_resolution_descriptor.name, resolution_, encode_resolution_descriptor);

  getParam<double>(encode_resolution_descriptor.name, resolution_);

  auto param_change_callback = [this](const std::vector<rclcpp::Parameter>& parameters) {
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

CloudiniPublisher::TypedEncodeResult CloudiniPublisher::encodeTyped(const sensor_msgs::msg::PointCloud2& raw) const {
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
