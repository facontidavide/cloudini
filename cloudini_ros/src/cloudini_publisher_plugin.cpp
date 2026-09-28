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

#include "cloudini_plugin/cloudini_publisher_plugin.hpp"

#include <string>

#include "cloudini_ros/conversion_utils.hpp"

namespace cloudini_point_cloud_transport {

CloudiniPublisher::CloudiniPublisher() {}

void CloudiniPublisher::declareParameters(const std::string& base_topic) {
  rcl_interfaces::msg::ParameterDescriptor encode_resolution_descriptor;
  encode_resolution_descriptor.name = "cloudini_resolution";
  encode_resolution_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  encode_resolution_descriptor.description = "resolution of floating points fields (XYZ) in meters";

  encode_resolution_descriptor.set__integer_range({rcl_interfaces::msg::IntegerRange().set__to_value(0.001)});

  declareParam<double>(encode_resolution_descriptor.name, resolution_, encode_resolution_descriptor);

  getParam<double>(encode_resolution_descriptor.name, resolution_);

  rcl_interfaces::msg::ParameterDescriptor encoding_version_descriptor;
  encoding_version_descriptor.name = "cloudini_encoding_version";
  encoding_version_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  encoding_version_descriptor.description =
      "Cloudini wire version: 6 (default) or 5 (larger, for decoders from 1.3.1 and earlier)";
  declareParam<int64_t>(encoding_version_descriptor.name, encoding_version_, encoding_version_descriptor);
  // declareParam drops the descriptor, so a range in it would not be enforced: check the value here
  const auto valid_version = [](int64_t v) {
    return v >= Cloudini::kMinEncodingVersion && v <= Cloudini::kMaxEncodingVersion;
  };
  const std::string version_range =
      std::to_string(Cloudini::kMinEncodingVersion) + " to " + std::to_string(Cloudini::kMaxEncodingVersion);
  int64_t encoding_version = encoding_version_;
  getParam<int64_t>(encoding_version_descriptor.name, encoding_version);
  if (valid_version(encoding_version)) {
    encoding_version_ = encoding_version;
  } else {
    RCLCPP_ERROR(
        getLogger(), "cloudini_encoding_version must be %s (got %ld), using %ld", version_range.c_str(),
        static_cast<long>(encoding_version), static_cast<long>(encoding_version_));
  }

  auto param_change_callback = [this, valid_version, version_range](const std::vector<rclcpp::Parameter>& parameters) {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    for (auto parameter : parameters) {
      if (parameter.get_name().find("cloudini_resolution") != std::string::npos) {
        resolution_ = parameter.as_double();
      } else if (parameter.get_name().find("cloudini_encoding_version") != std::string::npos) {
        if (!valid_version(parameter.as_int())) {
          result.successful = false;
          result.reason = "cloudini_encoding_version must be " + version_range;
          return result;
        }
        encoding_version_ = parameter.as_int();
      }
    }
    return result;
  };
  setParamCallback(param_change_callback);
}

CloudiniPublisher::TypedEncodeResult CloudiniPublisher::encodeTyped(const sensor_msgs::msg::PointCloud2& raw) const {
  auto info = Cloudini::ConvertToEncodingInfo(raw, resolution_);
  info.version = static_cast<uint8_t>(encoding_version_);
  Cloudini::RefineResolutionsToData(info, Cloudini::ConstBufferView(raw.data.data(), raw.data.size()));
  std::lock_guard<std::mutex> lock(encoder_mutex_);
  Cloudini::PointcloudEncoder& encoder = encoder_cache_.get(info);

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

  // encode() sizes compressed_data to the bytes it writes
  encoder.encode(Cloudini::ConstBufferView(raw.data.data(), raw.data.size()), result.compressed_data);
  return result;
}

}  // namespace cloudini_point_cloud_transport
