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

#pragma once

#include <cloudini_lib/cloudini.hpp>
#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace Cloudini {

/**
 * @bried Convert a PointCloud2 message to EncodingInfo
 * Default options (that can be overwitten later) are:
 * - encoding_opt = LOSSY
 * - compression_opt = ZSTD
 *
 * @param msg The PointCloud2 message to convert
 * @param resolution The resolution to use for FLOAT32 fields (XYZ, XYZI).
 * @return The EncodingInfo structure
 */
EncodingInfo ConvertToEncodingInfo(const sensor_msgs::msg::PointCloud2& msg, float resolution);

EncodingInfo ReadEncodingInfo(const point_cloud_interfaces::msg::CompressedPointCloud2& msg);

}  // namespace Cloudini
