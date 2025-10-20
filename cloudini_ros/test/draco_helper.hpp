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

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "cloudini_lib/cloudini.hpp"
#include "rosbag_benchmark.hpp"

#ifdef DRACO_FOUND

/**
 * Compress a ROS PointCloud2 message using Draco encoding.
 *
 * Uses 14-bit quantization for position (XYZ) and 10-bit for other fields.
 * Uses sequential encoding method.
 *
 * @param msg The ROS PointCloud2 message to compress
 * @param encoding_info The encoding information with field metadata
 * @param stats Output statistics (compression time and ratio)
 */
void compressDraco(
    const sensor_msgs::msg::PointCloud2& msg, const Cloudini::EncodingInfo& encoding_info, StatsData& stats);

#endif  // DRACO_FOUND
