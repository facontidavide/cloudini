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

#ifndef CLOUDINI_ROS__CONVERSION_UTILS_HPP_
#define CLOUDINI_ROS__CONVERSION_UTILS_HPP_

#include <cloudini_lib/cloudini.hpp>
#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace Cloudini
{

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
EncodingInfo ConvertToEncodingInfo(const sensor_msgs::msg::PointCloud2 & msg, float resolution);

EncodingInfo ReadEncodingInfo(const point_cloud_interfaces::msg::CompressedPointCloud2 & msg);

}  // namespace Cloudini

#endif  // CLOUDINI_ROS__CONVERSION_UTILS_HPP_
