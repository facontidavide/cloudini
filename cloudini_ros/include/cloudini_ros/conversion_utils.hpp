#pragma once
#include <cloudini/cloudini.hpp>
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

}  // namespace Cloudini