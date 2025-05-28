#pragma once

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/contrib/nanocdr.hpp"

namespace Cloudini {

// sensor_msgs::msg::PointCloud2:
//
//     std_msgs/Header header
//     uint32 height
//     uint32 width
//     PointField[] fields
//     bool    is_bigendian
//     uint32  point_step
//     uint32  row_step
//     uint8[] data
//     bool is_dense
//
// sensor_msgs::msg::PointField:
//
//     string name
//     uint32 offset
//     uint8  datatype
//     uint32 count
//
// point_cloud_interfaces::msg::CompressedPointCloud2
//
//     std_msgs/Header header
//     uint32 height
//     uint32 width
//     PointField[] fields
//     bool    is_bigendian
//     uint32  point_step
//     uint32  row_step
//     uint8[] compressed_data
//     bool is_dense
//     string format

struct RosHeader {
  int32_t stamp_sec = 0;    // seconds
  uint32_t stamp_nsec = 0;  // nanoseconds
  std::string frame_id;     // frame ID
};

struct RosPointCloud2 {
  nanocdr::CdrHeader cdr_header;
  RosHeader ros_header;            // ROS header
  uint32_t height = 1;             // default to unorganized point cloud
  uint32_t width = 0;              // number of points when height == 1
  std::vector<PointField> fields;  // point fields
  uint32_t point_step = 0;         // size of a single point in bytes
  uint32_t row_step = 0;           // size of a single row in bytes (not used)
  bool is_bigendian = false;       // endianness (not used)
  ConstBufferView data;
  bool is_dense = true;  // whether all points are valid
};

EncodingInfo toEncodingInfo(const RosPointCloud2& pc_info);

RosPointCloud2 readPointCloud2(ConstBufferView& raw_dds_msg);

size_t writePointCloud2(const RosPointCloud2& pc_info, std::vector<uint8_t>& raw_dds_msg, bool is_compressed);

}  // namespace Cloudini