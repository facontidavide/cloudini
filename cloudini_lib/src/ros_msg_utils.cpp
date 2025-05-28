#include "cloudini_lib/ros_msg_utils.hpp"

namespace Cloudini {

// sensor_msgs/msg/PointCloud2
//
//     uint32 height
//     uint32 width
//     PointField[] fields
//     bool    is_bigendian
//     uint32  point_step
//     uint32  row_step
//     uint8[] data
//     bool is_dense
//
// sensor_msgs/msg/PointField:
//
//     string name
//     uint32 offset
//     uint8  datatype
//     uint32 count

RosPointCloud2 readPointCloud2(ConstBufferView& raw_dds_msg) {
  nanocdr::Decoder cdr(raw_dds_msg);

  RosPointCloud2 result;

  //----- read the header -----
  cdr.decode(result.ros_header.stamp_sec);
  cdr.decode(result.ros_header.stamp_nsec);
  cdr.decode(result.ros_header.frame_id);

  //----- pointcloud info -----
  cdr.decode(result.height);
  cdr.decode(result.width);

  //----- fields -----
  uint32_t num_fields = 0;
  cdr.decode(num_fields);

  for (uint32_t i = 0; i < num_fields; ++i) {
    PointField field;
    cdr.decode(field.name);
    cdr.decode(field.offset);
    uint8_t type = 0;
    cdr.decode(type);
    field.type = static_cast<FieldType>(type);

    uint32_t count = 0;  // not used
    cdr.decode(count);

    result.fields.push_back(std::move(field));
  }
  bool is_bigendian = false;  // not used
  cdr.decode(is_bigendian);

  cdr.decode(result.point_step);
  cdr.decode(result.row_step);

  cdr.decode(result.data);
  cdr.decode(result.is_dense);

  result.cdr_header = cdr.header();
  return result;
}

size_t writePointCloud2(const RosPointCloud2& pc_info, std::vector<uint8_t>& raw_dds_msg, bool is_compressed) {
  nanocdr::Encoder cdr(pc_info.cdr_header, raw_dds_msg);

  //----- write the header -----
  cdr.encode(pc_info.ros_header.stamp_sec);   // header_stamp_sec
  cdr.encode(pc_info.ros_header.stamp_nsec);  // header_stamp_nsec
  cdr.encode(pc_info.ros_header.frame_id);    // frame_id

  //----- pointcloud info -----
  cdr.encode(pc_info.height);
  cdr.encode(pc_info.width);

  //----- fields -----
  cdr.encode(static_cast<uint32_t>(pc_info.fields.size()));
  for (const auto& field : pc_info.fields) {
    cdr.encode(field.name);
    cdr.encode(field.offset);
    cdr.encode(static_cast<uint8_t>(field.type));
    cdr.encode(static_cast<uint32_t>(1));  // count, not used
  }

  cdr.encode(false);  // is_bigendian, not used
  cdr.encode(pc_info.point_step);
  cdr.encode(static_cast<uint32_t>(pc_info.point_step * pc_info.width));

  cdr.encode(pc_info.data);

  cdr.encode(false);  // is_dense, not used

  if (is_compressed) {
    std::string format = "cloudini";
    cdr.encode(format);
  }

  return raw_dds_msg.size();
}

EncodingInfo toEncodingInfo(const RosPointCloud2& pc_info) {
  EncodingInfo info;
  info.height = pc_info.height;
  info.width = pc_info.width;
  info.point_step = pc_info.point_step;
  info.encoding_opt = EncodingOptions::LOSSY;      // default to lossy encoding
  info.compression_opt = CompressionOption::ZSTD;  // default to ZSTD compression
  info.fields = pc_info.fields;
  return info;
}

}  // namespace Cloudini