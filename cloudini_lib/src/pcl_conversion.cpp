#include "cloudini/pcl_conversion.hpp"

namespace Cloudini {

EncodingInfo ConvertToEncodingInfo(  //
    const pcl::PCLPointCloud2& cloud, double resolution_XYZ) {
  EncodingInfo info;

  info.width = cloud.width;
  info.height = cloud.height;
  info.point_step = cloud.point_step;

  const bool starts_with_XYZ =
      (cloud.fields.size() >= 3) &&
      (cloud.fields[0].name == "x" && cloud.fields[0].datatype == pcl::PCLPointField::FLOAT32) &&
      (cloud.fields[1].name == "y" && cloud.fields[1].datatype == pcl::PCLPointField::FLOAT32) &&
      (cloud.fields[2].name == "z" && cloud.fields[2].datatype == pcl::PCLPointField::FLOAT32);

  const bool is_XYZI = starts_with_XYZ && (cloud.fields.size() >= 4) &&
                       (cloud.fields[3].name == "intensity" && cloud.fields[3].datatype == pcl::PCLPointField::FLOAT32);

  size_t index = 0;

  if (is_XYZI) {
    info.fields.push_back(PointField{"XYZI", 0, FieldType::POINT_XYZI, resolution_XYZ});
    index = 4;
  } else if (starts_with_XYZ) {
    info.fields.push_back(PointField{"XYZ", 0, FieldType::POSITION_XYZ, resolution_XYZ});
    index = 3;
  }

  // remaining fields
  while (index < cloud.fields.size()) {
    const pcl::PCLPointField& field = cloud.fields[index];

    PointField point_field;
    point_field.name = field.name;
    point_field.offset = field.offset;
    point_field.type = static_cast<FieldType>(field.datatype);
    info.fields.push_back(point_field);
    index++;
  }

  return info;
}

template <>
EncodingInfo ConvertToEncodingInfo<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>& cloud, double resolution_XYZ) {
  EncodingInfo info;
  info.width = cloud.width;
  info.height = cloud.height;
  info.point_step = sizeof(pcl::PointXYZ);
  info.fields.push_back(PointField{"XYZ", 0, FieldType::POSITION_XYZ, resolution_XYZ});
  return info;
}

template <>
EncodingInfo ConvertToEncodingInfo<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>& cloud, double resolution_XYZ) {
  EncodingInfo info;
  info.width = cloud.width;
  info.height = cloud.height;
  info.point_step = sizeof(pcl::PointXYZI);
  info.fields.push_back(PointField{"XYZI", 0, FieldType::POINT_XYZI, resolution_XYZ});
  return info;
}

template <>
EncodingInfo ConvertToEncodingInfo<pcl::PointXYZRGBA>(
    const pcl::PointCloud<pcl::PointXYZRGBA>& cloud, double resolution_XYZ) {
  EncodingInfo info;
  info.width = cloud.width;
  info.height = cloud.height;
  info.point_step = sizeof(pcl::PointXYZRGBA);
  info.fields.push_back(PointField{"XYZ", 0, FieldType::POSITION_XYZ, resolution_XYZ});
  info.fields.push_back(PointField{"RGBA", 16, FieldType::UINT32, std::nullopt});
  return info;
}

template <>
EncodingInfo ConvertToEncodingInfo<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>& cloud, double resolution_XYZ) {
  EncodingInfo info;
  info.width = cloud.width;
  info.height = cloud.height;
  info.point_step = sizeof(pcl::PointXYZRGB);
  info.fields.push_back(PointField{"XYZ", 0, FieldType::POSITION_XYZ, resolution_XYZ});
  info.fields.push_back(PointField{"RGB", 16, FieldType::UINT32, std::nullopt});
  return info;
}

template <>
EncodingInfo ConvertToEncodingInfo<pcl::Normal>(const pcl::PointCloud<pcl::Normal>& cloud, double resolution_XYZ) {
  EncodingInfo info;
  info.width = cloud.width;
  info.height = cloud.height;
  info.point_step = sizeof(pcl::Normal);
  info.fields.push_back(PointField{"NORMAL", 0, FieldType::FLOAT32, resolution_XYZ});
  return info;
}

}  // namespace Cloudini